#include <zephyr/kernel.h>
#include <stdio.h>
#include <math.h>

/* Plant parameters - same DC servo model as Stage 1
 * Full derivation: docs/notes/controls_math_reference.md, Section 1 */
#define J   0.01
#define B_F 0.1
#define K_M 0.01
#define R_A 1.0
#define A21 (-(B_F + (K_M*K_M)/R_A) / J)   /* -10.01 */
#define B21 (K_M / (R_A * J))               /* 1.0 */

/* PD gains from Stage 1 pole placement (zeta=0.7, omega_n=12) */
#define KP 144.0
#define KD 6.79

#define DT_S 0.001   /* 1kHz control loop, matches Stage 1 */
#define TARGET 1.0

static struct k_timer control_timer;
static struct k_sem control_sem;

/* Plant state - shared between control task (writer) and nothing else
 * reads it concurrently here, so no mutex needed yet (Stage 4 changes this) */
static double theta = 0.0;
static double theta_dot = 0.0;
static double prev_theta = 0.0;

/* Timing instrumentation, shared with logging task for the final report */
static int64_t min_period_us = 1000000;
static int64_t max_period_us = 0;
static int64_t total_period_us = 0;
static int period_count = 0;

static void timer_expiry(struct k_timer *timer)
{
    k_sem_give(&control_sem);
}

/* HIGH PRIORITY: the real-time control task */
void control_task(void)
{
    int64_t last_uptime = 0;
    int step_count = 0;

    printk("[CONTROL] starting, priority=%d (higher priority = lower number in Zephyr)\n",
           k_thread_priority_get(k_current_get()));

    k_sem_init(&control_sem, 0, 1);
    k_timer_init(&control_timer, timer_expiry, NULL);
    k_timer_start(&control_timer, K_USEC(1000), K_USEC(1000));

    while (step_count < 3000) {
        k_sem_take(&control_sem, K_FOREVER);

        int64_t now = k_uptime_ticks();
        int64_t period_us = (step_count == 0) ? 1000 :
            (now - last_uptime) * 1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        last_uptime = now;

        if (step_count > 0) {
            if (period_us < min_period_us) min_period_us = period_us;
            if (period_us > max_period_us) max_period_us = period_us;
            total_period_us += period_us;
            period_count++;
        }

        double error = TARGET - theta;
        double derivative = -(theta - prev_theta) / DT_S;
        double u = KP * error + KD * derivative;
        prev_theta = theta;

        double theta_ddot = A21 * theta_dot + B21 * u;
        theta = theta + theta_dot * DT_S;
        theta_dot = theta_dot + theta_ddot * DT_S;

        if (step_count % 500 == 0) {
            printk("[CONTROL] t=%.3fs  theta=%.4f  period=%lldus\n",
                   step_count * DT_S, theta, period_us);
        }

        step_count++;
    }

    int64_t avg_period_us = period_count > 0 ? total_period_us / period_count : 0;
    printk("[CONTROL] DONE. final theta=%.4f\n", theta);
    printk("[CONTROL] Timing over %d periods: min=%lldus max=%lldus avg=%lldus (requested=1000us)\n",
           period_count, min_period_us, max_period_us, avg_period_us);
}

/* LOWER PRIORITY: a logging/telemetry task doing meaningful busy-work
 * each cycle - long enough to threaten the control deadline if it
 * were NOT preempted. This is what actually tests preemption. */
void logging_task(void)
{
    int log_count = 0;
    volatile double busy_accumulator = 0.0;

    printk("[LOGGING] starting, priority=%d (lower priority = higher number)\n",
           k_thread_priority_get(k_current_get()));

    while (1) {
        /* Simulate ~2ms of real work (e.g. formatting + writing telemetry) -
         * deliberately LONGER than the control task's 1ms period, so if
         * preemption did NOT work, this task alone would already blow the
         * control loop's deadline every single cycle.
         *
         * IMPORTANT: native_sim's simulated clock only advances on kernel
         * scheduling events - a manual k_uptime_get() spin loop never
         * yields to the kernel, so simulated time never moves and this
         * would hang forever (confirmed the hard way - see DEVLOG).
         * k_busy_wait() is the correct primitive: it's a kernel call that
         * properly advances simulated time while still allowing async
         * interrupts (like our control task's timer) to be delivered. */
        k_busy_wait(2000);   /* 2000us = 2ms of simulated CPU-bound work */
        for (int i = 0; i < 1000; i++) {
            busy_accumulator += i * 0.0001;
        }

        log_count++;
        if (log_count % 200 == 0) {
            printk("[LOGGING] cycle %d complete (accumulator=%.2f)\n",
                   log_count, busy_accumulator);
        }
    }
}

K_THREAD_DEFINE(control_tid, 4096, control_task, NULL, NULL, NULL,
                 5, 0, 0);      /* priority 5 - higher priority (real-time) */

K_THREAD_DEFINE(logging_tid, 2048, logging_task, NULL, NULL, NULL,
                 7, 0, 0);      /* priority 7 - lower priority (background) */

int main(void)
{
    printk("controlloop-rt Stage 3 Task 11: preemption demo\n");
    printk("control_task (prio 5) MUST preempt logging_task (prio 7)\n");
    printk("every 1ms, even though logging_task does 2ms of busy-work per cycle.\n\n");
    return 0;
}
