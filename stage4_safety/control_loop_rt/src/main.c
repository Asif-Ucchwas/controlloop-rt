#include <zephyr/kernel.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* Plant parameters - same DC servo model as Stage 1
 * Full derivation: docs/notes/controls_math_reference.md, Section 1 */
#define J   0.01
#define B_F 0.1
#define K_M 0.01
#define R_A 1.0
#define A21 (-(B_F + (K_M*K_M)/R_A) / J)
#define B21 (K_M / (R_A * J))

#define KP 144.0
#define KD 6.79
#define DT_S 0.001
#define TARGET 1.0

/* Watchdog config */
#define WATCHDOG_CHECK_PERIOD_MS 2      /* check every 2ms (2x control period) */
#define WATCHDOG_TIMEOUT_US 2500        /* fault if no kick within 2.5ms */

/* Fault injection: deliberately withhold kicks during this window to
 * prove the watchdog actually detects a real stall, not just review code
 * that looks correct. This simulates the SYMPTOM a real watchdog catches
 * (a hung/blocked task), without needing an actual infinite loop. */
#define FAULT_INJECT_START_STEP 50
#define FAULT_INJECT_DURATION_STEPS 10  /* 10ms of withheld kicks */

static struct k_timer control_timer;
static struct k_sem control_sem;

static double theta = 0.0;
static double theta_dot = 0.0;
static double prev_theta = 0.0;

/* Watchdog shared state */
static volatile uint32_t last_kick_cycle = 0;
static volatile bool fault_detected = false;
static volatile bool control_output_enabled = true;
static int64_t fault_detected_at_step = -1;

static void timer_expiry(struct k_timer *timer)
{
    k_sem_give(&control_sem);
}

/* HIGHEST PRIORITY: independent watchdog monitor.
 * Must run at higher priority than control_task, otherwise a genuinely
 * hung control_task could also starve the watchdog itself. */
void watchdog_task(void)
{
    printk("[WATCHDOG] starting, priority=%d, checking every %dms, timeout=%dus\n",
           k_thread_priority_get(k_current_get()),
           WATCHDOG_CHECK_PERIOD_MS, WATCHDOG_TIMEOUT_US);

    last_kick_cycle = k_cycle_get_32();

    while (1) {
        k_sleep(K_MSEC(WATCHDOG_CHECK_PERIOD_MS));

        uint32_t now = k_cycle_get_32();
        uint32_t elapsed_cyc = now - last_kick_cycle;
        uint32_t elapsed_us = (uint32_t)((uint64_t)elapsed_cyc * 1000000ULL
                                          / CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

        if (elapsed_us > WATCHDOG_TIMEOUT_US && !fault_detected) {
            fault_detected = true;
            control_output_enabled = false;   /* trigger fallback: cut actuator power */
            printk("[WATCHDOG] *** FAULT DETECTED *** no kick for %uus (timeout=%dus)\n",
                   elapsed_us, WATCHDOG_TIMEOUT_US);
            printk("[WATCHDOG] Fallback triggered: control output disabled (0V, safe state)\n");
        }
    }
}

/* HIGH PRIORITY: the real-time control task */
void control_task(void)
{
    uint32_t last_cycle = 0;
    int step_count = 0;

    printk("[CONTROL] starting, priority=%d, target=%.2f rad\n",
           k_thread_priority_get(k_current_get()), TARGET);
    printk("[CONTROL] Fault injection scheduled: steps %d-%d will withhold watchdog kicks\n\n",
           FAULT_INJECT_START_STEP, FAULT_INJECT_START_STEP + FAULT_INJECT_DURATION_STEPS - 1);

    k_sem_init(&control_sem, 0, 1);
    k_timer_init(&control_timer, timer_expiry, NULL);
    k_timer_start(&control_timer, K_USEC(1000), K_USEC(1000));

    while (step_count < 3000) {
        k_sem_take(&control_sem, K_FOREVER);

        uint32_t now_cyc = k_cycle_get_32();
        uint32_t period_cyc = (step_count == 0) ? 0 : (now_cyc - last_cycle);
        last_cycle = now_cyc;
        uint32_t period_us = (uint32_t)((uint64_t)period_cyc * 1000000ULL
                                         / CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

        /* Fault injection window: skip the kick, simulating a stall */
        bool injecting_fault = (step_count >= FAULT_INJECT_START_STEP &&
                                 step_count < FAULT_INJECT_START_STEP + FAULT_INJECT_DURATION_STEPS);
        if (!injecting_fault) {
            last_kick_cycle = now_cyc;   /* normal operation: kick the watchdog */
        }

        double error = TARGET - theta;
        double derivative = -(theta - prev_theta) / DT_S;
        double u_computed = KP * error + KD * derivative;
        prev_theta = theta;

        /* Safety layer: watchdog can force output to zero regardless of
         * what the control law computed. This is the actual fallback. */
        double u = control_output_enabled ? u_computed : 0.0;

        double theta_ddot = A21 * theta_dot + B21 * u;
        theta = theta + theta_dot * DT_S;
        theta_dot = theta_dot + theta_ddot * DT_S;

        bool coast_window = (step_count >= FAULT_INJECT_START_STEP &&
                              step_count < FAULT_INJECT_START_STEP + 300);
        if (step_count % 250 == 0 || injecting_fault ||
            (fault_detected && fault_detected_at_step < 0) ||
            (coast_window && step_count % 25 == 0)) {
            printk("[CONTROL] t=%.3fs step=%d theta=%.4f u=%.2fV period=%uus%s%s\n",
                   step_count * DT_S, step_count, theta, u, period_us,
                   injecting_fault ? " [WITHHOLDING KICK]" : "",
                   fault_detected ? " [FAULT ACTIVE]" : "");
        }

        if (fault_detected && fault_detected_at_step < 0) {
            fault_detected_at_step = step_count;
        }

        step_count++;
    }

    printk("\n[CONTROL] DONE. final theta=%.4f\n", theta);
    printk("[CONTROL] Fault was detected at step %lld (injection started at step %d, "
           "detection latency = %lld steps = %lldus)\n",
           fault_detected_at_step, FAULT_INJECT_START_STEP,
           fault_detected_at_step - FAULT_INJECT_START_STEP,
           (fault_detected_at_step - FAULT_INJECT_START_STEP) * 1000LL);
}

K_THREAD_DEFINE(watchdog_tid, 2048, watchdog_task, NULL, NULL, NULL,
                 3, 0, 0);      /* priority 3 - HIGHEST, must not be starved */

K_THREAD_DEFINE(control_tid, 4096, control_task, NULL, NULL, NULL,
                 5, 0, 0);      /* priority 5 */

int main(void)
{
    printk("controlloop-rt Stage 4 Task 13: software watchdog with fault injection\n\n");
    return 0;
}
