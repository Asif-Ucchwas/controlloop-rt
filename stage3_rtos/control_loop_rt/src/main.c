#include <zephyr/kernel.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

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

static struct k_timer control_timer;
static struct k_sem control_sem;

static double theta = 0.0;
static double theta_dot = 0.0;
static double prev_theta = 0.0;

/* Cycle-resolution jitter stats (1us resolution, vs. 100us tick resolution) */
static uint32_t min_period_cyc = 0xFFFFFFFF;
static uint32_t max_period_cyc = 0;
static uint64_t total_period_cyc = 0;
static int period_count = 0;
static int deadline_miss_count = 0;   /* periods > 1000us */

static void timer_expiry(struct k_timer *timer)
{
    k_sem_give(&control_sem);
}

void control_task(void)
{
    uint32_t last_cycle = 0;
    int step_count = 0;

    printk("[CONTROL] starting, priority=%d, measuring at cycle resolution (%d Hz)\n",
           k_thread_priority_get(k_current_get()), CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

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

        if (step_count > 0) {
            if (period_cyc < min_period_cyc) min_period_cyc = period_cyc;
            if (period_cyc > max_period_cyc) max_period_cyc = period_cyc;
            total_period_cyc += period_cyc;
            period_count++;
            if (period_us > 1000) deadline_miss_count++;
        }

        double error = TARGET - theta;
        double derivative = -(theta - prev_theta) / DT_S;
        double u = KP * error + KD * derivative;
        prev_theta = theta;

        double theta_ddot = A21 * theta_dot + B21 * u;
        theta = theta + theta_dot * DT_S;
        theta_dot = theta_dot + theta_ddot * DT_S;

        if (step_count % 500 == 0) {
            printk("[CONTROL] t=%.3fs  theta=%.4f  period=%uus (%u cycles)\n",
                   step_count * DT_S, theta, period_us, period_cyc);
        }

        step_count++;
    }

    uint32_t min_us = (uint32_t)((uint64_t)min_period_cyc * 1000000ULL
                                  / CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
    uint32_t max_us = (uint32_t)((uint64_t)max_period_cyc * 1000000ULL
                                  / CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
    uint32_t avg_us = period_count > 0 ?
        (uint32_t)((total_period_cyc / period_count) * 1000000ULL
                   / CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) : 0;
    uint32_t jitter_us = max_us - min_us;

    printk("\n[CONTROL] === TIMING REPORT (cycle-resolution, %d Hz) ===\n",
           CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
    printk("[CONTROL] periods=%d  min=%uus  max=%uus  avg=%uus  jitter(max-min)=%uus\n",
           period_count, min_us, max_us, avg_us, jitter_us);
    printk("[CONTROL] deadline misses (>1000us): %d / %d (%.2f%%)\n",
           deadline_miss_count, period_count,
           100.0 * deadline_miss_count / period_count);
    printk("[CONTROL] final theta=%.4f (target=%.2f)\n", theta, TARGET);
}

/* Heavier, VARIABLE-duration competing load - deliberately harder than
 * Task 11's fixed 2ms, to actually try to find a breaking point rather
 * than re-confirm the easy case. */
void logging_task(void)
{
    int log_count = 0;
    volatile double busy_accumulator = 0.0;
    unsigned int seed = 12345;

    printk("[LOGGING] starting, priority=%d, variable 1-4ms load per cycle\n",
           k_thread_priority_get(k_current_get()));

    while (1) {
        /* Pseudo-random 1000-4000us busy-work duration each cycle */
        seed = seed * 1103515245 + 12345;
        uint32_t work_us = 1000 + (seed % 3001);   /* 1000-4000us */

        k_busy_wait(work_us);
        for (int i = 0; i < 500; i++) {
            busy_accumulator += i * 0.0001;
        }

        log_count++;
        if (log_count % 300 == 0) {
            printk("[LOGGING] cycle %d, last work=%uus\n", log_count, work_us);
        }
    }
}

K_THREAD_DEFINE(control_tid, 4096, control_task, NULL, NULL, NULL,
                 5, 0, 0);

K_THREAD_DEFINE(logging_tid, 2048, logging_task, NULL, NULL, NULL,
                 7, 0, 0);

int main(void)
{
    printk("controlloop-rt Stage 3 Task 12: jitter measurement under variable load\n");
    printk("logging_task now does VARIABLE 1-4ms work/cycle (was fixed 2ms in Task 11)\n");
    printk("Measuring control_task timing at cycle resolution (1us), not tick resolution (100us)\n\n");
    return 0;
}
