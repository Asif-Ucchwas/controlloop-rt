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

/* Watchdog config (Task 13) - kept active as a safety net, but its own
 * fault injection is disabled this run (set beyond 3000 steps) so we
 * cleanly isolate Task 14's sensor-voting fault in this test. */
#define WATCHDOG_CHECK_PERIOD_MS 2
#define WATCHDOG_TIMEOUT_US 2500
#define FAULT_INJECT_START_STEP 999999
#define FAULT_INJECT_DURATION_STEPS 10

/* Sensor redundancy config (Task 14) */
#define SENSOR_NOISE_AMPLITUDE 0.005     /* rad - realistic small encoder noise */
#define DISAGREEMENT_THRESHOLD 0.05      /* rad - beyond this, sensors "disagree" */
#define SENSOR_FAULT_START_STEP 800
#define SENSOR_FAULT_DURATION_STEPS 200  /* 200ms of sensor B reading falsely */
#define SENSOR_FAULT_BIAS 0.5            /* rad - simulated stuck/miscalibrated offset */

static struct k_timer control_timer;
static struct k_sem control_sem;

/* Ground-truth plant state (the physical reality; NOT what control_task
 * is allowed to read directly anymore - it must go through the sensors) */
static double theta = 0.0;
static double theta_dot = 0.0;

/* What the control law actually uses - derived from voted sensor data */
static double prev_theta_voted = 0.0;

static volatile uint32_t last_kick_cycle = 0;
static volatile bool fault_detected = false;
static volatile bool control_output_enabled = true;
static int64_t fault_detected_at_step = -1;

static int disagreement_count = 0;

static uint32_t seed_a = 111;
static uint32_t seed_b = 222;

static double lcg_noise(uint32_t *seed, double amplitude)
{
    *seed = (*seed) * 1103515245 + 12345;
    double normalized = ((double)(*seed % 20001) / 10000.0) - 1.0;  /* [-1, 1] */
    return normalized * amplitude;
}

static void timer_expiry(struct k_timer *timer)
{
    k_sem_give(&control_sem);
}

void watchdog_task(void)
{
    printk("[WATCHDOG] starting, priority=%d, timeout=%dus\n",
           k_thread_priority_get(k_current_get()), WATCHDOG_TIMEOUT_US);
    last_kick_cycle = k_cycle_get_32();

    while (1) {
        k_sleep(K_MSEC(WATCHDOG_CHECK_PERIOD_MS));
        uint32_t now = k_cycle_get_32();
        uint32_t elapsed_us = (uint32_t)((uint64_t)(now - last_kick_cycle) * 1000000ULL
                                          / CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
        if (elapsed_us > WATCHDOG_TIMEOUT_US && !fault_detected) {
            fault_detected = true;
            control_output_enabled = false;
            printk("[WATCHDOG] *** FAULT DETECTED *** no kick for %uus\n", elapsed_us);
        }
    }
}

void control_task(void)
{
    uint32_t last_cycle = 0;
    int step_count = 0;

    printk("[CONTROL] starting, priority=%d, target=%.2f rad\n",
           k_thread_priority_get(k_current_get()), TARGET);
    printk("[CONTROL] Dual-sensor voting active. Sensor B fault window: steps %d-%d "
           "(bias=+%.2f rad)\n\n", SENSOR_FAULT_START_STEP,
           SENSOR_FAULT_START_STEP + SENSOR_FAULT_DURATION_STEPS - 1, SENSOR_FAULT_BIAS);

    k_sem_init(&control_sem, 0, 1);
    k_timer_init(&control_timer, timer_expiry, NULL);
    k_timer_start(&control_timer, K_USEC(1000), K_USEC(1000));

    while (step_count < 3000) {
        k_sem_take(&control_sem, K_FOREVER);

        uint32_t now_cyc = k_cycle_get_32();
        last_cycle = now_cyc;

        bool injecting_kick_fault = (step_count >= FAULT_INJECT_START_STEP &&
                                      step_count < FAULT_INJECT_START_STEP + FAULT_INJECT_DURATION_STEPS);
        if (!injecting_kick_fault) {
            last_kick_cycle = now_cyc;
        }

        /* --- Sensor layer: two independent noisy readings of ground truth --- */
        double theta_sensor_a = theta + lcg_noise(&seed_a, SENSOR_NOISE_AMPLITUDE);
        double theta_sensor_b = theta + lcg_noise(&seed_b, SENSOR_NOISE_AMPLITUDE);

        bool sensor_fault_active = (step_count >= SENSOR_FAULT_START_STEP &&
                                     step_count < SENSOR_FAULT_START_STEP + SENSOR_FAULT_DURATION_STEPS);
        if (sensor_fault_active) {
            theta_sensor_b += SENSOR_FAULT_BIAS;   /* simulated stuck/miscalibrated sensor */
        }

        /* --- Voting logic --- */
        double diff = fabs(theta_sensor_a - theta_sensor_b);
        bool sensors_agree = diff < DISAGREEMENT_THRESHOLD;
        double theta_voted;
        double theta_naive_avg = (theta_sensor_a + theta_sensor_b) / 2.0;  /* comparison only */

        if (sensors_agree) {
            theta_voted = theta_naive_avg;
        } else {
            theta_voted = theta_sensor_a;   /* fall back to trusted primary channel */
            disagreement_count++;
        }

        /* --- Control law now uses VOTED sensor data, not ground truth --- */
        double error = TARGET - theta_voted;
        double derivative = -(theta_voted - prev_theta_voted) / DT_S;
        double u_computed = KP * error + KD * derivative;
        prev_theta_voted = theta_voted;

        double u = control_output_enabled ? u_computed : 0.0;

        /* --- Plant physics still evolves from ground truth (the real world) --- */
        double theta_ddot = A21 * theta_dot + B21 * u;
        theta = theta + theta_dot * DT_S;
        theta_dot = theta_dot + theta_ddot * DT_S;

        bool near_fault_window = (step_count >= SENSOR_FAULT_START_STEP - 5 &&
                                   step_count < SENSOR_FAULT_START_STEP + SENSOR_FAULT_DURATION_STEPS + 20);
        if (step_count % 250 == 0 || (near_fault_window && step_count % 10 == 0)) {
            printk("[CONTROL] t=%.3fs step=%d  true_theta=%.4f  sensA=%.4f  sensB=%.4f  "
                   "diff=%.4f  %s  voted=%.4f  naive_avg_would_be=%.4f\n",
                   step_count * DT_S, step_count, theta, theta_sensor_a, theta_sensor_b,
                   diff, sensors_agree ? "AGREE" : "DISAGREE", theta_voted, theta_naive_avg);
        }

        if (fault_detected && fault_detected_at_step < 0) {
            fault_detected_at_step = step_count;
        }

        step_count++;
    }

    printk("\n[CONTROL] DONE. final theta=%.4f\n", theta);
    printk("[CONTROL] Total sensor disagreements detected: %d / 3000 cycles\n", disagreement_count);
}

K_THREAD_DEFINE(watchdog_tid, 2048, watchdog_task, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(control_tid, 4096, control_task, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
    printk("controlloop-rt Stage 4 Task 14: dual-sensor redundancy and voting\n\n");
    return 0;
}
