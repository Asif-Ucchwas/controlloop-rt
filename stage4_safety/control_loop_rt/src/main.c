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

/* Watchdog config (Task 13) */
#define WATCHDOG_CHECK_PERIOD_MS 2
#define WATCHDOG_TIMEOUT_US 2500

/* Fault injection: mid-transient, same as Task 13's "hard case" test,
 * for a direct apples-to-apples comparison of coast distance. */
#define FAULT_INJECT_START_STEP 50
#define FAULT_INJECT_DURATION_STEPS 10

/* Sensor redundancy config (Task 14) - disabled this run to isolate
 * Task 15's safe-state mechanism cleanly (no sensor fault this test) */
#define SENSOR_NOISE_AMPLITUDE 0.005
#define DISAGREEMENT_THRESHOLD 0.05
#define SENSOR_FAULT_START_STEP 999999
#define SENSOR_FAULT_DURATION_STEPS 200
#define SENSOR_FAULT_BIAS 0.5

static struct k_timer control_timer;
static struct k_sem control_sem;

static double theta = 0.0;
static double theta_dot = 0.0;
static double prev_theta_voted = 0.0;

static volatile uint32_t last_kick_cycle = 0;
static volatile bool fault_detected = false;
static int64_t fault_detected_at_step = -1;
static double theta_at_fault = 0.0;   /* for coast-distance comparison vs Task 13 */

/* Task 15: active-hold safe-state target */
static double theta_hold_target = 0.0;
static bool hold_target_captured = false;

static int disagreement_count = 0;
static uint32_t seed_a = 111;
static uint32_t seed_b = 222;

static double lcg_noise(uint32_t *seed, double amplitude)
{
    *seed = (*seed) * 1103515245 + 12345;
    double normalized = ((double)(*seed % 20001) / 10000.0) - 1.0;
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
            printk("[WATCHDOG] *** FAULT DETECTED *** no kick for %uus - "
                   "triggering ACTIVE-HOLD safe-state (not power cutoff)\n", elapsed_us);
        }
    }
}

void control_task(void)
{
    uint32_t last_cycle = 0;
    int step_count = 0;

    printk("[CONTROL] starting, priority=%d, target=%.2f rad\n",
           k_thread_priority_get(k_current_get()), TARGET);
    printk("[CONTROL] Fault injection: steps %d-%d (watchdog kick withheld)\n",
           FAULT_INJECT_START_STEP, FAULT_INJECT_START_STEP + FAULT_INJECT_DURATION_STEPS - 1);
    printk("[CONTROL] Safe-state design: ACTIVE-HOLD (re-target PD to fault-time "
           "position), not power cutoff\n\n");

    k_sem_init(&control_sem, 0, 1);
    k_timer_init(&control_timer, timer_expiry, NULL);
    k_timer_start(&control_timer, K_USEC(1000), K_USEC(1000));

    while (step_count < 3000) {
        k_sem_take(&control_sem, K_FOREVER);

        uint32_t now_cyc = k_cycle_get_32();
        last_cycle = now_cyc;

        bool injecting_fault = (step_count >= FAULT_INJECT_START_STEP &&
                                 step_count < FAULT_INJECT_START_STEP + FAULT_INJECT_DURATION_STEPS);
        if (!injecting_fault) {
            last_kick_cycle = now_cyc;
        }

        /* Sensor layer + voting (Task 14) */
        double theta_sensor_a = theta + lcg_noise(&seed_a, SENSOR_NOISE_AMPLITUDE);
        double theta_sensor_b = theta + lcg_noise(&seed_b, SENSOR_NOISE_AMPLITUDE);
        bool sensor_fault_active = (step_count >= SENSOR_FAULT_START_STEP &&
                                     step_count < SENSOR_FAULT_START_STEP + SENSOR_FAULT_DURATION_STEPS);
        if (sensor_fault_active) theta_sensor_b += SENSOR_FAULT_BIAS;

        double diff = fabs(theta_sensor_a - theta_sensor_b);
        bool sensors_agree = diff < DISAGREEMENT_THRESHOLD;
        double theta_voted = sensors_agree ? (theta_sensor_a + theta_sensor_b) / 2.0 : theta_sensor_a;
        if (!sensors_agree) disagreement_count++;

        /* --- Task 15: safe-state control law selection --- */
        double error;
        if (!fault_detected) {
            error = TARGET - theta_voted;
        } else {
            if (!hold_target_captured) {
                theta_hold_target = theta_voted;
                theta_at_fault = theta;   /* ground truth, for honest coast-distance measurement */
                hold_target_captured = true;
                printk("[SAFE-STATE] Active-hold engaged at theta_voted=%.4f "
                       "(true_theta=%.4f)\n", theta_hold_target, theta_at_fault);
            }
            error = theta_hold_target - theta_voted;
        }

        double derivative = -(theta_voted - prev_theta_voted) / DT_S;
        double u = KP * error + KD * derivative;   /* ALWAYS active control - never zeroed */
        prev_theta_voted = theta_voted;

        double theta_ddot = A21 * theta_dot + B21 * u;
        theta = theta + theta_dot * DT_S;
        theta_dot = theta_dot + theta_ddot * DT_S;

        bool near_fault = (step_count >= FAULT_INJECT_START_STEP - 2 &&
                            step_count < FAULT_INJECT_START_STEP + 60);
        if (step_count % 250 == 0 || injecting_fault ||
            (near_fault && step_count % 5 == 0)) {
            printk("[CONTROL] t=%.3fs step=%d true_theta=%.4f voted=%.4f u=%.2fV%s%s\n",
                   step_count * DT_S, step_count, theta, theta_voted, u,
                   injecting_fault ? " [WITHHOLDING KICK]" : "",
                   fault_detected ? " [ACTIVE-HOLD]" : "");
        }

        if (fault_detected && fault_detected_at_step < 0) {
            fault_detected_at_step = step_count;
        }

        step_count++;
    }

    double coast_distance = fabs(theta - theta_at_fault);
    printk("\n[CONTROL] DONE. final theta=%.4f\n", theta);
    printk("[CONTROL] Fault detected at step %lld. theta_at_fault=%.4f, final theta=%.4f\n",
           fault_detected_at_step, theta_at_fault, theta);
    printk("[CONTROL] COAST DISTANCE with active-hold: %.4f rad "
           "(Task 13's naive power-cutoff coast distance was 0.48 rad, for comparison)\n",
           coast_distance);
    printk("[CONTROL] Sensor disagreements: %d/3000\n", disagreement_count);
}

K_THREAD_DEFINE(watchdog_tid, 2048, watchdog_task, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(control_tid, 4096, control_task, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
    printk("controlloop-rt Stage 4 Task 15: active-hold safe-state transition\n\n");
    return 0;
}
