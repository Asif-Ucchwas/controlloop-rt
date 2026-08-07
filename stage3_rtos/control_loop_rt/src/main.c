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

/* PD gains from Stage 1 pole placement (zeta=0.7, omega_n=12)
 * Section 4: Kp = omega_n^2, Kd = 2*zeta*omega_n - A21 */
#define KP 144.0
#define KD 6.79

#define DT_S 0.001   /* 1kHz control loop, matches Stage 1 */
#define TARGET 1.0

static struct k_timer control_timer;
static struct k_sem control_sem;

/* Plant state */
static double theta = 0.0;
static double theta_dot = 0.0;
static double prev_theta = 0.0;   /* for derivative-on-measurement */

static void timer_expiry(struct k_timer *timer)
{
    k_sem_give(&control_sem);
}

void control_task(void)
{
    int64_t last_uptime = 0;
    int step_count = 0;
    int64_t min_period = 1000000;
    int64_t max_period = 0;

    printk("controlloop-rt: starting PD control task, 1kHz, target=%.2f rad\n", TARGET);
    printk("Using derivative-on-measurement to avoid setpoint-change kick.\n");

    k_sem_init(&control_sem, 0, 1);
    k_timer_init(&control_timer, timer_expiry, NULL);
    k_timer_start(&control_timer, K_USEC(1000), K_USEC(1000));  /* 1000us = 1ms period */

    while (step_count < 3000) {  /* 3 seconds at 1kHz, matches Stage 1 test window */
        k_sem_take(&control_sem, K_FOREVER);

        int64_t now = k_uptime_ticks();
        int64_t actual_period_us = (step_count == 0) ? 1000 :
            (now - last_uptime) * 1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        last_uptime = now;

        if (step_count > 0) {
            if (actual_period_us < min_period) min_period = actual_period_us;
            if (actual_period_us > max_period) max_period = actual_period_us;
        }

        /* PD control law - derivative on MEASUREMENT, not error, avoids kick */
        double error = TARGET - theta;
        double derivative = -(theta - prev_theta) / DT_S;
        double u = KP * error + KD * derivative;
        prev_theta = theta;

        /* Advance plant one step - Euler integration, Section 10 of math doc */
        double theta_ddot = A21 * theta_dot + B21 * u;
        theta = theta + theta_dot * DT_S;
        theta_dot = theta_dot + theta_ddot * DT_S;

        if (step_count % 500 == 0) {
            printk("t=%.3fs  theta=%.4f  u=%.2fV  period=%lldus\n",
                   step_count * DT_S, theta, u, actual_period_us);
        }

        step_count++;
    }

    printk("controlloop-rt: final theta=%.4f (target=%.2f)\n", theta, TARGET);
    printk("Timing: min_period=%lldus  max_period=%lldus  (requested=1000us)\n",
           min_period, max_period);
}

int main(void)
{
    control_task();
    return 0;
}
