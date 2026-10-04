/*
 * Common control functions and classes, ported from motulator. See common.h.
 */

#include "common.h"

/* Utility functions -------------------------------------------------------- */

static double complex abc2complex(const double u[3])
{
    return (2.0 / 3.0) * u[0] - (u[1] + u[2]) / 3.0 + I * (u[1] - u[2]) / sqrt(3.0);
}

static void complex2abc(double complex u, double u_abc[3])
{
    u_abc[0] = creal(u);
    u_abc[1] = 0.5 * (-creal(u) + sqrt(3.0) * cimag(u));
    u_abc[2] = 0.5 * (-creal(u) - sqrt(3.0) * cimag(u));
}

static double complex line2complex(const double u[2])
{
    return (2.0 * u[0] + u[1]) / 3.0 + I * u[1] / sqrt(3.0);
}

static double wrap(double theta)
{
    /* Limit the angle into the range [-pi, pi), as numpy.mod does */
    double x = fmod(theta + M_PI, 2.0 * M_PI);
    if (x < 0.0) {
        x += 2.0 * M_PI;
    }
    return x - M_PI;
}

static double clip(double value, double min_value, double max_value)
{
    return fmax(fmin(value, max_value), min_value);
}

static double sign(double x)
{
    return (x < 0.0) ? -1.0 : ((x > 0.0) ? 1.0 : 0.0);
}

static double lut_interp(const LUT *lut, double x)
{
    int n = lut->n;
    if (x <= lut->x[0]) {
        return lut->y[0];
    }
    if (x >= lut->x[n - 1]) {
        return lut->y[n - 1];
    }
    /* Binary search for the interval x[k] <= x < x[k + 1] */
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (lut->x[mid] <= x) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    double slope = (lut->y[hi] - lut->y[lo]) / (lut->x[hi] - lut->x[lo]);
    return slope * (x - lut->x[lo]) + lut->y[lo];
}

/* Port of scipy/optimize/Zeros/brentq.c with xtol = 2e-12, rtol = 4*eps, and
 * maxiter = 100 (the defaults of scipy.optimize.brentq). The function values at the
 * bracket ends must have opposite signs (or be zero). */
static double brentq(ScalarFunction f, double xa, double xb, void *data)
{
    const double xtol = 2e-12;
    const double rtol = 8.881784197001252e-16;
    const int iter = 100;
    double xpre = xa, xcur = xb;
    double xblk = 0.0, fblk = 0.0, spre = 0.0, scur = 0.0;
    double fpre = f(xpre, data);
    double fcur = f(xcur, data);

    if (fpre == 0.0) {
        return xpre;
    }
    if (fcur == 0.0) {
        return xcur;
    }
    if (signbit(fpre) == signbit(fcur)) {
        return NAN; /* No sign change */
    }
    for (int i = 0; i < iter; i++) {
        if (fpre != 0.0 && fcur != 0.0 && signbit(fpre) != signbit(fcur)) {
            xblk = xpre;
            fblk = fpre;
            spre = scur = xcur - xpre;
        }
        if (fabs(fblk) < fabs(fcur)) {
            xpre = xcur;
            xcur = xblk;
            xblk = xpre;
            fpre = fcur;
            fcur = fblk;
            fblk = fpre;
        }
        double delta = (xtol + rtol * fabs(xcur)) / 2.0;
        double sbis = (xblk - xcur) / 2.0;
        if (fcur == 0.0 || fabs(sbis) < delta) {
            return xcur;
        }
        if (fabs(spre) > delta && fabs(fcur) < fabs(fpre)) {
            double stry;
            if (xpre == xblk) {
                /* Interpolate */
                stry = -fcur * (xcur - xpre) / (fcur - fpre);
            } else {
                /* Extrapolate */
                double dpre = (fpre - fcur) / (xpre - xcur);
                double dblk = (fblk - fcur) / (xblk - xcur);
                stry = -fcur * (fblk * dblk - fpre * dpre) / (dblk * dpre * (fblk - fpre));
            }
            if (2.0 * fabs(stry) < fmin(fabs(spre), 3.0 * fabs(sbis) - delta)) {
                /* Good short step */
                spre = scur;
                scur = stry;
            } else {
                /* Bisect */
                spre = sbis;
                scur = sbis;
            }
        } else {
            /* Bisect */
            spre = sbis;
            scur = sbis;
        }
        xpre = xcur;
        fpre = fcur;
        if (fabs(scur) > delta) {
            xcur += scur;
        } else {
            xcur += (sbis > 0.0 ? delta : -delta);
        }
        fcur = f(xcur, data);
    }
    return xcur;
}

/* PIController ------------------------------------------------------------- */

static void pi_init(PIController *self, double k_p, double k_t, double alpha_i,
                    double u_max)
{
    self->k_p = k_p;
    self->k_t = k_t;
    self->alpha_i = alpha_i;
    self->u_max = u_max;
    self->u_i = 0.0;
    self->v = 0.0;
}

static double pi_compute_output(PIController *self, double y_ref, double y,
                                double u_ff)
{
    /* Estimate of a disturbance input */
    self->v = self->u_i - (self->k_p - self->k_t) * y + u_ff;
    double u = self->k_t * (y_ref - y) + self->v;
    /* Limit the controller output */
    return clip(u, -self->u_max, self->u_max);
}

static void pi_update(PIController *self, double T_s, double u)
{
    self->u_i += T_s * self->alpha_i * (u - self->v);
}

/* ComplexPIController ------------------------------------------------------ */

static void complex_pi_init(ComplexPIController *self, double k_p, double k_i,
                            double k_t)
{
    self->k_p = k_p;
    self->k_t = k_t;
    self->alpha_i = k_i / k_t;
    self->u_i = 0.0;
    self->v = 0.0;
}

static double complex complex_pi_compute_output(ComplexPIController *self,
                                                double complex i_ref,
                                                double complex i,
                                                double complex u_ff)
{
    /* Disturbance input estimate */
    self->v = self->u_i - (self->k_p - self->k_t) * i + u_ff;
    return self->k_t * (i_ref - i) + self->v;
}

static void complex_pi_update(ComplexPIController *self, double T_s,
                              double complex u, double w_c)
{
    self->u_i += T_s * (self->alpha_i + I * w_c) * (u - self->v);
}

/* SpeedController ---------------------------------------------------------- */

static PIController speed_controller(double J, double alpha_s, double alpha_i,
                                     double tau_M_max)
{
    PIController ctrl;
    alpha_i = isnan(alpha_i) ? alpha_s : alpha_i;
    double k_p = (alpha_s + alpha_i) * J;
    double k_i = alpha_s * alpha_i * J;
    double k_t = alpha_s * J;
    pi_init(&ctrl, k_p, k_t, k_i / k_t, tau_M_max);
    return ctrl;
}

/* SpeedObserver ------------------------------------------------------------ */

static void speed_observer_init(SpeedObserver *self, double k_w, double k_tau,
                                double J)
{
    self->k_w = k_w;
    self->k_tau = k_tau;
    self->J = J;
    self->w_M = 0.0;
    self->tau_L = 0.0;
}

static void speed_observer_update(SpeedObserver *self, double T_s, double eps,
                                  double tau_M)
{
    double d_w_M, d_tau_L;
    if (self->J <= 0.0) {
        d_w_M = self->k_w * eps;
        d_tau_L = 0.0;
    } else {
        d_w_M = (tau_M - self->tau_L) / self->J + self->k_w * eps;
        d_tau_L = -self->k_tau * eps;
    }
    self->w_M += T_s * d_w_M;
    self->tau_L += T_s * d_tau_L;
}

/* Dead time --------------------------------------------------------------- */

static void dead_time_error(const double i_abc[3], const double d_abc[3], double t_d,
                            double T_s, double i_0, double d_err[3])
{
    for (int k = 0; k < 3; k++) {
        /* Switching-cycle averaged on-times of the upper and lower switches
         * (averaged_gate_signals) */
        double d = d_abc[k];
        double delta = (d > 0.0 && d < 1.0) ? t_d / (2.0 * T_s) : 0.0;
        double q_hi = fmax(d - delta, 0.0);
        double q_lo = fmax(1.0 - d - delta, 0.0);
        double b = 1.0 - q_hi - q_lo;
        /* Current direction: tanh(i/i_0), or the signum function if i_0 = 0 */
        double s = (i_0 > 0.0) ? tanh(i_abc[k] / i_0) : sign(i_abc[k]);
        d_err[k] = d - q_hi - 0.5 * b * (1.0 - s);
    }
}

/* PWM ---------------------------------------------------------------------- */

static void pwm_init(PWM *self, double k_comp)
{
    self->k_comp = k_comp;
    self->t_d = 0.0;
    self->T_s = 0.0;
    self->i_0 = 0.0;
    self->feedforward = 1;
    self->realized_voltage = 0.0;
    self->old_u_c_ab = 0.0;
    for (int k = 0; k < 3; k++) {
        self->d_abc[0][k] = 0.0;
        self->d_abc[1][k] = 0.0;
    }
}

static void pwm_set_dead_time(PWM *self, double t_d, double T_s, double i_0,
                              int feedforward)
{
    self->t_d = t_d;
    self->T_s = T_s;
    self->i_0 = i_0;
    self->feedforward = feedforward;
}

static double complex pwm_realized_voltage(const PWM *self, double complex i_c_ab,
                                           double u_dc)
{
    if (self->t_d <= 0.0) {
        return self->realized_voltage;
    }
    double i_abc[3], e0[3], e1[3], d_err[3];
    complex2abc(i_c_ab, i_abc);
    dead_time_error(i_abc, self->d_abc[0], self->t_d, self->T_s, self->i_0, e0);
    dead_time_error(i_abc, self->d_abc[1], self->t_d, self->T_s, self->i_0, e1);
    for (int k = 0; k < 3; k++) {
        d_err[k] = 0.5 * (e0[k] + e1[k]);
    }
    return self->realized_voltage - u_dc * abc2complex(d_err);
}

static void pwm_duty_ratios(double complex u_c_ref_ab, double u_dc, double d_abc[3])
{
    double u_abc[3];
    complex2abc(u_c_ref_ab, u_abc);

    /* Zero-sequence voltage resulting in space-vector PWM */
    double u_max = fmax(fmax(u_abc[0], u_abc[1]), u_abc[2]);
    double u_min = fmin(fmin(u_abc[0], u_abc[1]), u_abc[2]);
    double u_0 = 0.5 * (u_max + u_min);
    for (int k = 0; k < 3; k++) {
        u_abc[k] -= u_0;
    }

    /* MPE overmodulation */
    double m = (2.0 / u_dc) * fmax(fmax(u_abc[0], u_abc[1]), u_abc[2]);
    if (m > 1.0) {
        for (int k = 0; k < 3; k++) {
            u_abc[k] = u_abc[k] / m;
        }
    }

    /* Duty ratios */
    for (int k = 0; k < 3; k++) {
        d_abc[k] = fmax(fmin(u_abc[k] / u_dc + 0.5, 1.0), 0.0);
    }
}

static double complex pwm_compute_output(const PWM *self, double T_s,
                                         double complex u_c_ref_ab, double u_dc,
                                         double w, double complex i_c_ab,
                                         double d_abc[3])
{
    /* Advance the angle due to the computational and ZOH (PWM) delays */
    double theta_comp = self->k_comp * T_s * w;
    u_c_ref_ab = cexp(I * theta_comp) * u_c_ref_ab;

    /* Duty ratios */
    pwm_duty_ratios(u_c_ref_ab, u_dc, d_abc);

    /* Compensate for the duty-ratio error using the predicted currents */
    if (self->t_d > 0.0 && self->feedforward) {
        double i_c_abc[3], d_err[3];
        complex2abc(cexp(I * theta_comp) * i_c_ab, i_c_abc);
        dead_time_error(i_c_abc, d_abc, self->t_d, self->T_s, self->i_0,
                        d_err);
        for (int k = 0; k < 3; k++) {
            d_abc[k] = clip(d_abc[k] + d_err[k], 0.0, 1.0);
        }
    }

    /* Limited voltage reference, including the compensation */
    return abc2complex(d_abc) * u_dc;
}

static void pwm_update(PWM *self, double complex u_c_ab, const double d_abc[3])
{
    self->realized_voltage = 0.5 * (self->old_u_c_ab + u_c_ab);
    self->old_u_c_ab = u_c_ab;
    for (int k = 0; k < 3; k++) {
        self->d_abc[0][k] = self->d_abc[1][k];
        self->d_abc[1][k] = d_abc[k];
    }
}
