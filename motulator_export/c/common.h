/*
 * Common control functions and classes, ported from motulator.
 *
 * Python counterparts:
 *   motulator/common/utils/_utils.py        abc2complex, complex2abc, line2complex,
 *                                           wrap, clip, sign
 *   motulator/common/control/_controllers.py PIController, ComplexPIController,
 *                                            RateLimiter
 *   motulator/common/control/_pwm.py         PWM (MPE and MME overmodulation)
 *   motulator/common/utils/_dead_time.py     dead_time_error (with numpy.sign or tanh)
 *   motulator/drive/control/_common.py       SpeedController, SpeedObserver
 *
 * Complex space vectors use peak-value scaling, as in motulator.
 */

#ifndef MOTULATOR_COMMON_H
#define MOTULATOR_COMMON_H

#include <complex.h>
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Utility functions */
static double complex abc2complex(const double u[3]);
static void complex2abc(double complex u, double u_abc[3]);
/* Space vector from the line-to-line quantities u_ab and u_bc */
static double complex line2complex(const double u[2]);
/* Complex number from its real and imaginary parts, exactly (CMPLX of C11) */
static double complex complex_from(double re, double im);
static double wrap(double theta);
static double clip(double value, double min_value, double max_value);
static double sign(double x);

/* Lookup table with linear interpolation, equivalent to numpy.interp */
#define LUT_MAX_SIZE 64

typedef struct {
    int n;
    double x[LUT_MAX_SIZE]; /* Increasing sample points */
    double y[LUT_MAX_SIZE];
} LUT;

static double lut_interp(const LUT *lut, double x);

/* Brent's root finding in a bracketing interval, as scipy.optimize.brentq with the
 * default tolerances (used via scipy.optimize.root_scalar in motulator) */
typedef double (*ScalarFunction)(double x, void *data);

static double brentq(ScalarFunction f, double xa, double xb, void *data);

/* 2DOF PI controller */
typedef struct {
    double k_p;
    double k_t;
    double alpha_i; /* Inverse of the integration time */
    double u_max;
    /* States and workspace variables */
    double u_i;
    double v;
} PIController;

static void pi_init(PIController *self, double k_p, double k_t, double alpha_i,
                    double u_max);
static double pi_compute_output(PIController *self, double y_ref, double y,
                                double u_ff);
static void pi_update(PIController *self, double T_s, double u);

/* 2DOF synchronous-frame complex-vector PI controller */
typedef struct {
    double k_p;
    double k_t;
    double alpha_i; /* Inverse of the integration time */
    /* States and workspace variables */
    double complex u_i;
    double complex v;
} ComplexPIController;

static void complex_pi_init(ComplexPIController *self, double k_p, double k_i,
                            double k_t);
static double complex complex_pi_compute_output(ComplexPIController *self,
                                                double complex i_ref,
                                                double complex i,
                                                double complex u_ff);
static void complex_pi_update(ComplexPIController *self, double T_s,
                              double complex u, double w_c);

/* Limit the magnitude of a current vector to i_max (CurrentLimiter of the grid
 * converter control) */
static double complex current_limiter(double i_max, double complex i);

/* 2DOF PI speed controller (SpeedController); alpha_i = NAN means alpha_s */
static PIController speed_controller(double J, double alpha_s, double alpha_i,
                                     double tau_M_max);

/* Speed observer (mechanical system model is used if J > 0) */
typedef struct {
    double k_w;
    double k_tau;
    double J;
    /* States */
    double w_M;
    double tau_L;
} SpeedObserver;

static void speed_observer_init(SpeedObserver *self, double k_w, double k_tau,
                                double J);
static void speed_observer_update(SpeedObserver *self, double T_s, double eps,
                                  double tau_M);

/* Rate limiter (RateLimiter), the falling rate limit being the negative rising one */
typedef struct {
    double rate_limit;
    double old_y; /* State: the previous output */
} RateLimiter;

static void rate_limiter_init(RateLimiter *self, double rate_limit);
/* Limited signal; the state is updated with it by rate_limiter_update */
static double rate_limiter_compute_output(const RateLimiter *self, double T_s,
                                          double u);
static void rate_limiter_update(RateLimiter *self, double y);

/* Duty-ratio error due to the dead time t_d (dead_time_error), T_s being the half
 * carrier period. The current direction is sign = tanh(i/i_0), or the signum
 * function (numpy.sign) if i_0 = 0. */
static void dead_time_error(const double i_abc[3], const double d_abc[3], double t_d,
                            double T_s, double i_0, double d_err[3]);

/* Space-vector PWM with the minimum-phase-error (MPE) or minimum-magnitude-error
 * (MME) overmodulation (pwm_set_overmodulation, MPE by default). Optionally,
 * the duty-ratio error due to the dead time is modeled (pwm_set_dead_time), which
 * corresponds to d_err = dead_time_error(i_abc, d_abc, t_d, T_s, sign) in motulator
 * with sign = tanh(i/i_0), or numpy.sign if i_0 = 0. */
typedef struct {
    double k_comp;
    int mme;         /* MME overmodulation (else MPE) */
    double t_d;      /* Dead time of the duty-ratio error model, 0 = no error model */
    double T_s;      /* Sampling period of the duty-ratio error model */
    double i_0;      /* Current scale of sign = tanh(i/i_0), 0 = signum function */
    int feedforward; /* Compensate for the duty-ratio error */
    double d_min;    /* Minimum duty ratio of a switching leg, 0 = no limit */
    /* States */
    double d_abc[2][3]; /* Duty ratios of the previous and the ongoing periods */
} PWM;

static void pwm_init(PWM *self, double k_comp);
static void pwm_set_dead_time(PWM *self, double t_d, double T_s, double i_0,
                              int feedforward);
/* Overmodulation: MME if mme is nonzero, else MPE (overmodulation of PWM) */
static void pwm_set_overmodulation(PWM *self, int mme);
/* Minimum duty ratio d_min of a switching leg (d_min of PWM in motulator), see
 * pwm_limit_pulses */
static void pwm_set_min_pulse(PWM *self, double d_min);
/* Limit the duty ratios of the switching legs to [d_min, 1 - d_min] (limit_pulses):
 * a duty ratio in (0, d_min) is replaced by 0 or d_min, and in (1 - d_min, 1) by
 * 1 - d_min or 1, whichever gives the realized duty ratio nearer to the reference
 * d_ref (the leg switches if equally near). The realized duty ratios are corrected
 * for the duty-ratio error at the currents i_abc, unless i_abc is NULL. */
static void pwm_limit_pulses(const PWM *self, double d_abc[3], const double d_ref[3],
                             const double *i_abc);
/* Realized voltage from the duty ratios and the measured DC-bus voltage, corrected
 * for the duty-ratio error using the measured current (get_realized_voltage): the
 * average of the previous and the ongoing sampling periods if average is nonzero,
 * else that of the ongoing sampling period */
static double complex pwm_realized_voltage(const PWM *self, double complex i_c_ab,
                                           double u_dc, int average);
/* Duty ratios and the limited voltage reference; the measured current i_c_ab is
 * used in the feedforward compensation of the duty-ratio error */
static double complex pwm_compute_output(const PWM *self, double T_s,
                                         double complex u_c_ref_ab, double u_dc,
                                         double w, double complex i_c_ab,
                                         double d_abc[3]);
/* Store the duty ratios of the next sampling period */
static void pwm_update(PWM *self, const double d_abc[3]);

#endif
