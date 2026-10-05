/*
 * Common control functions and classes, ported from motulator.
 *
 * Python counterparts:
 *   motulator/common/utils/_utils.py        abc2complex, complex2abc, line2complex,
 *                                           wrap, clip, sign
 *   motulator/common/control/_controllers.py PIController, ComplexPIController
 *   motulator/common/control/_pwm.py         PWM (MPE overmodulation)
 *   motulator/common/utils/_dead_time.py     dead_time_error (with numpy.sign or tanh)
 *   motulator/drive/control/_common.py       SpeedController, SpeedObserver
 *
 * Complex space vectors use peak-value scaling, as in motulator.
 */

#ifndef MOTULATOR_COMMON_H
#define MOTULATOR_COMMON_H

#include <complex.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Utility functions */
static double complex abc2complex(const double u[3]);
static void complex2abc(double complex u, double u_abc[3]);
/* Space vector from the line-to-line quantities u_ab and u_bc */
static double complex line2complex(const double u[2]);
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

/* Duty-ratio error due to the dead time t_d (dead_time_error), T_s being the half
 * carrier period. The current direction is sign = tanh(i/i_0), or the signum
 * function (numpy.sign) if i_0 = 0. */
static void dead_time_error(const double i_abc[3], const double d_abc[3], double t_d,
                            double T_s, double i_0, double d_err[3]);

/* Space-vector PWM with the minimum-phase-error (MPE) overmodulation. Optionally,
 * the duty-ratio error due to the dead time is modeled (pwm_set_dead_time), which
 * corresponds to d_err = dead_time_error(i_abc, d_abc, t_d, T_s, sign) in motulator
 * with sign = tanh(i/i_0), or numpy.sign if i_0 = 0. */
typedef struct {
    double k_comp;
    double k_pred;   /* Prediction factor of the currents for the feedforward */
    double t_d;      /* Dead time of the duty-ratio error model, 0 = no error model */
    double T_s;      /* Sampling period of the duty-ratio error model */
    double i_0;      /* Current scale of sign = tanh(i/i_0), 0 = signum function */
    int feedforward; /* Compensate for the duty-ratio error */
    /* States */
    double complex realized_voltage;
    double complex old_u_c_ab;
    double d_abc[2][3]; /* Duty ratios of the two latest sampling periods */
} PWM;

static void pwm_init(PWM *self, double k_comp);
static void pwm_set_dead_time(PWM *self, double t_d, double T_s, double i_0,
                              int feedforward);
/* Realized voltage, corrected for the duty-ratio error using the measured current
 * (get_realized_voltage) */
static double complex pwm_realized_voltage(const PWM *self, double complex i_c_ab,
                                           double u_dc);
/* Duty ratios and the limited voltage reference; the measured current i_c_ab is
 * used in the feedforward compensation of the duty-ratio error */
static double complex pwm_compute_output(const PWM *self, double T_s,
                                         double complex u_c_ref_ab, double u_dc,
                                         double w, double complex i_c_ab,
                                         double d_abc[3]);
static void pwm_update(PWM *self, double complex u_c_ab, const double d_abc[3]);

#endif
