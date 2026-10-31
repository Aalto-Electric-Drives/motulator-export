/*
 * Common control functions and classes, ported from motulator.
 *
 * Python counterparts:
 *   motulator/common/utils/_utils.py        abc2complex, complex2abc, line2complex,
 *                                           wrap, clip, sign
 *   motulator/common/control/_controllers.py PIController, ComplexPIController,
 *                                            DiscreteComplexController,
 *                                            RateLimiter
 *   motulator/common/control/_pwm.py         PWM (MPE and MME overmodulation),
 *                                            InverterNonlinearity (with numpy.sign
 *                                            or tanh)
 *   motulator/common/utils/_dead_time.py     averaged_gate_signals
 *   motulator/grid/control/_common.py        CurrentLimiter, PowerReferenceLimiter
 *   motulator/drive/control/_common.py       SpeedController, SpeedObserver,
 *                                            hold_equivalent, matched_pole
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
/* The same with the tolerances xtol and rtol (e.g., 4*eps of the events of
 * scipy.integrate.solve_ivp) */
static double brentq_tol(ScalarFunction f, double xa, double xb, void *data,
                         double xtol, double rtol);

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

/* Direct discrete-time 2DOF complex-vector controller, whose design includes the
 * computational delay, so that the angle of the output is not advanced (k_comp = 0
 * in the control system) */
typedef struct {
    double T_s;
    double beta; /* exp(-alpha_c*T_s) */
    /* States */
    double complex u_i;   /* Integral state */
    double complex u_lim; /* Limited output of the previous sampling period */
    double w_c;           /* Angular speed of the coordinates */
    /* Workspace variables */
    double complex e;
    double complex u;
} DiscreteComplexController;

static void discrete_complex_init(DiscreteComplexController *self, double alpha_c,
                                  double T_s);
static double complex discrete_complex_compute_output(DiscreteComplexController *self,
                                                      double complex i_ref,
                                                      double complex i);
/* Update with the limited output u, and the angular speed w_c of the coordinates */
static void discrete_complex_update(DiscreteComplexController *self, double T_s,
                                    double complex u, double w_c);
/* Preset the integral state to the steady state of the output u (preset) */
static void discrete_complex_preset(DiscreteComplexController *self, double complex u,
                                    double w_c);

/* Limit the magnitude of a current vector to i_max (CurrentLimiter of the grid
 * converter control) */
static double complex current_limiter(double i_max, double complex i);

/* Limiter of the active-power reference of the grid converter control
 * (PowerReferenceLimiter): the maximum active power is computed from the voltage
 * reference and the currents, prioritizing the reactive and the negative-sequence
 * currents, and low-pass filtered */
typedef struct {
    double i_d_max; /* Maximum active current (A), peak value */
    double alpha_l; /* Bandwidth of the low-pass filter (rad/s) */
    double p_max;   /* State: the maximum active power */
} PowerReferenceLimiter;

/* The initial maximum active power is that at the nominal grid voltage u_nom */
static void power_reference_limiter_init(PowerReferenceLimiter *self, double i_d_max,
                                         double alpha_l, double u_nom);
/* Limited active-power reference (__call__) */
static double power_reference_limiter_compute_output(const PowerReferenceLimiter *self,
                                                     double p_g_ref);
/* Update the maximum active power from the converter voltage reference u_c, the
 * converter current i_c (their positive-sequence components in unbalanced
 * conditions), and the negative-sequence converter current i_cn */
static void power_reference_limiter_update(PowerReferenceLimiter *self, double T_s,
                                           double complex u_c, double complex i_c,
                                           double complex i_cn);

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

/* Hold-equivalent discretization of a vector integrator in rotating coordinates
 * (hold_equivalent): the coefficients discretize d(psi)/dt = u + v - 1j*w_c*psi,
 * where psi is in coordinates rotating at w_c, u is constant in stator coordinates,
 * and v is constant in the rotating coordinates over the sampling period. Then,
 * psi(k+1) = Phi*(psi(k) + T_s*u(k)) + Gamma*v(k), with Phi = exp(-1j*w_c*T_s) and
 * Gamma = (1 - Phi)/(1j*w_c). */
static void hold_equivalent(double T_s, double w_c, double complex *Phi,
                            double complex *Gamma);

/* Pole for the gain formulas of estimators updated by the forward Euler method
 * (matched_pole): (1 - exp(-alpha*T_s))/T_s, placing the discrete-time poles at
 * exp(-alpha*T_s) */
static double matched_pole(double alpha, double T_s);

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

/* Duty-ratio error due to the dead time t_d and the threshold voltage u_th of the
 * power devices (InverterNonlinearity), u_dc being the DC-bus voltage and T_half the
 * half carrier period. The current direction is sign = tanh(i/i_0), or the signum
 * function (numpy.sign) if i_0 = 0. */
static void inverter_nonlinearity(double t_d, double u_th, double i_0,
                                  const double i_abc[3], const double d_abc[3],
                                  double u_dc, double T_half, double d_err[3]);

/* Space-vector PWM with the minimum-phase-error (MPE) or minimum-magnitude-error
 * (MME) overmodulation (pwm_set_overmodulation, MPE by default). Optionally, the
 * duty-ratio error due to the dead time and the threshold voltage is modeled
 * (pwm_set_nonlinearity), which corresponds to d_err = InverterNonlinearity(t_d,
 * u_th, sign) in motulator with sign = tanh(i/i_0), or numpy.sign if i_0 = 0. The
 * duty ratios are updated twice per carrier period, or once (pwm_set_single_update),
 * which sets the half carrier period of the error model. The angle of the voltage
 * reference is not advanced here: the control systems compensate for the delays. */
typedef struct {
    double k_pred;     /* Prediction factor of the currents for the compensation */
    int mme;           /* MME overmodulation (else MPE) */
    double t_d;        /* Dead time of the duty-ratio error model */
    double u_th;       /* Threshold voltage of the duty-ratio error model */
    double i_0;        /* Current scale of sign = tanh(i/i_0), 0 = signum function */
    int feedforward;   /* Compensate for the duty-ratio error */
    double d_min;      /* Minimum duty ratio of a switching leg, 0 = no limit */
    int single_update; /* Duty ratios updated once per carrier period (else twice) */
    /* States */
    double d_abc[2][3]; /* Duty ratios of the previous and the ongoing periods */
    double T_half;      /* Half carrier period of the previous period, assumed to
                         * apply also to the ongoing one (infinite at first) */
} PWM;

static void pwm_init(PWM *self, double k_pred);
/* Duty-ratio error model of the dead time t_d and the threshold voltage u_th (no
 * error model if both are zero) and its compensation */
static void pwm_set_nonlinearity(PWM *self, double t_d, double u_th, double i_0,
                                 int feedforward);
/* Overmodulation: MME if mme is nonzero, else MPE (overmodulation of PWM) */
static void pwm_set_overmodulation(PWM *self, int mme);
/* Minimum duty ratio d_min of a switching leg (d_min of PWM in motulator), see
 * pwm_limit_pulses */
static void pwm_set_min_pulse(PWM *self, double d_min);
/* Single update (single_update of PWM in motulator) if single_update is nonzero */
static void pwm_set_single_update(PWM *self, int single_update);
/* Limit the duty ratios of the switching legs to [d_min, 1 - d_min] (_limit_pulses):
 * a duty ratio in (0, d_min) is replaced by 0 or d_min, and in (1 - d_min, 1) by
 * 1 - d_min or 1, whichever gives the realized duty ratio nearer to the reference
 * d_ref (the leg switches if equally near). The realized duty ratios are corrected
 * for the duty-ratio error at the currents i_abc, the DC-bus voltage u_dc, and the
 * half carrier period T_half, unless i_abc is NULL. The output d_realized gives the
 * realized duty ratios: d_ref for the legs not changed, and the realized duty ratios
 * of the limited legs. */
static void pwm_limit_pulses(const PWM *self, double d_abc[3], const double d_ref[3],
                             const double *i_abc, double u_dc, double T_half,
                             double d_realized[3]);
/* Realized voltage from the duty ratios and the measured DC-bus voltage, corrected
 * for the duty-ratio error using the measured current and the half carrier period
 * of the previous period (get_realized_voltage): the average of the previous and the
 * ongoing sampling periods if average is nonzero, else that of the ongoing sampling
 * period. With a discharged DC bus (u_dc <= 0), the voltage is zero. */
static double complex pwm_realized_voltage(const PWM *self, double complex i_c_ab,
                                           double u_dc, int average);
/* Duty ratios and the limited voltage reference, including the compensation of the
 * duty-ratio error; the measured current i_c_ab, predicted by the angle k_pred*T_s*w,
 * is used in the feedforward compensation. Unless u_lim_ab is NULL, it gives the
 * limited voltage reference excluding the compensation, i.e., the voltage of the
 * realized duty ratios (limited_voltage). With a discharged DC bus (u_dc <= 0), the
 * duty ratios are 0.5 and the voltages zero. */
static double complex pwm_compute_output(const PWM *self, double T_s,
                                         double complex u_c_ref_ab, double u_dc,
                                         double w, double complex i_c_ab,
                                         double d_abc[3], double complex *u_lim_ab);
/* Store the duty ratios of the next sampling period and the half carrier period of
 * the sampling period T_s */
static void pwm_update(PWM *self, double T_s, const double d_abc[3]);

#endif
