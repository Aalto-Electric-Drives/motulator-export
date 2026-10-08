/*
 * Disturbance-observer-based grid-forming control of grid converters, ported from
 * motulator.
 *
 * Python counterparts:
 *   motulator/grid/control/_gfm_observer.py  Observer,
 *                                            ObserverBasedGridFormingController
 *   motulator/grid/control/_common.py        CurrentLimiter
 *   motulator/grid/control/_base.py          GridConverterControlSystem
 *
 * Only the power-control mode (active-power and converter-voltage references) is
 * ported. The control loop follows motulator: the output function is free of side
 * effects and all states are updated in the update function, in the same order as
 * in motulator.
 */

#ifndef MOTULATOR_GFM_OBSERVER_H
#define MOTULATOR_GFM_OBSERVER_H

#include "common.h"

/* Configuration (ObserverBasedGridFormingControllerCfg). As in motulator, the
 * optional parameters default to None, represented here by NAN. */
typedef struct {
    double i_max;   /* Maximum current (A), peak value */
    double L;       /* Total inductance estimate (H) */
    double R;       /* Total series resistance estimate (Ω) */
    double R_a;     /* Active resistance (Ω), NAN = 0.25*u_nom/i_max */
    double k_v;     /* Voltage control gain, NAN = alpha_o/w_nom */
    double alpha_o; /* Observer gain (rad/s) */
    double alpha_c; /* Current control bandwidth (rad/s) */
    double u_nom;   /* Nominal grid voltage (V), line-to-neutral peak value */
    double w_nom;   /* Nominal grid angular frequency (rad/s) */
    double T_s;     /* Sampling period (s) */
    double i_d_max; /* Maximum active current (A), NAN = no power limitation */
    double alpha_l; /* Power-limitation bandwidth (rad/s) */
} GFMControllerCfg;

/* Configuration with the defaults of motulator for a given maximum current and
 * total inductance */
static GFMControllerCfg gfm_controller_cfg(double i_max, double L);

/* Measurements */
typedef struct {
    double complex i_c_ab; /* Converter current (A) */
    double u_dc;           /* DC-bus voltage (V) */
} GFMMeasurements;

/* Feedback signals (ObserverOutputs) */
typedef struct {
    double complex i_c; /* Converter current in controller coordinates */
    double complex u_c; /* Realized converter voltage */
    double complex v_c; /* Quasi-static converter voltage estimate */
    double complex u_g; /* Grid voltage estimate */
    double p_g;         /* Active power */
    double q_g;         /* Reactive power */
    double theta_c;     /* Angle of the coordinate system */
    double w_c;         /* Angular speed of the coordinate system */
    double u_dc;        /* DC-bus voltage */
} GFMFeedbacks;

/* Reference signals */
typedef struct {
    double T_s;
    double p_g;            /* Limited active power reference */
    double v_c;            /* Converter voltage magnitude reference */
    double complex i_c;    /* Converter current reference */
    double complex u_c;    /* Converter voltage reference */
    double d_abc[3];       /* Duty ratios */
    double complex u_c_ab; /* Limited converter voltage in stator coordinates */
} GFMReferences;

/* Disturbance observer (Observer) */
typedef struct {
    double alpha_o, L, R, w_g;
    /* States */
    double theta_c;
    double complex u_gp;
} GFMObserver;

static void gfm_observer_init(GFMObserver *self, double u_nom, double w_nom,
                              double alpha_o, double L, double R);
/* Feedback signals from the realized converter voltage and the converter current in
 * stationary coordinates (the field u_dc is not set) */
static void gfm_observer_compute_output(const GFMObserver *self,
                                        double complex u_c_ab, double complex i_c_ab,
                                        GFMFeedbacks *out);
static void gfm_observer_update(GFMObserver *self, double T_s, const GFMFeedbacks *out);

/* Controller (ObserverBasedGridFormingController) with the resolved gains. Its
 * compute_output is split around the CurrentLimiter: the current reference
 * (gfm_current_reference) and the voltage reference (gfm_voltage_reference). */
typedef struct {
    GFMObserver observer;
    double i_max, R_a, k_v, k_c, i_d_max, alpha_l;
    double p_max; /* State of the active-power reference limitation */
} GFMController;

/* Initialize from the configuration, resolving the defaults as
 * ObserverBasedGridFormingControllerCfg.__post_init__ */
static void gfm_controller_init(GFMController *self, const GFMControllerCfg *cfg);
/* The limited active-power reference, the voltage reference, and the current
 * reference before the CurrentLimiter */
static void gfm_current_reference(const GFMController *self, double p_g_ref,
                                  double v_c_ref, const GFMFeedbacks *fbk,
                                  GFMReferences *ref);
/* The voltage reference from the limited current reference ref->i_c */
static void gfm_voltage_reference(const GFMController *self, const GFMFeedbacks *fbk,
                                  GFMReferences *ref);
/* Update the active-power reference limitation (the observer is updated separately) */
static void gfm_power_limit_update(GFMController *self, double T_s,
                                   const GFMReferences *ref, const GFMFeedbacks *fbk);

/* Control system (GridConverterControlSystem with
 * ObserverBasedGridFormingController) */
typedef struct {
    GFMControllerCfg cfg;
    GFMController ctrl;
    PWM pwm;
    GFMFeedbacks fbk;
    GFMReferences ref;
} GFMControlSystem;

static void gfm_control_system_init(GFMControlSystem *self,
                                    const GFMControllerCfg *cfg);
static void gfm_control_system_compute_output(GFMControlSystem *self,
                                              const GFMMeasurements *meas,
                                              double p_g_ref, double v_c_ref);
static void gfm_control_system_update(GFMControlSystem *self);

/* Signal vectors of the modular control system (see sm_flux_vector.h): the
 * measurements (meas), the feedback signals (fbk) without u_dc, and the references
 * (ref) p_g, v_c, i_c, and u_c */
#define MEAS_i_c_ab 0
#define MEAS_u_dc 2
#define MEAS_WIDTH 3

#define FBK_i_c 0
#define FBK_u_c 2
#define FBK_v_c 4
#define FBK_u_g 6
#define FBK_p_g 8
#define FBK_q_g 9
#define FBK_theta_c 10
#define FBK_w_c 11
#define FBK_WIDTH 12

#define REF_p_g 0
#define REF_v_c 1
#define REF_i_c 2
#define REF_u_c 4
#define REF_WIDTH 6

static void gfm_measurements_pack(const GFMMeasurements *meas, double y[MEAS_WIDTH]);
static void gfm_measurements_unpack(const double u[MEAS_WIDTH], GFMMeasurements *meas);
static void gfm_feedbacks_pack(const GFMFeedbacks *fbk, double y[FBK_WIDTH]);
/* The field u_dc is zero */
static void gfm_feedbacks_unpack(const double u[FBK_WIDTH], GFMFeedbacks *fbk);
/* The fields p_g, v_c, i_c, and u_c, the others are zero */
static void gfm_references_unpack(const double u[REF_WIDTH], GFMReferences *ref);

#endif
