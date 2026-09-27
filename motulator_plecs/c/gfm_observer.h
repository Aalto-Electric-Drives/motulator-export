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

/* Control system (GridConverterControlSystem with
 * ObserverBasedGridFormingController) */
typedef struct {
    GFMControllerCfg cfg;
    /* Disturbance observer (Observer) */
    double w_g;
    double theta_c;      /* State */
    double complex u_gp; /* State */
    /* Controller gains */
    double R_a;
    double k_v;
    double k_c;
    double p_max; /* State of the active-power reference limitation */
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

#endif
