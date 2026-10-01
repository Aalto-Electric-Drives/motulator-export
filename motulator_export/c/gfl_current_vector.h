/*
 * Grid-following current-vector control of grid converters, ported from motulator.
 *
 * Python counterparts:
 *   motulator/grid/control/_gfl_current_vector.py  PLL, CurrentController,
 *                                                  CurrentVectorController
 *   motulator/grid/control/_common.py              CurrentLimiter
 *   motulator/grid/control/_base.py                GridConverterControlSystem
 *   motulator/common/control/_controllers.py       ComplexPIController
 *
 * Only the power-control mode (active and reactive power references) is ported.
 * The control loop follows motulator: the output function is free of side effects
 * and all states are updated in the update function, in the same order as in
 * motulator.
 */

#ifndef MOTULATOR_GFL_CURRENT_VECTOR_H
#define MOTULATOR_GFL_CURRENT_VECTOR_H

#include "common.h"

/* Configuration (CurrentVectorControllerCfg) */
typedef struct {
    double i_max;     /* Maximum current (A), peak value */
    double L;         /* Filter inductance (H) */
    double alpha_c;   /* Current-control bandwidth (rad/s) */
    double alpha_i;   /* Integral-action bandwidth (rad/s) */
    double u_nom;     /* Nominal grid voltage (V), line-to-neutral peak value */
    double w_nom;     /* Nominal grid angular frequency (rad/s) */
    double alpha_pll; /* PLL frequency-tracking bandwidth (rad/s) */
    double T_s;       /* Sampling period (s) */
} GFLControllerCfg;

/* Configuration with the defaults of motulator for a given maximum current and
 * filter inductance */
static GFLControllerCfg gfl_controller_cfg(double i_max, double L);

/* Measurements */
typedef struct {
    double complex i_c_ab; /* Converter current (A) */
    double complex u_g_ab; /* PCC voltage (V) */
    double u_dc;           /* DC-bus voltage (V) */
} GridMeasurements;

/* Feedback signals (PLLOutputSignals) */
typedef struct {
    double complex i_c;      /* Converter current in controller coordinates */
    double complex u_c;      /* Realized converter voltage */
    double u_g;              /* Filtered PCC voltage magnitude */
    double complex u_g_meas; /* Measured PCC voltage */
    double theta_c;          /* Angle of the coordinate system */
    double w_g;              /* Grid angular frequency estimate */
    double w_c;              /* Angular speed of the coordinate system */
    double p_g;              /* Active power */
    double q_g;              /* Reactive power */
    double eps;              /* PLL error signal */
    double u_dc;             /* DC-bus voltage */
} GFLFeedbacks;

/* Reference signals */
typedef struct {
    double T_s;
    double p_g;
    double q_g;
    double complex i_c;    /* Converter current reference */
    double complex u_c;    /* Converter voltage reference */
    double d_abc[3];       /* Duty ratios */
    double complex u_c_ab; /* Limited converter voltage in stator coordinates */
} GFLReferences;

/* Control system (GridConverterControlSystem with CurrentVectorController) */
typedef struct {
    GFLControllerCfg cfg;
    ComplexPIController current_ctrl;
    double k_p_pll, k_i_pll; /* PLL gains */
    double w_g, theta_c, u_g; /* PLL states */
    PWM pwm;
    GFLFeedbacks fbk;
    GFLReferences ref;
} GFLControlSystem;

static void gfl_control_system_init(GFLControlSystem *self, const GFLControllerCfg *cfg);
static void gfl_control_system_compute_output(GFLControlSystem *self,
                                              const GridMeasurements *meas,
                                              double p_g_ref, double q_g_ref);
static void gfl_control_system_update(GFLControlSystem *self);

#endif
