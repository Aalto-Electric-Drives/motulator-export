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

/* Phase-locked loop with the voltage-magnitude filtering (PLL) */
typedef struct {
    double k_p, k_i; /* Gains */
    double w_g, theta_c, u_g; /* States */
} PLL;

static void pll_init(PLL *self, double u_nom, double w_nom, double alpha_pll);
/* Feedback signals from the realized converter voltage, the converter current, and
 * the PCC voltage in stationary coordinates (the field u_dc is not set) */
static void pll_compute_output(const PLL *self, double complex u_c_ab,
                               double complex i_c_ab, double complex u_g_ab,
                               GFLFeedbacks *out);
static void pll_update(PLL *self, double T_s, const GFLFeedbacks *out);

/* Current controller (CurrentController), a 2DOF PI controller with the gains from
 * the bandwidths and the inductance */
static ComplexPIController gfl_current_controller(double L, double alpha_c,
                                                  double alpha_i);

/* Current reference from the power references and the filtered PCC voltage
 * magnitude u_g (CurrentVectorController.compute_output, before the
 * CurrentLimiter) */
static double complex gfl_current_reference(double p_g_ref, double q_g_ref,
                                            double u_g);

/* Control system (GridConverterControlSystem with CurrentVectorController) */
typedef struct {
    GFLControllerCfg cfg;
    ComplexPIController current_ctrl;
    PLL pll;
    PWM pwm;
    GFLFeedbacks fbk;
    GFLReferences ref;
} GFLControlSystem;

static void gfl_control_system_init(GFLControlSystem *self, const GFLControllerCfg *cfg);
static void gfl_control_system_compute_output(GFLControlSystem *self,
                                              const GridMeasurements *meas,
                                              double p_g_ref, double q_g_ref);
static void gfl_control_system_update(GFLControlSystem *self);

/* Signal vectors of the modular control system (see sm_flux_vector.h): the
 * measurements (meas), the feedback signals (fbk) without u_dc, and the references
 * (ref) p_g, q_g, i_c, and u_c */
#define MEAS_i_c_ab 0
#define MEAS_u_g_ab 2
#define MEAS_u_dc 4
#define MEAS_WIDTH 5

#define FBK_i_c 0
#define FBK_u_c 2
#define FBK_u_g 4
#define FBK_u_g_meas 5
#define FBK_theta_c 7
#define FBK_w_g 8
#define FBK_w_c 9
#define FBK_p_g 10
#define FBK_q_g 11
#define FBK_eps 12
#define FBK_WIDTH 13

#define REF_p_g 0
#define REF_q_g 1
#define REF_i_c 2
#define REF_u_c 4
#define REF_WIDTH 6

static void grid_measurements_pack(const GridMeasurements *meas, double y[MEAS_WIDTH]);
static void grid_measurements_unpack(const double u[MEAS_WIDTH],
                                     GridMeasurements *meas);
static void gfl_feedbacks_pack(const GFLFeedbacks *fbk, double y[FBK_WIDTH]);
/* The field u_dc is zero */
static void gfl_feedbacks_unpack(const double u[FBK_WIDTH], GFLFeedbacks *fbk);
/* The fields p_g, q_g, i_c, and u_c, the others are zero */
static void gfl_references_unpack(const double u[REF_WIDTH], GFLReferences *ref);

#endif
