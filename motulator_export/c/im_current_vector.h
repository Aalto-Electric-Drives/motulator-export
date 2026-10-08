/*
 * Current-vector control of induction machine drives, ported from motulator.
 *
 * Python counterparts:
 *   motulator/drive/utils/_parameters.py          InductionMachineInvGammaPars
 *   motulator/drive/control/_im_observers.py      FluxObserver, SpeedFluxObserver,
 *                                                 create_speed_flux_observer
 *   motulator/drive/control/_im_current_vector.py CurrentController,
 *                                                 CurrentReferenceGenerator,
 *                                                 CurrentVectorController
 *   motulator/drive/control/_base.py              VectorControlSystem
 *   motulator/drive/control/_common.py            SpeedController, SpeedObserver
 *
 * The machine model parameters are the constant inverse-Γ parameters, and the
 * default observer gains k_o of motulator are used. The control loop follows
 * motulator: the output function is free of side effects (apart from workspace
 * variables) and all states are updated in the update function, in the same order
 * as in motulator.
 */

#ifndef MOTULATOR_IM_CURRENT_VECTOR_H
#define MOTULATOR_IM_CURRENT_VECTOR_H

#include "common.h"

/* Constant inverse-Γ model parameters (InductionMachineInvGammaPars) */
typedef struct {
    double n_p;
    double R_s;
    double R_R;
    double L_sgm;
    double L_M;
} InductionMachineInvGammaPars;

/* Feedback signals for the control system (ObserverOutputs) */
typedef struct {
    double u_dc;          /* DC-bus voltage */
    double complex i_s;   /* Stator current */
    double complex u_s;   /* Stator voltage */
    double complex psi_s; /* Stator flux estimate */
    double complex psi_R; /* Rotor flux estimate */
    double tau_M;         /* Electromagnetic torque estimate */
    double tau_L;         /* Load torque estimate */
    double w_c;           /* Angular speed of the coordinate system */
    double w_s;           /* Synchronous angular frequency */
    double w_r;           /* Slip angular frequency estimate */
    double w_m;           /* Electrical angular rotor speed estimate */
    double w_M;           /* Mechanical angular rotor speed estimate */
    double theta_c;       /* Coordinate system angle */
    double complex e_o;   /* Estimation error */
    double eps;           /* Rotor speed error signal */
    double h;             /* Weight of the external speed error signal */
} IMObserverOutputs;

/* Reduced-order flux observer in synchronous coordinates (FluxObserver) */
typedef struct {
    InductionMachineInvGammaPars par;
    int sensorless; /* Selects the default observer gain k_o(w_m) */
    int unit_gain;  /* k_o = 1 (create_vhz_observer with L_M = inf) */
    /* States */
    double complex psi_s;
    double theta_c;
    /* Other memory variables */
    double T_s_old;
    double complex old_i_s;
} IMFluxObserver;

typedef struct {
    SpeedObserver speed_observer;
    IMFluxObserver flux_observer;
} IMSpeedFluxObserver;

/* Current reference generator with field weakening (CurrentReferenceGenerator) */
typedef struct {
    InductionMachineInvGammaPars par;
    double i_s_max;
    double k_u;
    double i_sd_nom;
    double k_fw;
    double i_sd_ref; /* Integral state */
} IMCurrentReferenceGenerator;

/* Current-vector controller configuration (CurrentVectorControllerCfg). As in
 * motulator, the optional parameters default to None, represented here by NAN. The
 * observer gain k_o is the default of motulator. */
typedef struct {
    double psi_s_nom;
    double i_s_max;
    double alpha_c;
    double alpha_i;
    double alpha_o;
    double w_s_nom;
    double k_u;
    double k_fw; /* NAN or 0 = default */
    double J;
    int sensorless;
    double T_s;
} IMCurrentVectorControllerCfg;

/* Current controller (CurrentController), a 2DOF PI controller with the gains from
 * the bandwidths and the leakage inductance */
static ComplexPIController
im_current_controller(const InductionMachineInvGammaPars *par, double alpha_c,
                      double alpha_i);

typedef struct {
    double T_s;
    int sensorless;
    IMCurrentReferenceGenerator reference_gen;
    ComplexPIController current_ctrl;
    IMSpeedFluxObserver observer;
} IMCurrentVectorController;

/* Reference signals */
typedef struct {
    double T_s;
    double d_abc[3];
    double w_M;
    double tau_M;
    double complex i_s;
    double complex u_s;
    double complex u_c_ab; /* Limited converter voltage from the PWM */
} IMReferences;

/* Measured signals */
typedef struct {
    double complex i_c_ab; /* Converter current in stationary coordinates */
    double u_dc;
    double w_M; /* Mechanical rotor speed, used only in the sensored mode */
} IMMeasurements;

typedef struct {
    PWM pwm;
    IMCurrentVectorController vector_ctrl;
    PIController speed_ctrl;
    /* Workspace variables passed from the output to the update function */
    IMObserverOutputs fbk;
    IMReferences ref;
} IMVectorControlSystem;

static IMCurrentVectorControllerCfg im_current_vector_controller_cfg(double psi_s_nom,
                                                                     double i_s_max);

static void im_vector_control_system_init(IMVectorControlSystem *self,
                                          InductionMachineInvGammaPars par,
                                          const IMCurrentVectorControllerCfg *cfg,
                                          PIController speed_ctrl);
static void im_vector_control_system_compute_output(IMVectorControlSystem *self,
                                                    const IMMeasurements *meas,
                                                    double w_M_ref);
static void im_vector_control_system_update(IMVectorControlSystem *self);

/* Signal vectors of the modular control system (see sm_flux_vector.h): the
 * measurements (meas), the feedback signals (fbk) without u_dc and tau_L, and the
 * references (ref) tau_M, i_s, and u_s */
#define MEAS_i_c_ab 0
#define MEAS_u_dc 2
#define MEAS_w_M 3
#define MEAS_WIDTH 4

#define FBK_i_s 0
#define FBK_u_s 2
#define FBK_psi_s 4
#define FBK_psi_R 6
#define FBK_tau_M 8
#define FBK_w_c 9
#define FBK_w_s 10
#define FBK_w_r 11
#define FBK_w_m 12
#define FBK_w_M 13
#define FBK_theta_c 14
#define FBK_e_o 15
#define FBK_eps 17
#define FBK_h 18
#define FBK_WIDTH 19

#define REF_tau_M 0
#define REF_i_s 1
#define REF_u_s 3
#define REF_WIDTH 5

static void im_measurements_pack(const IMMeasurements *meas, double y[MEAS_WIDTH]);
static void im_measurements_unpack(const double u[MEAS_WIDTH], IMMeasurements *meas);
static void im_observer_outputs_pack(const IMObserverOutputs *fbk, double y[FBK_WIDTH]);
/* The fields u_dc and tau_L are zero */
static void im_observer_outputs_unpack(const double u[FBK_WIDTH],
                                       IMObserverOutputs *fbk);
/* The fields tau_M, i_s, and u_s, the others are zero */
static void im_references_unpack(const double u[REF_WIDTH], IMReferences *ref);

#endif
