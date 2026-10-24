/*
 * Sensorless flux-vector control and observer-based V/Hz control of synchronous
 * machine drives, ported from motulator.
 *
 * Python counterparts:
 *   motulator/drive/control/_sm_observers.py      FluxObserver, SpeedFluxObserver,
 *                                                 create_vhz_observer
 *   motulator/drive/control/_sm_reference_gen.py  ReferenceGenerator (LUT-based,
 *                                                 with the current reference)
 *   motulator/drive/control/_sm_flux_vector.py    FluxTorqueController,
 *                                                 FluxVectorController,
 *                                                 ObserverBasedVHzControllerCfg,
 *                                                 ObserverBasedVHzController
 *   motulator/drive/control/_base.py              DriveControlSystem,
 *                                                 VectorControlSystem,
 *                                                 VHzControlSystem
 *   motulator/common/control/_controllers.py      RateLimiter
 *   motulator/common/control/_base.py             ControlSystem (gate enable)
 *   motulator/drive/control/_common.py            FluxTorqueControllerBase,
 *                                                 SpeedController
 *
 * The machine model parameters correspond to SynchronousMachinePars, i.e., a
 * machine without magnetic saturation. The control loop follows motulator: the
 * output function is free of side effects (apart from workspace variables and the
 * flying start of the observer) and all states are updated in the update
 * function, in the same order as in motulator.
 */

#ifndef MOTULATOR_SM_FLUX_VECTOR_H
#define MOTULATOR_SM_FLUX_VECTOR_H

#include "common.h"
#include "sm_control_loci.h"

/* Feedback signals for the control system (ObserverOutputs) */
typedef struct {
    double u_dc;           /* DC-bus voltage */
    double complex i_s;    /* Stator current */
    double complex u_s;    /* Stator voltage (previous and ongoing periods), set by
                            * the control system for the control laws */
    double complex u_s_zoh; /* Stator voltage (ongoing period) */
    double complex psi_s;  /* Stator flux linkage estimate */
    double complex e_o;    /* Flux estimation error signal */
    double eps;            /* Mechanical position estimation error signal */
    double eps_f;          /* PM-flux error signal */
    double complex psi_a;  /* Auxiliary flux linkage */
    double tau_M;          /* Electromagnetic torque estimate */
    double tau_L;          /* Load torque estimate */
    double w_c;            /* Angular speed of the coordinate system */
    double w_m;            /* Electrical angular rotor speed estimate */
    double w_M;            /* Mechanical angular rotor speed estimate */
    double theta_c;        /* Coordinate system angle */
    double theta_m;        /* Electrical rotor angle estimate */
    double psi_f;          /* PM-flux linkage estimate */
    double h;              /* Weight of the external position error signal */
    int caught;            /* Estimates initialized by a flying start */
} ObserverOutputs;

/* Flux observer in estimated rotor coordinates */
typedef struct {
    SynchronousMachinePars par;
    double k_theta;
    double k_o[2]; /* Observer gain k_o(w_m) = k_o[0] + k_o[1]*abs(w_m) */
    /* States */
    double theta_m;
    double complex psi_s;
    /* Other memory variables */
    double T_s_prev; /* Length of the previous sampling period */
} FluxObserver;

typedef struct {
    SpeedObserver speed_observer;
    FluxObserver flux_observer;
    /* Current at the start of the latest sampling period with an unknown voltage,
     * NAN if the voltage is known (see flux_observer_flying_start) */
    double complex i_s_ab;
} SpeedFluxObserver;

/* Flux and torque reference generator based on precomputed lookup tables, with the
 * current reference of current-vector control as a state, driven toward the flux
 * and torque references by the tracking law (alpha the bandwidth, zero if not used,
 * as in flux-vector and V/Hz control) */
typedef struct {
    SynchronousMachinePars par;
    double k_u;
    double k_mtpv;
    double alpha;
    double psi_s_min;
    double psi_s_max;
    LUT psi_s_mtpa;  /* MTPA flux magnitude vs. torque */
    LUT tau_M_cl;    /* Current-limit torque vs. flux magnitude */
    LUT tau_M_mtpv;  /* MTPV torque vs. flux magnitude */
    /* States: the current reference, and the flux and torque references tracked */
    double complex i_s_ref;
    double tau_M_ref;
    double psi_s_ref;
} ReferenceGenerator;

typedef struct {
    SynchronousMachinePars par;
    double alpha_psi;
    double alpha_tau;
    double alpha_i;
    /* Integral states */
    double x_psi;
    double x_tau;
    /* Workspace variables */
    double complex i_a;
    double complex u_ff;
} FluxTorqueController;

typedef struct {
    double T_s;
    int sensorless;
    ReferenceGenerator reference_gen;
    FluxTorqueController flux_torque_ctrl;
    SpeedFluxObserver observer;
} FluxVectorController;

/* Reference signals */
typedef struct {
    double T_s;
    double d_abc[3];
    double w_M;
    double tau_M;
    double psi_s;
    double complex i_s; /* Current reference of current-vector control */
    double complex u_s;
    double complex u_c_ab; /* Limited converter voltage from the PWM */
} References;

/* Measured signals */
typedef struct {
    double complex i_c_ab; /* Converter current in stationary coordinates */
    double u_dc;
    double theta_M; /* Mechanical rotor angle, used only in the sensored mode */
} Measurements;

/* Vector control system (VectorControlSystem). While the gates are blocked (the gate
 * enable is zero, see ControlSystem.set_enable), the control system is in standby:
 * the converter voltage is unknown, so the observer runs on its open-circuit model,
 * the controller computes its output for zero torque, and the speed controller is
 * not updated. A sensorless drive stays in standby with the zero vector also at the
 * first sampling instant after the release, and the observer catches the rotating
 * machine from the current response (flux_observer_flying_start). */
typedef struct {
    PWM pwm;
    double k_comp; /* Angle-advance factor of the voltage reference (k_comp) */
    FluxVectorController vector_ctrl;
    PIController speed_ctrl;
    /* Workspace variables passed from the output to the update function */
    int enabled; /* Gate enable */
    ObserverOutputs fbk;
    References ref;
    int release_step; /* First sampling instant after the release */
} VectorControlSystem;

/* Flux-vector controller configuration (FluxVectorControllerCfg). As in motulator,
 * the optional parameters default to None, represented here by NAN. Only the
 * offline reference generation (online_ref=False) and the default PM-flux
 * estimation gain (k_f=None) are supported. The observer gain k_o is given as
 * k_o(w_m) = k_o[0] + k_o[1]*abs(w_m). */
typedef struct {
    double i_s_max;
    double alpha_tau;
    double alpha_psi;
    double alpha_i;
    double alpha_o;
    double k_o[2];
    double psi_s_min;
    double psi_s_max;
    double k_u;
    double k_mtpv;
    double J;
    double T_s;
    int sensorless;
} FluxVectorControllerCfg;

static FluxVectorControllerCfg flux_vector_controller_cfg(double i_s_max);

static void vector_control_system_init(VectorControlSystem *self,
                                       SynchronousMachinePars par,
                                       const FluxVectorControllerCfg *cfg,
                                       PIController speed_ctrl);
/* Output of the control loop, with the gate enable at the sampling instant */
static void vector_control_system_compute_output(VectorControlSystem *self,
                                                 const Measurements *meas,
                                                 double w_M_ref, int enabled);
static void vector_control_system_update(VectorControlSystem *self);

/* Observer-based V/Hz controller configuration (ObserverBasedVHzControllerCfg). As in
 * FluxVectorControllerCfg, the optional parameters default to None, represented by
 * NAN, and the observer gain is k_o(w_m) = k_o[0] + k_o[1]*abs(w_m). */
typedef struct {
    double i_s_max;
    double alpha_psi;
    double alpha_tau;
    double alpha_f;
    double alpha_o;
    double k_o[2];
    double k_u;
    double k_mtpv;
    double psi_s_min;
    double psi_s_max;
    double T_s;
} SMVHzControllerCfg;

/* Observer-based V/Hz controller (ObserverBasedVHzController): the flux observer
 * without speed estimation (create_vhz_observer), whose rotor speed is the
 * rate-limited speed reference, and the flux and torque references from the
 * low-pass-filtered torque estimate */
typedef struct {
    double T_s;
    double alpha_f;
    ReferenceGenerator reference_gen;
    FluxTorqueController flux_torque_ctrl;
    FluxObserver observer;
    double tau_M_lpf; /* Low-pass-filtered torque estimate (state) */
} SMVHzController;

/* V/Hz control system (VHzControlSystem). While the gates are blocked, the converter
 * voltage is unknown: the observer runs on its open-circuit model, and the
 * controller computes its standby output. */
typedef struct {
    PWM pwm;
    double k_comp; /* Angle-advance factor of the voltage reference (k_comp) */
    SMVHzController vhz_ctrl;
    RateLimiter rate_limiter; /* Of the speed reference (mechanical rad/s^2) */
    /* Workspace variables passed from the output to the update function */
    ObserverOutputs fbk;
    References ref;
} SMVHzControlSystem;

static SMVHzControllerCfg sm_vhz_controller_cfg(double i_s_max);

static void sm_vhz_control_system_init(SMVHzControlSystem *self,
                                       SynchronousMachinePars par,
                                       const SMVHzControllerCfg *cfg, double slew_rate);
/* Output of the control loop, with the gate enable at the sampling instant */
static void sm_vhz_control_system_compute_output(SMVHzControlSystem *self,
                                                 const Measurements *meas,
                                                 double w_M_ref, int enabled);
static void sm_vhz_control_system_update(SMVHzControlSystem *self);

/* Signal vectors of the modular control system, in which each class is a block (see
 * motulator_export.plecs._sm): the measurements (meas), the feedback signals (fbk),
 * and the references of the flux-vector controller (ref) as arrays of doubles, a
 * complex field as its real and imaginary parts. The defines give the indices of
 * the fields. The fields u_dc and tau_L of ObserverOutputs, which the callers of
 * FluxObserver set in motulator (VectorControlSystem and SpeedFluxObserver), are not
 * included. */
#define MEAS_i_c_ab 0
#define MEAS_u_dc 2
#define MEAS_theta_M 3
#define MEAS_WIDTH 4

#define FBK_i_s 0
#define FBK_u_s 2
#define FBK_u_s_zoh 4
#define FBK_psi_s 6
#define FBK_e_o 8
#define FBK_eps 10
#define FBK_eps_f 11
#define FBK_psi_a 12
#define FBK_tau_M 14
#define FBK_w_c 15
#define FBK_w_m 16
#define FBK_w_M 17
#define FBK_theta_c 18
#define FBK_theta_m 19
#define FBK_psi_f 20
#define FBK_h 21
#define FBK_caught 22
#define FBK_WIDTH 23

#define REF_psi_s 0
#define REF_tau_M 1
#define REF_u_s 2
#define REF_WIDTH 4

static void measurements_pack(const Measurements *meas, double y[MEAS_WIDTH]);
static void measurements_unpack(const double u[MEAS_WIDTH], Measurements *meas);
static void observer_outputs_pack(const ObserverOutputs *fbk, double y[FBK_WIDTH]);
/* The fields u_dc and tau_L are zero */
static void observer_outputs_unpack(const double u[FBK_WIDTH], ObserverOutputs *fbk);
/* The fields psi_s, tau_M, and u_s, the others are zero */
static void references_unpack(const double u[REF_WIDTH], References *ref);

#endif
