/*
 * Observer-based V/Hz control of induction machine drives, ported from motulator.
 *
 * Python counterparts:
 *   motulator/drive/control/_im_flux_vector.py    FluxTorqueController,
 *                                                 ReferenceGenerator,
 *                                                 ObserverBasedVHzControllerCfg,
 *                                                 ObserverBasedVHzController
 *   motulator/drive/control/_im_observers.py      create_vhz_observer
 *   motulator/drive/control/_base.py              VHzControlSystem
 *   motulator/common/control/_controllers.py      RateLimiter
 *
 * The flux observer of im_current_vector.h is used without speed estimation, the
 * rotor speed being the rate-limited speed reference. Pure open-loop V/Hz control is
 * the special case L_M = inf (with R_s = R_R = L_sgm = 0 and zero gains), as in
 * motulator. The observer gain k_o is the default of motulator. The PWM uses the MME
 * overmodulation (pwm_mode of ObserverBasedVHzController). As in the other control
 * systems, the output function has no side effects (apart from workspace variables),
 * and all states are updated in the update function.
 */

#ifndef MOTULATOR_IM_FLUX_VECTOR_H
#define MOTULATOR_IM_FLUX_VECTOR_H

#include "im_current_vector.h"

/* Flux and torque controller (FluxTorqueController), without the integral action,
 * which observer-based V/Hz control does not use */
typedef struct {
    InductionMachineInvGammaPars par;
    double alpha_psi;
    double alpha_tau;
} IMFluxTorqueController;

static void im_flux_torque_ctrl_init(IMFluxTorqueController *self,
                                     InductionMachineInvGammaPars par,
                                     double alpha_psi, double alpha_tau);
/* Stator voltage reference from the flux and torque references */
static double complex im_flux_torque_ctrl_compute_output(
    const IMFluxTorqueController *self, double psi_s_ref, double tau_M_ref,
    const IMObserverOutputs *fbk);

/* Reference generator with field weakening and torque limits (ReferenceGenerator) */
typedef struct {
    InductionMachineInvGammaPars par;
    double psi_s_nom;
    double i_s_max;
    double tau_M_max;
    double k_u;
    double k_b;
} IMReferenceGenerator;

static void im_flux_reference_gen_init(IMReferenceGenerator *self,
                                       InductionMachineInvGammaPars par,
                                       double psi_s_nom, double i_s_max,
                                       double tau_M_max, double k_u, double k_b);
/* Stator flux reference and limited torque reference */
static void im_reference_gen_flux_and_torque(const IMReferenceGenerator *self,
                                             double tau_M_ref, double w_s,
                                             double psi_R, double u_dc,
                                             double *psi_s_ref, double *tau_M_ref_lim);

/* Observer-based V/Hz controller configuration (ObserverBasedVHzControllerCfg). The
 * observer gain k_o is the default of motulator. */
typedef struct {
    double psi_s_nom;
    double i_s_max;
    double alpha_psi;
    double alpha_tau;
    double alpha_f;
    double k_u;
    double k_b;
    double T_s;
} IMVHzControllerCfg;

/* Observer-based V/Hz controller (ObserverBasedVHzController) */
typedef struct {
    double T_s;
    double alpha_f;
    double h; /* Weight of the external speed error signal: 1 in open-loop V/Hz */
    IMReferenceGenerator reference_gen;
    IMFluxTorqueController flux_torque_ctrl;
    IMFluxObserver observer;
    double tau_M_lpf; /* Low-pass-filtered torque estimate (state) */
} IMVHzController;

/* Reference signals */
typedef struct {
    double T_s;
    double d_abc[3];
    double w_M;   /* Rate-limited speed reference */
    double tau_M; /* Limited torque reference */
    double psi_s; /* Stator flux reference */
    double complex u_s;
    double complex u_c_ab; /* Limited converter voltage from the PWM */
} IMVHzReferences;

/* Measured signals */
typedef struct {
    double complex i_c_ab; /* Converter current in stationary coordinates */
    double u_dc;
} IMVHzMeasurements;

/* V/Hz control system (VHzControlSystem) */
typedef struct {
    PWM pwm;
    IMVHzController vhz_ctrl;
    RateLimiter rate_limiter; /* Of the speed reference (mechanical rad/s^2) */
    /* Workspace variables passed from the output to the update function */
    IMObserverOutputs fbk;
    IMVHzReferences ref;
} IMVHzControlSystem;

static IMVHzControllerCfg im_vhz_controller_cfg(double psi_s_nom, double i_s_max);

static void im_vhz_control_system_init(IMVHzControlSystem *self,
                                       InductionMachineInvGammaPars par,
                                       const IMVHzControllerCfg *cfg, double slew_rate);
static void im_vhz_control_system_compute_output(IMVHzControlSystem *self,
                                                 const IMVHzMeasurements *meas,
                                                 double w_M_ref);
static void im_vhz_control_system_update(IMVHzControlSystem *self);
/* Change the stator and rotor resistances of the machine model while the control
 * system runs, as changing par.R_s and par.R_R of the controller in motulator */
static void im_vhz_control_system_set_resistances(IMVHzControlSystem *self, double R_s,
                                                  double R_R);

/* Signal vectors of the control system (see sm_flux_vector.h): the measurements
 * (VHZ_MEAS), and the references (VHZ_REF) psi_s, tau_M, and u_s. The feedback
 * signals are those of current-vector control (FBK of im_current_vector.h). */
#define VHZ_MEAS_i_c_ab 0
#define VHZ_MEAS_u_dc 2
#define VHZ_MEAS_WIDTH 3

#define VHZ_REF_psi_s 0
#define VHZ_REF_tau_M 1
#define VHZ_REF_u_s 2
#define VHZ_REF_WIDTH 4

static void im_vhz_measurements_pack(const IMVHzMeasurements *meas,
                                     double y[VHZ_MEAS_WIDTH]);
static void im_vhz_measurements_unpack(const double u[VHZ_MEAS_WIDTH],
                                       IMVHzMeasurements *meas);
/* The fields psi_s, tau_M, and u_s, the others are zero */
static void im_vhz_references_unpack(const double u[VHZ_REF_WIDTH],
                                     IMVHzReferences *ref);

#endif
