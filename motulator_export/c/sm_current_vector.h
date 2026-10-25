/*
 * Current-vector control of synchronous machine drives, ported from motulator.
 *
 * Python counterparts:
 *   motulator/drive/control/_sm_current_vector.py  CurrentController,
 *                                                  DiscreteCurrentController,
 *                                                  CurrentVectorControllerCfg,
 *                                                  CurrentVectorController
 *   motulator/drive/control/_base.py               VectorControlSystem
 *   motulator/common/control/_controllers.py       ComplexPIController,
 *                                                  DiscreteComplexController
 *
 * The observer (SpeedFluxObserver) and the reference generator (ReferenceGenerator,
 * with the current reference) are those of flux-vector control (sm_flux_vector.h),
 * and so are the feedback signals, the references, and the measurements. Only the
 * offline reference generation is supported.
 */

#ifndef MOTULATOR_SM_CURRENT_VECTOR_H
#define MOTULATOR_SM_CURRENT_VECTOR_H

#include "sm_flux_vector.h"

/* Current controller: a complex-vector PI controller of the flux linkages
 * (CurrentController) or, if discrete is nonzero, the direct discrete-time controller
 * (DiscreteCurrentController), to which the currents are mapped by the machine
 * model */
typedef struct {
    SynchronousMachinePars par;
    int discrete;
    ComplexPIController pi;
    DiscreteComplexController dc;
} SMCurrentController;

typedef struct {
    double T_s;
    int sensorless;
    ReferenceGenerator reference_gen;
    SMCurrentController current_ctrl;
    SpeedFluxObserver observer;
} SMCurrentVectorController;

/* Current-vector controller configuration (CurrentVectorControllerCfg). As in
 * FluxVectorControllerCfg, the optional parameters default to None, represented by
 * NAN, k_f is zero, and the observer gain is k_o(w_m) = k_o[0] + k_o[1]*abs(w_m). */
typedef struct {
    double i_s_max;
    double alpha_c;
    double alpha_i;
    double alpha_o;
    double alpha_ref;
    double k_o[2];
    double psi_s_min;
    double psi_s_max;
    double k_u;
    double k_mtpv;
    double J;
    double T_s;
    int sensorless;
    int discrete;
} SMCurrentVectorControllerCfg;

/* Vector control system (VectorControlSystem) with current-vector control, in
 * standby as with flux-vector control (see VectorControlSystem). When the observer
 * catches the machine, the current reference and the current controller restart
 * from the zero-torque operating point of the caught speed. */
typedef struct {
    PWM pwm;
    double k_comp; /* Angle-advance factor of the voltage reference (k_comp) */
    SMCurrentVectorController vector_ctrl;
    PIController speed_ctrl;
    int torque_mode; /* Torque-control mode (speed_ctrl = None): the external
                      * reference is the torque reference */
    /* Workspace variables passed from the output to the update function */
    int enabled; /* Gate enable */
    ObserverOutputs fbk;
    References ref;
    int release_step; /* First sampling instant after the release */
} SMCVCControlSystem;

static SMCurrentVectorControllerCfg sm_current_vector_controller_cfg(double i_s_max);

static void sm_cvc_control_system_init(SMCVCControlSystem *self,
                                       SynchronousMachinePars par,
                                       const SMCurrentVectorControllerCfg *cfg,
                                       PIController speed_ctrl);
/* Output of the control loop with the external reference (the speed reference,
 * or the torque reference in torque-control mode) and the gate enable at the
 * sampling instant */
static void sm_cvc_control_system_compute_output(SMCVCControlSystem *self,
                                                 const Measurements *meas,
                                                 double ext_ref, int enabled);
static void sm_cvc_control_system_update(SMCVCControlSystem *self);

/* Signal vector of the references of current-vector control (see the signal vectors
 * of sm_flux_vector.h, whose measurements and feedback signals are used): the flux,
 * torque, current, and voltage references */
#define CVC_REF_psi_s 0
#define CVC_REF_tau_M 1
#define CVC_REF_i_s 2
#define CVC_REF_u_s 4
#define CVC_REF_WIDTH 6

/* The fields psi_s, tau_M, i_s, and u_s, the others are zero */
static void cvc_references_unpack(const double u[CVC_REF_WIDTH], References *ref);

#endif
