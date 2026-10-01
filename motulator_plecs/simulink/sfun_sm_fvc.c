/*
 * Flux-vector control of a synchronous machine drive as a Level-2 C MEX S-function
 * (VectorControlSystem with FluxVectorController and SpeedController of motulator).
 *
 * The control algorithms are the C port in ../c, the same code as in the C-Script
 * blocks of the PLECS models. The parameters are in the order of PARAM_NAMES below
 * and correspond to the motulator API (SynchronousMachinePars,
 * FluxVectorControllerCfg, and SpeedController). An empty parameter ([])
 * corresponds to None in motulator, i.e., the default resolved in the C code.
 *
 * Inputs: w_M_ref, i_s_abc (3), u_dc, theta_M (used only in the sensored mode)
 * Outputs: d_abc (3), monitored signals (9): w_M_ref, w_M, tau_M_ref, tau_M,
 *          psi_s_ref, psi_s, theta_m, i_d, i_q
 *
 * The block runs at the sampling period T_s. The outputs are computed from the
 * measurements of the same sampling instant, and the states are updated in
 * mdlUpdate, as in motulator. The computational delay is outside the block.
 *
 * Build in MATLAB (requires a C99 compiler with complex.h, e.g., gcc or MinGW-w64,
 * not MSVC):
 *
 *     mex -I<path to motulator_plecs/c> sfun_sm_fvc.c
 */

#define S_FUNCTION_NAME sfun_sm_fvc
#define S_FUNCTION_LEVEL 2

#include <stdlib.h>

#include "simstruc.h"

/* The C port (after simstruc.h, since complex.h defines the macro I) */
#include "common.c"
#include "gradnet.c"
#include "sm_parameters.c"
#include "sm_control_loci.c"
#include "sm_flux_vector.c"

/* Parameters */
enum {
    PRM_N_P,
    PRM_R_S,
    PRM_L_D,
    PRM_L_Q,
    PRM_PSI_F,
    PRM_I_S_MAX,
    PRM_ALPHA_TAU,
    PRM_ALPHA_PSI,
    PRM_ALPHA_I,
    PRM_ALPHA_O,
    PRM_K_O,
    PRM_PSI_S_MIN,
    PRM_PSI_S_MAX,
    PRM_K_U,
    PRM_K_MTPV,
    PRM_J,
    PRM_SENSORLESS,
    PRM_T_S,
    PRM_SPEED_J,
    PRM_SPEED_ALPHA_S,
    PRM_SPEED_ALPHA_I,
    PRM_SPEED_TAU_M_MAX,
    NUM_PARAMS
};

/* Number of elements of a parameter */
#define PDIM(S, k) ((int)mxGetNumberOfElements(ssGetSFcnParam(S, k)))

/* Element j of a parameter */
#define P(S, k, j) (mxGetPr(ssGetSFcnParam(S, k))[j])

/* Parameter value, or NAN for an empty parameter (None in motulator) */
#define PARAM(S, k) (PDIM(S, k) > 0 ? P(S, k, 0) : NAN)

/* Inputs and outputs */
static const int INPUT_WIDTHS[] = {1, 3, 1, 1};
static const int OUTPUT_WIDTHS[] = {3, 9};
#define NUM_INPUTS (int)(sizeof(INPUT_WIDTHS) / sizeof(INPUT_WIDTHS[0]))
#define NUM_OUTPUTS (int)(sizeof(OUTPUT_WIDTHS) / sizeof(OUTPUT_WIDTHS[0]))

/* -------------------------------------------------------------------------- */

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    /* Required scalars (constant inductances, the GradNet flux map is not
     * supported yet) */
    static const int required[] = {
        PRM_N_P,     PRM_R_S, PRM_L_D,     PRM_L_Q,          PRM_PSI_F,
        PRM_I_S_MAX, PRM_T_S, PRM_SPEED_J, PRM_SPEED_ALPHA_S};
    for (int k = 0; k < NUM_PARAMS; k++) {
        const mxArray *p = ssGetSFcnParam(S, k);
        if (!mxIsDouble(p) || mxIsComplex(p)) {
            ssSetErrorStatus(S, "The parameters must be real double arrays.");
            return;
        }
        if (k != PRM_K_O && PDIM(S, k) > 1) {
            ssSetErrorStatus(S, "The parameters must be scalars or empty ([]).");
            return;
        }
    }
    for (size_t k = 0; k < sizeof(required) / sizeof(required[0]); k++) {
        if (PDIM(S, required[k]) != 1) {
            ssSetErrorStatus(S, "n_p, R_s, L_d, L_q, psi_f, i_s_max, T_s, speed_J, "
                                "and speed_alpha_s must be scalars.");
            return;
        }
    }
    if (PDIM(S, PRM_K_O) != 0 && PDIM(S, PRM_K_O) != 2) {
        ssSetErrorStatus(S, "k_o must be [] or [k0 k1].");
        return;
    }
    if (!(P(S, PRM_T_S, 0) > 0.0)) {
        ssSetErrorStatus(S, "T_s must be positive.");
        return;
    }
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    /* The ports do not depend on the parameters, so they are set first: the block
     * then has its ports even before its parameters are given */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) {
        return;
    }
    for (int k = 0; k < NUM_INPUTS; k++) {
        ssSetInputPortWidth(S, k, INPUT_WIDTHS[k]);
        ssSetInputPortDirectFeedThrough(S, k, 1);
        ssSetInputPortRequiredContiguous(S, k, 1);
    }
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) {
        return;
    }
    for (int k = 0; k < NUM_OUTPUTS; k++) {
        ssSetOutputPortWidth(S, k, OUTPUT_WIDTHS[k]);
    }
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1); /* VectorControlSystem */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Reported by Simulink */
    }
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) {
        return;
    }
#endif
    /* The parameters define the sample time and the lookup tables */
    for (int k = 0; k < NUM_PARAMS; k++) {
        ssSetSFcnParamTunable(S, k, SS_PRM_NOT_TUNABLE);
    }
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, P(S, PRM_T_S, 0));
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    VectorControlSystem *ctrl = calloc(1, sizeof(VectorControlSystem));
    if (ctrl == NULL) {
        ssSetErrorStatus(S, "Out of memory.");
        return;
    }
    ssGetPWork(S)[0] = ctrl;

    /* Machine model parameters (SynchronousMachinePars) */
    SynchronousMachinePars par = synchronous_machine_pars(
        P(S, PRM_N_P, 0), P(S, PRM_R_S, 0), P(S, PRM_L_D, 0), P(S, PRM_L_Q, 0),
        P(S, PRM_PSI_F, 0));

    /* Flux-vector controller configuration (FluxVectorControllerCfg), the defaults
     * are used for empty parameters */
    FluxVectorControllerCfg cfg = flux_vector_controller_cfg(P(S, PRM_I_S_MAX, 0));
#define OPTIONAL(field, k)                                                       \
    if (PDIM(S, k) > 0) {                                                        \
        cfg.field = P(S, k, 0);                                                  \
    }
    OPTIONAL(alpha_tau, PRM_ALPHA_TAU);
    OPTIONAL(alpha_psi, PRM_ALPHA_PSI);
    OPTIONAL(alpha_i, PRM_ALPHA_I);
    OPTIONAL(alpha_o, PRM_ALPHA_O);
    OPTIONAL(psi_s_min, PRM_PSI_S_MIN);
    OPTIONAL(psi_s_max, PRM_PSI_S_MAX);
    OPTIONAL(k_u, PRM_K_U);
    OPTIONAL(k_mtpv, PRM_K_MTPV);
    OPTIONAL(J, PRM_J);
    OPTIONAL(T_s, PRM_T_S);
#undef OPTIONAL
    if (PDIM(S, PRM_K_O) == 2) {
        cfg.k_o[0] = P(S, PRM_K_O, 0);
        cfg.k_o[1] = P(S, PRM_K_O, 1);
    }
    if (PDIM(S, PRM_SENSORLESS) > 0) {
        cfg.sensorless = P(S, PRM_SENSORLESS, 0) != 0.0;
    }

    /* Speed controller (SpeedController) */
    PIController speed_ctrl = speed_controller(
        P(S, PRM_SPEED_J, 0), P(S, PRM_SPEED_ALPHA_S, 0),
        PARAM(S, PRM_SPEED_ALPHA_I),
        PDIM(S, PRM_SPEED_TAU_M_MAX) > 0 ? P(S, PRM_SPEED_TAU_M_MAX, 0) : INFINITY);

    vector_control_system_init(ctrl, par, &cfg, speed_ctrl);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    VectorControlSystem *ctrl = (VectorControlSystem *)ssGetPWork(S)[0];

    /* Measurements */
    double w_M_ref = ssGetInputPortRealSignal(S, 0)[0];
    Measurements meas = {abc2complex(ssGetInputPortRealSignal(S, 1)),
                         ssGetInputPortRealSignal(S, 2)[0],
                         ssGetInputPortRealSignal(S, 3)[0]};

    vector_control_system_compute_output(ctrl, &meas, w_M_ref);

    /* Duty ratios, delayed by a Unit Delay block outside the S-function */
    real_T *d_abc = ssGetOutputPortRealSignal(S, 0);
    for (int k = 0; k < 3; k++) {
        d_abc[k] = ctrl->ref.d_abc[k];
    }

    /* Monitored signals */
    real_T *y = ssGetOutputPortRealSignal(S, 1);
    y[0] = ctrl->ref.w_M;
    y[1] = ctrl->fbk.w_M;
    y[2] = ctrl->ref.tau_M;
    y[3] = ctrl->fbk.tau_M;
    y[4] = ctrl->ref.psi_s;
    y[5] = cabs(ctrl->fbk.psi_s);
    y[6] = ctrl->fbk.theta_m;
    y[7] = creal(ctrl->fbk.i_s);
    y[8] = cimag(ctrl->fbk.i_s);
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    vector_control_system_update((VectorControlSystem *)ssGetPWork(S)[0]);
}

static void mdlTerminate(SimStruct *S)
{
    free(ssGetPWork(S)[0]);
    ssGetPWork(S)[0] = NULL;
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
