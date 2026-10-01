/*
 * Minimal mock of the Simulink S-function API (simstruc.h) for testing the
 * S-functions with gcc, without MATLAB. Only the macros used by the S-functions of
 * the package are defined. The test API is in simulink.c, which the S-function
 * includes at its end (as with the real simulink.c), so it can call the static
 * callback functions.
 */

#ifndef MOCK_SIMSTRUC_H
#define MOCK_SIMSTRUC_H

#include <stddef.h>

typedef double real_T;
typedef int int_T;

typedef struct {
    size_t numel;
    const double *pr;
} mxArray;

#define MOCK_MAX_PARAMS 32
#define MOCK_MAX_PORTS 8
#define MOCK_MAX_WIDTH 16

typedef struct {
    int num_params; /* Expected by the S-function */
    int params_count; /* Given */
    const mxArray *params[MOCK_MAX_PARAMS];
    const char *error;
    int num_inputs;
    int num_outputs;
    int input_width[MOCK_MAX_PORTS];
    int output_width[MOCK_MAX_PORTS];
    int feedthrough[MOCK_MAX_PORTS];
    double inputs[MOCK_MAX_PORTS][MOCK_MAX_WIDTH];
    double outputs[MOCK_MAX_PORTS][MOCK_MAX_WIDTH];
    void *pwork[4];
    double sample_time;
    double offset_time;
} SimStruct;

#define mxGetNumberOfElements(p) ((p)->numel)
#define mxGetPr(p) ((double *)(p)->pr)
#define mxIsDouble(p) ((void)(p), 1)
#define mxIsComplex(p) ((void)(p), 0)

#define SS_PRM_NOT_TUNABLE 0
#define SS_OPTION_EXCEPTION_FREE_CODE 0
#define UNUSED_ARG(x) (void)(x)

#define ssSetNumSFcnParams(S, n) ((S)->num_params = (n))
#define ssGetNumSFcnParams(S) ((S)->num_params)
#define ssGetSFcnParamsCount(S) ((S)->params_count)
#define ssGetSFcnParam(S, k) ((S)->params[k])
#define ssSetSFcnParamTunable(S, k, v) ((void)(k), (void)(v))
#define ssSetErrorStatus(S, msg) ((S)->error = (msg))
#define ssGetErrorStatus(S) ((S)->error)
#define ssSetNumContStates(S, n) ((void)(n))
#define ssSetNumDiscStates(S, n) ((void)(n))
#define ssSetNumInputPorts(S, n) ((S)->num_inputs = (n), 1)
#define ssSetInputPortWidth(S, k, w) ((S)->input_width[k] = (w))
#define ssSetInputPortDirectFeedThrough(S, k, v) ((S)->feedthrough[k] = (v))
#define ssSetInputPortRequiredContiguous(S, k, v) ((void)(v))
#define ssSetNumOutputPorts(S, n) ((S)->num_outputs = (n), 1)
#define ssSetOutputPortWidth(S, k, w) ((S)->output_width[k] = (w))
#define ssSetNumSampleTimes(S, n) ((void)(n))
#define ssSetNumPWork(S, n) ((void)(n))
#define ssSetOptions(S, opts) ((void)(opts))
#define ssSetSampleTime(S, k, t) ((S)->sample_time = (t))
#define ssSetOffsetTime(S, k, t) ((S)->offset_time = (t))
#define ssGetPWork(S) ((S)->pwork)
#define ssGetInputPortRealSignal(S, k) ((const real_T *)(S)->inputs[k])
#define ssGetOutputPortRealSignal(S, k) ((S)->outputs[k])

#endif
