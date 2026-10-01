/*
 * Test API of the mock Simulink engine (see simstruc.h): the S-function callbacks
 * in the order Simulink calls them, with the inputs and outputs as flat arrays.
 * The parameters are column vectors. The callbacks that the S-function does not
 * define are not called.
 */

static SimStruct mock;
static mxArray mock_params[MOCK_MAX_PARAMS];

/* Initialize the block with the parameters (concatenated values and the numbers
 * of their elements), returning the error message or NULL */
const char *sfun_start(int num_params, const int *numel, const double *values)
{
    SimStruct empty = {0};
    mock = empty;
    mock.params_count = num_params;
    for (int k = 0; k < num_params; k++) {
        mock_params[k].m = (size_t)numel[k];
        mock_params[k].n = 1;
        mock_params[k].pr = values;
        values += numel[k];
        mock.params[k] = &mock_params[k];
    }
    mdlInitializeSizes(&mock);
    if (mock.error != NULL) {
        return mock.error;
    }
    if (mock.num_params != num_params) {
        return "Wrong number of parameters.";
    }
    mdlInitializeSampleTimes(&mock);
    mdlStart(&mock);
    return mock.error;
}

double sfun_sample_time(void)
{
    return mock.sample_time;
}

/* One sampling period: the outputs from the inputs, then the state update */
void sfun_step(const double *u, double *y)
{
    for (int k = 0; k < mock.num_inputs; k++) {
        for (int j = 0; j < mock.input_width[k]; j++) {
            mock.inputs[k][j] = *u++;
        }
    }
    mdlOutputs(&mock, 0);
    for (int k = 0; k < mock.num_outputs; k++) {
        for (int j = 0; j < mock.output_width[k]; j++) {
            *y++ = mock.outputs[k][j];
        }
    }
#ifdef MDL_UPDATE
    mdlUpdate(&mock, 0);
#endif
}

#ifdef MDL_DERIVATIVES
/* Continuous states: get (x != NULL) or set them, and evaluate the derivatives
 * after mdlOutputs (call sfun_step first with the inputs) */
void sfun_states(double *x, const double *x_set)
{
    for (int k = 0; k < mock.num_cont_states; k++) {
        if (x_set != NULL) {
            mock.x[k] = x_set[k];
        }
        if (x != NULL) {
            x[k] = mock.x[k];
        }
    }
}

void sfun_derivatives(double *dx)
{
    mdlDerivatives(&mock);
    for (int k = 0; k < mock.num_cont_states; k++) {
        dx[k] = mock.dx[k];
    }
}
#endif

void sfun_terminate(void)
{
    mdlTerminate(&mock);
}
