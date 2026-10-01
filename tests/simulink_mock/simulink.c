/*
 * Test API of the mock Simulink engine (see simstruc.h): the S-function callbacks
 * in the order Simulink calls them, with the inputs and outputs as flat arrays.
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
        mock_params[k].numel = (size_t)numel[k];
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
    mdlUpdate(&mock, 0);
}

void sfun_terminate(void)
{
    mdlTerminate(&mock);
}
