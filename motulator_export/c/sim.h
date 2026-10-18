/*
 * Simulation of a continuous-time system model with a discrete-time control system,
 * ported from motulator.
 *
 * Python counterparts:
 *   motulator/common/model/_simulation.py  Simulation
 *   motulator/common/model/_base.py        Model.modulate, Delay
 *   motulator/common/model/_pwm.py         ZOH, CarrierComparison
 *   motulator/common/utils/_dead_time.py   averaged_gate_signals
 *   motulator/common/model/_converter.py   VoltageSourceConverter
 *   motulator_export/plecs/_common.py      StepSignal
 *   scipy/integrate/_ivp/rk.py             RK45 (used by solve_ivp in Simulation)
 *
 * A model (Model) consists of a voltage-source converter, its modulator (the
 * computational delay and the PWM model), and the continuous-time subsystems, whose
 * state derivatives are given by the function rhs. The control system
 * (ControlSystem) is run at its sampling instants. The switching state of the
 * converter is held constant between the sampling and switching instants, and the
 * states are integrated over these intervals using the Dormand-Prince method of
 * scipy (RK45). As in motulator, the states are complex, the real-valued states
 * having zero imaginary parts.
 *
 * Supported: a single model with a single control system, the gates enabled (no
 * gate blocking), and a constant DC-bus voltage.
 */

#ifndef MOTULATOR_SIM_H
#define MOTULATOR_SIM_H

#include "common.h"

#define SIM_MAX_STATES 16
#define SIGNAL_MAX_STEPS 32
#define DELAY_MAX_LENGTH 8
/* Maximum number of switching intervals in a sampling period */
#define SIM_MAX_SWITCHING 32

/* Status codes of the simulation */
#define SIM_OK 0
#define SIM_ERROR_STEP_SIZE 1  /* The solver step size became too small */
#define SIM_ERROR_NOT_FINITE 2 /* A state is not finite (e.g., NaN) */
#define SIM_ERROR_SAMPLING 3   /* The sampling period is not positive */
#define SIM_ERROR_MEMORY 4     /* The memory for the logs could not be allocated */
#define SIM_ERROR_SWITCHING 5  /* Too many switching intervals in a sampling period */
#define SIM_ERROR_SIGNAL 6     /* A signal is not finite (e.g., a failed function) */

/* Signals ------------------------------------------------------------------ */

/* Function of time, e.g., a Python callable via ctypes */
typedef double (*SignalFunction)(double t);

/* Signal of time: steps from the value `before` to the values after[k] at t > time[k]
 * (StepSignal; a constant if there are no steps), or the function fcn if not NULL */
typedef struct {
    double before;
    int n;
    double time[SIGNAL_MAX_STEPS];
    double after[SIGNAL_MAX_STEPS];
    SignalFunction fcn;
} Signal;

static void signal_constant(Signal *self, double value);
static double signal_value(const Signal *self, double t);

/* Solver ------------------------------------------------------------------- */

/* State derivatives dx at the time t and the states x */
typedef void (*OdeFunction)(void *data, double t, const double complex *x,
                            double complex *dx);

typedef struct {
    double max_step;
    double rtol;
    double atol;
} SolverCfg;

/* Explicit Runge-Kutta method of order 5(4) with the step size control of scipy
 * (RK45), for complex states */
typedef struct {
    OdeFunction fun;
    void *data;
    int n;
    SolverCfg cfg;
    double t;
    double t_old;
    double t_bound;
    double h_abs;
    double complex x[SIM_MAX_STATES];
    double complex x_old[SIM_MAX_STATES];
    double complex f[SIM_MAX_STATES];    /* Derivative at (t, x) */
    double complex K[7][SIM_MAX_STATES]; /* Stages of the latest step */
} RK45;

#define RK45_RUNNING 0
#define RK45_FINISHED 1
#define RK45_FAILED -1

/* Initialize the solver at (t0, x0) for integration until t_bound, trying first_step
 * as the first step size */
static void rk45_init(RK45 *self, OdeFunction fun, void *data, int n, double t0,
                      const double complex *x0, double t_bound, double first_step,
                      const SolverCfg *cfg);
/* Take a step, returning the status */
static int rk45_step(RK45 *self);
/* Dense output (the interpolant of the latest step) at the time t */
static void rk45_dense_output(const RK45 *self, double t, double complex *x);

/* Modulator ---------------------------------------------------------------- */

/* Switching intervals of a sampling period: durations, gate signals of the upper
 * switches, and blanking signals */
typedef struct {
    int n;
    double t_step[SIM_MAX_SWITCHING];
    double q_abc[SIM_MAX_SWITCHING][3];
    double b_abc[SIM_MAX_SWITCHING][3];
} SwitchingSequence;

/* Computational delay (Delay) and the PWM model: zero-order hold (ZOH) or carrier
 * comparison (CarrierComparison) */
typedef struct {
    int carrier;       /* Carrier comparison (else ZOH) */
    double N;          /* Counter quantization levels of the carrier comparison */
    double t_d;        /* Dead time (s) */
    int single_update; /* Duty ratios updated once per carrier period (else twice) */
    /* Delay in half carrier periods, as a ring buffer of the duty ratios */
    int delay;
    int head;
    double buffer[DELAY_MAX_LENGTH][3];
    /* States of the carrier comparison */
    int rising_edge;
    int has_command; /* The commanded leg states are known (after the first call) */
    double command[3];
    double remaining_dead_time[3];
} Modulator;

/* Modulator with the given PWM model and delay, with the buffer of the delay and the
 * states of the carrier comparison at their initial values */
static void modulator_init(Modulator *self, int carrier, double N, double t_d,
                           int delay, int single_update);
/* Delay the duty ratios and convert them to the switching intervals of the sampling
 * period T_s (Model.modulate). Returns SIM_ERROR_SWITCHING if there are too many
 * intervals. */
static int modulator_modulate(Modulator *self, double T_s, const double d_abc[3],
                              SwitchingSequence *seq);

/* Converter ---------------------------------------------------------------- */

/* Three-phase voltage-source converter with a constant DC-bus voltage, the dead time
 * resolved by the current directions given by the signum function */
typedef struct {
    double u_dc;           /* DC-bus voltage (V) */
    double u_th;           /* Threshold voltage (V) of the power devices */
    double complex q_c_ab; /* Switching state */
    double complex s_c_ab; /* Current-direction vector at the latest switching */
} VoltageSourceConverter;

/* Set the switching state from the gate and blanking signals and the converter
 * current i_c_ab */
static void converter_set_gate_signals(VoltageSourceConverter *self,
                                       const double q_abc[3], const double b_abc[3],
                                       double complex i_c_ab);
/* Converter voltage (V) */
static double complex converter_voltage(const VoltageSourceConverter *self);

/* Logs --------------------------------------------------------------------- */

/* Rows of doubles, growing as needed */
typedef struct {
    int width; /* Doubles per row */
    int rows;
    int capacity;
    double *data;
} Log;

static void log_init(Log *self, int width);
/* Append a row, returning a pointer to it (NULL if the memory ran out) */
static double *log_append(Log *self);
static void log_free(Log *self);

/* Model and control system ------------------------------------------------- */

typedef struct Model Model;

/* Continuous-time system model. The concrete models embed this as their first member
 * and set the functions. */
struct Model {
    int n;              /* Number of states */
    const char *states; /* Names of the states, separated by spaces */
    VoltageSourceConverter converter;
    Modulator modulator;
    /* State derivatives, with the switching state of the converter held constant */
    void (*rhs)(const Model *self, double t, const double complex *x,
                double complex *dx);
    /* Converter current, determining the leg states during blanking */
    double complex (*converter_current)(const Model *self, const double complex *x);
    /* Set the states at the start of an integration interval (e.g., keep the angles
     * within [-pi, pi), as Subsystem.set_states) */
    void (*set_states)(const Model *self, double complex *x);
};

typedef struct ControlSystem ControlSystem;

/* Discrete-time control system. The concrete control systems embed this as their
 * first member and set the function run, which logs the signals of the step into
 * log: the controller time followed by the fields (see log_fields_width). */
struct ControlSystem {
    /* Control step at a sampling instant, with the model states x: gives the
     * sampling period T_s and the duty ratios, and returns the status */
    int (*run)(ControlSystem *self, const Model *mdl, const double complex *x,
               double *T_s, double d_abc[3]);
    const char *fields; /* Logged fields */
    Log log;
};

/* Simulation --------------------------------------------------------------- */

typedef struct {
    Model *mdl;
    ControlSystem *ctrl;
    SolverCfg cfg;
    double t0;                        /* Model time */
    double complex x[SIM_MAX_STATES]; /* States at t0 */
    double t_sample;                  /* Next sampling instant */
    /* Pending switching instants with their gate and blanking signals */
    int n_events;
    int i_event;
    double t_event[SIM_MAX_SWITCHING];
    double q_abc[SIM_MAX_SWITCHING][3];
    double b_abc[SIM_MAX_SWITCHING][3];
    /* Solution points: the time, the states, and the switching state and the
     * current-direction vector of the converter, complex values as their real and
     * imaginary parts */
    Log log;
} Simulation;

/* Simulation of the model mdl (with its states at the time t0) and the control
 * system ctrl, whose first sampling instant is t_start */
static void simulation_init(Simulation *self, Model *mdl, ControlSystem *ctrl,
                            const SolverCfg *cfg, double t0, const double complex *x0,
                            double t_start);
/* Run until the first sampling instant after t_stop (Simulation.simulate). A
 * subsequent call continues. The solver returns N_eval evenly spaced points for each
 * integration interval, or its steps if N_eval is zero. Returns the status. */
static int simulation_run(Simulation *self, double t_stop, int N_eval);
static void simulation_free(Simulation *self);

#endif
