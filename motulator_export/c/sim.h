/*
 * Simulation of a continuous-time system model with a discrete-time control system,
 * ported from motulator.
 *
 * Python counterparts:
 *   motulator/common/model/_simulation.py  Simulation
 *   motulator/common/model/_base.py        Model.modulate, Delay
 *   motulator/common/model/_pwm.py         ZOH, CarrierComparison
 *   motulator/common/utils/_dead_time.py   averaged_gate_signals
 *   motulator/common/model/_converter.py   VoltageSourceConverter, DCBus
 *   motulator/common/model/_ac_port.py     ACPort, leg_voltages, diode_margin,
 *                                          update_diodes
 *   motulator_export/plecs/_common.py      StepSignal
 *   scipy/integrate/_ivp/rk.py             RK45 (used by solve_ivp in Simulation)
 *
 * A model (Model) consists of a voltage-source converter, its modulator (the
 * computational delay and the PWM model), optionally a DC-bus model feeding the
 * converter, and the continuous-time subsystems, whose state derivatives are given by
 * the function rhs. The control system
 * (ControlSystem) is run at its sampling instants. The switching state of the
 * converter is held constant between the sampling and switching instants, and the
 * states are integrated over these intervals using the Dormand-Prince method of
 * scipy (RK45). As in motulator, the states are complex, the real-valued states
 * having zero imaginary parts.
 *
 * The control system can block the gates of the converter (its gate enable), in
 * which case the diodes of the converter conduct according to the currents, and the
 * converter voltage is given by the AC port of its load. The transitions of the
 * diode states are located as the events of solve_ivp, at which the integration is
 * restarted, as in motulator.
 *
 * Supported: a single model with a single control system, and a constant DC-bus
 * voltage or a DC-bus model feeding only the converter, without an external DC
 * current, also fed from a diode bridge (a composite model of motulator, see
 * DCBus).
 */

#ifndef MOTULATOR_SIM_H
#define MOTULATOR_SIM_H

#include "common.h"

#define SIM_MAX_STATES 16
#define SIGNAL_MAX_STEPS 32
#define DELAY_MAX_LENGTH 8
/* Maximum number of switching intervals in a sampling period */
#define SIM_MAX_SWITCHING 32
/* Maximum number of consecutive transitions of the diode states at the same instant
 * (_MAX_STUCK of motulator) */
#define SIM_MAX_STUCK 10
/* Default maximum step (s) while the gates of the converter are blocked (_MAX_STEP of
 * motulator) */
#define SIM_MAX_STEP 1e-4

/* Status codes of the simulation */
#define SIM_OK 0
#define SIM_ERROR_STEP_SIZE 1  /* The solver step size became too small */
#define SIM_ERROR_NOT_FINITE 2 /* A state is not finite (e.g., NaN) */
#define SIM_ERROR_SAMPLING 3   /* The sampling period is not positive */
#define SIM_ERROR_MEMORY 4     /* The memory for the logs could not be allocated */
#define SIM_ERROR_SWITCHING 5  /* Too many switching intervals in a sampling period */
#define SIM_ERROR_SIGNAL 6     /* A signal is not finite (e.g., a failed function) */
#define SIM_ERROR_TRANSITIONS 7 /* Diode transitions stuck at the same instant */

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

/* AC port of the load of a converter (ACPort): the current derivative of the load at
 * the converter terminals is Y (u - e) in stationary coordinates, where u is the
 * converter voltage and Y acts on the real and imaginary parts */
typedef struct {
    double complex i; /* Current (A) */
    double complex e; /* Voltage (V) at which the current derivative is zero */
    double Y[2][2];   /* Inverse differential inductance matrix (1/H) */
} ACPort;

/* Three-phase voltage-source converter with a constant DC-bus voltage, the dead time
 * resolved by the current directions given by the signum function, a current within
 * the tolerance of the diode-state transitions having no direction (current_direction
 * of _ac_port.py). With the gates
 * blocked, the diode state of each leg is 1 if its lower diode conducts, -1 if its
 * upper diode conducts, and 0 if the leg floats (see leg_voltages). */
typedef struct {
    double u_dc;           /* DC-bus voltage (V) */
    double u_th;           /* Threshold voltage (V) of the power devices */
    double complex q_c_ab; /* Switching state */
    double complex s_c_ab; /* Current-direction vector at the latest switching */
    int enabled;           /* Gate enable (else the gates are blocked) */
    double diodes[3];      /* Diode states of the legs with blocked gates */
} VoltageSourceConverter;

/* Set the switching state from the gate and blanking signals and the converter
 * current i_c_ab */
static void converter_set_gate_signals(VoltageSourceConverter *self,
                                       const double q_abc[3], const double b_abc[3],
                                       double complex i_c_ab);
/* Converter voltage (V) with the DC-bus voltage u_dc */
static double complex converter_voltage(const VoltageSourceConverter *self,
                                        double u_dc);
/* Internal DC current (A) with the converter current i_c_ab
 * (VoltageSourceConverter.compute_internal_dc_current), rectified by the diodes with
 * the gates blocked */
static double converter_dc_current(const VoltageSourceConverter *self,
                                   double complex i_c_ab);
/* Leg voltages (V) with respect to the negative rail of a converter with blocked
 * gates, the rail voltage u_dc, and the diode states (leg_voltages). The voltage of a
 * floating leg keeps its current at zero, i.e., it drives the residual current to
 * zero. */
static void leg_voltages(const ACPort *port, double u_dc, const double diodes[3],
                         double u_abc[3]);
/* Margin of the diode states (diode_margin), zero at their transition */
static double diode_margin(const ACPort *port, double u_dc, const double diodes[3],
                           const double u_abc[3]);
/* Update the diode states at a transition, or latch them from the directions of the
 * currents if latch is nonzero (update_diodes with diodes None) */
static void update_diodes(const ACPort *port, double u_dc, double diodes[3], int latch);
/* Direction of the phase current i (current_direction of _ac_port.py with np.sign),
 * zero within the tolerance of the diode-state transitions */
static double current_direction(double i);

/* DC bus ------------------------------------------------------------------- */

#define DC_BUS_NONE 0         /* Constant DC-bus voltage of the converter */
#define DC_BUS_CAPACITOR 1    /* DCBus */
#define DC_BUS_DIODE_BRIDGE 2 /* DCBus fed from a DiodeBridge */

/* DC-bus model feeding the converter, without an external DC current, optionally fed
 * from a diode bridge (DiodeBridge), which is supplied through an L filter (LFilter)
 * from a balanced three-phase source (ThreePhaseSource) with a constant magnitude
 * and frequency. Its states (u_dc of DCBus, and of the diode bridge, i_c_ab of the L
 * filter and the angle theta_g of the source) come first in the states of the model,
 * as in motulator (in a composite model, with the model of the diode bridge first). */
typedef struct {
    int type;
    double C_dc;      /* DC-bus capacitance (F) */
    double u_th;      /* Threshold voltage (V) of the diodes of the diode bridge */
    double L_t, R_t;  /* Inductance (H) and resistance (Ohm) of the L filter */
    double e_g, w_g;  /* Magnitude (V) and angular frequency (rad/s) of the source */
    double diodes[3]; /* Diode states of the legs of the diode bridge */
} DCBus;

/* Number of states, and their names separated by spaces */
static int dc_bus_states(const DCBus *self);
static const char *dc_bus_state_names(const DCBus *self);
/* DC-bus voltage (V) from the states x of the model, or the constant DC-bus voltage
 * u_dc of the converter without a DC-bus model */
static double dc_bus_voltage(const DCBus *self, const double complex *x, double u_dc);
/* Voltage (V) of the diode bridge at the states x (DiodeBridge.set_outputs), and the
 * margin of its diode states (if margin is not NULL) */
static double complex bridge_voltage(const DCBus *self, const double complex *x,
                                     double *margin);
/* Update the diode states of the diode bridge at a transition, or latch them from the
 * directions of the currents if latch is nonzero (DiodeBridge.update_diodes) */
static void bridge_update_diodes(DCBus *self, const double complex *x, int latch);
/* State derivatives of the DC bus (DCBus.rhs), and of the diode bridge feeding it
 * (LFilter.rhs and ThreePhaseSource.rhs), the converter drawing the internal DC
 * current i_dc_int */
static void dc_bus_rhs(const DCBus *self, const double complex *x, double i_dc_int,
                       double complex *dx);
/* Keep the angle of the source of the diode bridge within [-pi, pi)
 * (ThreePhaseSource.set_states) */
static void dc_bus_set_states(const DCBus *self, double complex *x);

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
 * and set the functions. The states of the DC bus come first. */
struct Model {
    int n;              /* Number of states */
    const char *states; /* Names of the states, separated by spaces */
    VoltageSourceConverter converter;
    Modulator modulator;
    DCBus dc_bus;
    /* Set the number and the names of the states from the configuration of the
     * subsystems (e.g., after setting the DC-bus model) */
    void (*init_states)(Model *self);
    /* State derivatives, with the switching state of the converter held constant */
    void (*rhs)(const Model *self, double t, const double complex *x,
                double complex *dx);
    /* Converter current, determining the leg states during blanking */
    double complex (*converter_current)(const Model *self, const double complex *x);
    /* Set the states at the start of an integration interval (e.g., keep the angles
     * within [-pi, pi), as Subsystem.set_states) */
    void (*set_states)(const Model *self, double complex *x);
    /* AC port of the load of the converter at the time t and the states x
     * (compute_ac_port of the load) */
    void (*ac_port)(const Model *self, double t, const double complex *x,
                    ACPort *port);
};

/* DC-bus voltage (V) of the converter at the states x */
static double model_dc_voltage(const Model *self, const double complex *x);
/* Converter voltage (V) at the time t and the states x (VoltageSourceConverter
 * .set_outputs), given by the AC port of the load with the gates blocked, and the
 * margin of the diode states (if margin is not NULL; infinite with the gates
 * enabled) */
static double complex model_converter_voltage(const Model *self, double t,
                                              const double complex *x, double *margin);
/* Set the gate enable at the time t and the states x, latching the diode states when
 * the gates are blocked (VoltageSourceConverter.set_enable) */
static void model_set_enable(Model *self, double t, const double complex *x,
                             int enabled);
/* Names of the states of the DC bus followed by `names`, written into `buffer` of
 * the given size */
static void model_state_names(const Model *self, const char *names, char *buffer,
                              int size);

typedef struct ControlSystem ControlSystem;

/* Discrete-time control system. The concrete control systems embed this as their
 * first member and set the function run, which logs the signals of the step into
 * log: the controller time followed by the fields (see log_fields_width). */
struct ControlSystem {
    /* Gate enable at the controller time (ControlSystem.get_enable), which takes
     * effect at the sampling instant before the control step */
    int (*get_enable)(const ControlSystem *self);
    /* Control step at the sampling instant t, with the model states x and the gate
     * enable: gives the sampling period T_s and the duty ratios, and returns the
     * status */
    int (*run)(ControlSystem *self, const Model *mdl, double t,
               const double complex *x, int enabled, double *T_s, double d_abc[3]);
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
    /* Solution points: the time, the states, the switching state and the
     * current-direction vector of the converter, its gate enable, and its voltage with
     * the gates blocked (else NaN), followed by the voltage of the diode bridge
     * feeding the DC bus, complex values as their real and imaginary parts */
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
