/*
 * Simulation of a continuous-time system model with a discrete-time control system,
 * ported from motulator. See sim.h.
 */

#include <stdio.h>
#include <stdlib.h>

#include "sim.h"

/* Tolerance (s) for treating the sampling and switching instants as simultaneous */
#define SIM_T_TOL 1e-12

/* Signals ------------------------------------------------------------------ */

static void signal_constant(Signal *self, double value)
{
    self->before = value;
    self->n = 0;
    self->fcn = NULL;
}

static double signal_value(const Signal *self, double t)
{
    if (self->fcn != NULL) {
        return self->fcn(t);
    }
    /* Sum of the steps taken, as StepSignal.__call__ */
    double sum = 0.0, level = self->before;
    for (int k = 0; k < self->n; k++) {
        if (t > self->time[k]) {
            sum += self->after[k] - level;
        }
        level = self->after[k];
    }
    return self->before + sum;
}

/* Solver ------------------------------------------------------------------- */

/* Dormand-Prince coefficients of scipy (RK45) */
static const double RK45_C[6] = {0.0, 1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0, 8.0 / 9.0, 1.0};
static const double RK45_A[6][5] = {
    {0.0, 0.0, 0.0, 0.0, 0.0},
    {1.0 / 5.0, 0.0, 0.0, 0.0, 0.0},
    {3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0},
    {44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0},
    {19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0, 0.0},
    {9017.0 / 3168.0, -355.0 / 33.0, 46732.0 / 5247.0, 49.0 / 176.0,
     -5103.0 / 18656.0}};
static const double RK45_B[6] = {
    35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0};
static const double RK45_E[7] = {
    -71.0 / 57600.0,    0.0,           71.0 / 16695.0, -71.0 / 1920.0,
    17253.0 / 339200.0, -22.0 / 525.0, 1.0 / 40.0};
/* Coefficients of the dense output */
static const double RK45_P[7][4] = {
    {1.0, -8048581381.0 / 2820520608.0, 8663915743.0 / 2820520608.0,
     -12715105075.0 / 11282082432.0},
    {0.0, 0.0, 0.0, 0.0},
    {0.0, 131558114200.0 / 32700410799.0, -68118460800.0 / 10900136933.0,
     87487479700.0 / 32700410799.0},
    {0.0, -1754552775.0 / 470086768.0, 14199869525.0 / 1410260304.0,
     -10690763975.0 / 1880347072.0},
    {0.0, 127303824393.0 / 49829197408.0, -318862633887.0 / 49829197408.0,
     701980252875.0 / 199316789632.0},
    {0.0, -282668133.0 / 205662961.0, 2019193451.0 / 616988883.0,
     -1453857185.0 / 822651844.0},
    {0.0, 40617522.0 / 29380423.0, -110615467.0 / 29380423.0, 69997945.0 / 29380423.0}};

#define RK45_SAFETY 0.9
#define RK45_MIN_FACTOR 0.2
#define RK45_MAX_FACTOR 10.0
#define RK45_ERROR_EXPONENT (-1.0 / 5.0)

static void rk45_init(RK45 *self, OdeFunction fun, void *data, int n, double t0,
                      const double complex *x0, double t_bound, double first_step,
                      const SolverCfg *cfg)
{
    self->fun = fun;
    self->data = data;
    self->n = n;
    self->cfg = *cfg;
    self->t = t0;
    self->t_old = t0;
    self->t_bound = t_bound;
    self->h_abs = first_step;
    for (int i = 0; i < n; i++) {
        self->x[i] = x0[i];
    }
    fun(data, t0, self->x, self->f);
}

/* Stages of a step of the size h, the new states, and their derivatives (rk_step of
 * scipy) */
static void rk45_stages(RK45 *self, double h, double complex *x_new)
{
    int n = self->n;
    double complex x_stage[SIM_MAX_STATES];
    for (int i = 0; i < n; i++) {
        self->K[0][i] = self->f[i];
    }
    for (int s = 1; s < 6; s++) {
        for (int i = 0; i < n; i++) {
            double complex dx = 0.0;
            for (int j = 0; j < s; j++) {
                dx += self->K[j][i] * RK45_A[s][j];
            }
            x_stage[i] = self->x[i] + dx * h;
        }
        self->fun(self->data, self->t + RK45_C[s] * h, x_stage, self->K[s]);
    }
    for (int i = 0; i < n; i++) {
        double complex dx = 0.0;
        for (int j = 0; j < 6; j++) {
            dx += self->K[j][i] * RK45_B[j];
        }
        x_new[i] = self->x[i] + h * dx;
    }
    self->fun(self->data, self->t + h, x_new, self->K[6]);
}

/* RMS norm of the error estimate relative to the tolerances */
static double rk45_error_norm(const RK45 *self, double h, const double complex *x_new)
{
    double sum_re = 0.0, sum_im = 0.0;
    for (int i = 0; i < self->n; i++) {
        double complex err = 0.0;
        for (int j = 0; j < 7; j++) {
            err += self->K[j][i] * RK45_E[j];
        }
        double scale =
            self->cfg.atol + fmax(cabs(self->x[i]), cabs(x_new[i])) * self->cfg.rtol;
        err = err * h / scale;
        sum_re += creal(err) * creal(err);
        sum_im += cimag(err) * cimag(err);
    }
    return sqrt(sum_re + sum_im) / pow(self->n, 0.5);
}

static int rk45_step(RK45 *self)
{
    double t = self->t;
    if (t == self->t_bound) {
        self->t_old = t;
        return RK45_FINISHED;
    }
    double min_step = 10.0 * fabs(nextafter(t, INFINITY) - t);
    double h_abs = self->h_abs;
    if (h_abs > self->cfg.max_step) {
        h_abs = self->cfg.max_step;
    } else if (h_abs < min_step) {
        h_abs = min_step;
    }

    double complex x_new[SIM_MAX_STATES];
    double t_new;
    int rejected = 0;
    for (;;) {
        if (h_abs < min_step) {
            return RK45_FAILED;
        }
        t_new = t + h_abs;
        if (t_new - self->t_bound > 0.0) {
            t_new = self->t_bound;
        }
        double h = t_new - t;
        h_abs = fabs(h);
        rk45_stages(self, h, x_new);
        double error_norm = rk45_error_norm(self, h, x_new);
        if (error_norm < 1.0) {
            double factor = RK45_MAX_FACTOR;
            if (error_norm != 0.0) {
                factor = fmin(RK45_MAX_FACTOR,
                              RK45_SAFETY * pow(error_norm, RK45_ERROR_EXPONENT));
            }
            if (rejected) {
                factor = fmin(1.0, factor);
            }
            h_abs *= factor;
            break;
        }
        h_abs *=
            fmax(RK45_MIN_FACTOR, RK45_SAFETY * pow(error_norm, RK45_ERROR_EXPONENT));
        rejected = 1;
    }

    self->t_old = t;
    self->t = t_new;
    self->h_abs = h_abs;
    for (int i = 0; i < self->n; i++) {
        self->x_old[i] = self->x[i];
        self->x[i] = x_new[i];
        self->f[i] = self->K[6][i];
    }
    return (self->t - self->t_bound >= 0.0) ? RK45_FINISHED : RK45_RUNNING;
}

static void rk45_dense_output(const RK45 *self, double t, double complex *x)
{
    double h = self->t - self->t_old;
    double p[4];
    p[0] = (t - self->t_old) / h;
    for (int k = 1; k < 4; k++) {
        p[k] = p[k - 1] * p[0];
    }
    for (int i = 0; i < self->n; i++) {
        double complex y = 0.0;
        for (int k = 0; k < 4; k++) {
            double complex q = 0.0;
            for (int j = 0; j < 7; j++) {
                q += self->K[j][i] * RK45_P[j][k];
            }
            y += q * p[k];
        }
        x[i] = h * y + self->x_old[i];
    }
}

/* Modulator ---------------------------------------------------------------- */

static void modulator_init(Modulator *self, int carrier, double N, double t_d,
                           int delay, int single_update)
{
    self->carrier = carrier;
    self->N = N;
    self->t_d = t_d;
    self->single_update = single_update;
    self->delay = (delay < DELAY_MAX_LENGTH) ? delay : DELAY_MAX_LENGTH;
    self->head = 0;
    for (int j = 0; j < DELAY_MAX_LENGTH; j++) {
        for (int k = 0; k < 3; k++) {
            self->buffer[j][k] = 0.0;
        }
    }
    self->rising_edge = 1;
    self->has_command = 0;
    for (int k = 0; k < 3; k++) {
        self->command[k] = 0.0;
        self->remaining_dead_time[k] = 0.0;
    }
}

/* Computational delay (Delay.__call__): the oldest duty ratios out, d_abc in */
static void modulator_delay(Modulator *self, const double d_abc[3], double out[3])
{
    if (self->delay == 0) {
        for (int k = 0; k < 3; k++) {
            out[k] = d_abc[k];
        }
        return;
    }
    double *slot = self->buffer[self->head];
    for (int k = 0; k < 3; k++) {
        out[k] = slot[k];
        slot[k] = d_abc[k];
    }
    self->head = (self->head + 1) % self->delay;
}

/* Gate signals averaged over the half carrier period T_half (averaged_gate_signals) */
static void averaged_gate_signals(const double d_abc[3], double t_d, double T_half,
                                  double q_abc[3], double b_abc[3])
{
    for (int k = 0; k < 3; k++) {
        double d = d_abc[k];
        double delta = (d > 0.0 && d < 1.0) ? t_d / (2.0 * T_half) : 0.0;
        double q_hi = fmax(d - delta, 0.0);
        double q_lo = fmax(1.0 - d - delta, 0.0);
        q_abc[k] = q_hi;
        b_abc[k] = 1.0 - q_hi - q_lo;
    }
}

/* Append a switching interval to the sequence */
static int append_interval(SwitchingSequence *seq, double t_step, const double q_abc[3],
                           const double b_abc[3])
{
    if (seq->n == SIM_MAX_SWITCHING) {
        return SIM_ERROR_SWITCHING;
    }
    seq->t_step[seq->n] = t_step;
    for (int k = 0; k < 3; k++) {
        seq->q_abc[seq->n][k] = q_abc[k];
        seq->b_abc[seq->n][k] = b_abc[k];
    }
    seq->n++;
    return SIM_OK;
}

/* Carrier comparison over the half carrier period T_s (CarrierComparison.__call__) */
static int carrier_comparison(Modulator *self, double T_s, const double d_abc[3],
                              SwitchingSequence *seq)
{
    double t_d = self->t_d;
    double command[3], t_sw[3], t_on[3];
    for (int k = 0; k < 3; k++) {
        double d = nearbyint(self->N * d_abc[k]) / self->N;
        command[k] = (self->rising_edge ? d == 1.0 : d > 0.0) ? 1.0 : 0.0;
        t_sw[k] =
            (d > 0.0 && d < 1.0) ? T_s * (self->rising_edge ? 1.0 - d : d) : INFINITY;
        double old_command = self->has_command ? self->command[k] : command[k];
        t_on[k] = (command[k] != old_command) ? t_d : self->remaining_dead_time[k];
    }

    /* Instants of the switching and the ends of the blanking, sorted and unique */
    double candidates[10] = {0.0}, t_n[10];
    int m = 0;
    for (int k = 0; k < 3; k++) {
        candidates[1 + k] = t_sw[k];
        candidates[4 + k] = fmin(t_on[k], t_sw[k]);
        candidates[7 + k] = t_sw[k] + t_d;
    }
    for (int j = 0; j < 10; j++) {
        double t = candidates[j];
        if (!(t < T_s)) {
            continue;
        }
        int i = m;
        while (i > 0 && t_n[i - 1] > t) {
            i--;
        }
        if (i > 0 && t_n[i - 1] == t) {
            continue;
        }
        for (int l = m; l > i; l--) {
            t_n[l] = t_n[l - 1];
        }
        t_n[i] = t;
        m++;
    }

    for (int i = 0; i < m; i++) {
        double q_abc[3], b_abc[3];
        for (int k = 0; k < 3; k++) {
            int switched = t_n[i] >= t_sw[k];
            int blanked = t_n[i] < (switched ? t_sw[k] + t_d : t_on[k]);
            double state = switched ? 1.0 - command[k] : command[k];
            b_abc[k] = blanked ? 1.0 : 0.0;
            q_abc[k] = blanked ? 0.0 : state;
        }
        double t_end = (i + 1 < m) ? t_n[i + 1] : T_s;
        if (append_interval(seq, t_end - t_n[i], q_abc, b_abc)) {
            return SIM_ERROR_SWITCHING;
        }
    }

    for (int k = 0; k < 3; k++) {
        int switches = t_sw[k] < T_s;
        double t_end = switches ? t_sw[k] + t_d : t_on[k];
        self->command[k] = switches ? 1.0 - command[k] : command[k];
        self->remaining_dead_time[k] = fmax(t_end - T_s, 0.0);
    }
    self->has_command = 1;
    self->rising_edge = !self->rising_edge;
    return SIM_OK;
}

static int modulator_modulate(Modulator *self, double T_s, const double d_abc[3],
                              SwitchingSequence *seq)
{
    /* The sampling period consists of one half carrier period with double update and
     * of two with single update */
    int n = self->single_update ? 2 : 1;
    seq->n = 0;
    for (int j = 0; j < n; j++) {
        double d[3];
        modulator_delay(self, d_abc, d);
        if (self->carrier) {
            if (carrier_comparison(self, T_s / n, d, seq)) {
                return SIM_ERROR_SWITCHING;
            }
        } else {
            double q_abc[3], b_abc[3];
            averaged_gate_signals(d, self->t_d, T_s / n, q_abc, b_abc);
            if (append_interval(seq, T_s / n, q_abc, b_abc)) {
                return SIM_ERROR_SWITCHING;
            }
        }
    }
    return SIM_OK;
}

/* Converter ---------------------------------------------------------------- */

static void converter_set_gate_signals(VoltageSourceConverter *self,
                                       const double q_abc[3], const double b_abc[3],
                                       double complex i_c_ab)
{
    /* During blanking, the current direction determines which diode conducts */
    double i_abc[3], s_abc[3], q[3];
    complex2abc(i_c_ab, i_abc);
    for (int k = 0; k < 3; k++) {
        s_abc[k] = sign(i_abc[k]);
        q[k] = q_abc[k] + 0.5 * b_abc[k] * (1.0 - s_abc[k]);
    }
    self->q_c_ab = abc2complex(q);
    self->s_c_ab = abc2complex(s_abc);
}

static double complex converter_voltage(const VoltageSourceConverter *self,
                                        double u_dc)
{
    return self->q_c_ab * u_dc - self->u_th * self->s_c_ab;
}

static double converter_dc_current(const VoltageSourceConverter *self,
                                   double complex i_c_ab)
{
    return 1.5 * creal(self->q_c_ab * conj(i_c_ab));
}

/* DC bus ------------------------------------------------------------------- */

static int dc_bus_states(const DCBus *self)
{
    switch (self->type) {
    case DC_BUS_CAPACITOR:
        return 1;
    case DC_BUS_DIODE_BRIDGE:
        return 3;
    default:
        return 0;
    }
}

static const char *dc_bus_state_names(const DCBus *self)
{
    switch (self->type) {
    case DC_BUS_CAPACITOR:
        return "u_dc";
    case DC_BUS_DIODE_BRIDGE:
        return "u_dc i_L exp_j_theta_g";
    default:
        return "";
    }
}

static double dc_bus_voltage(const DCBus *self, const double complex *x, double u_dc)
{
    return (self->type == DC_BUS_NONE) ? u_dc : creal(x[0]);
}

static void dc_bus_rhs(const DCBus *self, const double complex *x, double i_dc_int,
                       double complex *dx)
{
    /* Net current fed to the capacitor (DCBus.compute_dc_current), without an
     * external DC current */
    double i_dc = 0.0 - i_dc_int;
    if (self->type == DC_BUS_CAPACITOR) {
        dx[0] = i_dc / self->C_dc;
    } else if (self->type == DC_BUS_DIODE_BRIDGE) {
        /* Output voltage of the diode bridge (DiodeBridgeDCBus.compute_voltages) */
        double u_g_abc[3];
        complex2abc(self->u_g * x[2], u_g_abc);
        double u_max = fmax(fmax(u_g_abc[0], u_g_abc[1]), u_g_abc[2]);
        double u_min = fmin(fmin(u_g_abc[0], u_g_abc[1]), u_g_abc[2]);
        double u_di = u_max - u_min;
        double i_L = creal(x[1]);
        dx[0] = (i_L + i_dc) / self->C_dc;
        double d_i_L = (u_di - creal(x[0])) / self->L_dc;
        /* The inductor current cannot be negative due to the diode bridge */
        if (i_L < 0.0 && d_i_L < 0.0) {
            d_i_L = 0.0;
        }
        dx[1] = d_i_L;
        dx[2] = I * self->w_g * x[2];
    }
}

/* Model -------------------------------------------------------------------- */

static double model_dc_voltage(const Model *self, const double complex *x)
{
    return dc_bus_voltage(&self->dc_bus, x, self->converter.u_dc);
}

static void model_state_names(const Model *self, const char *names, char *buffer,
                              int size)
{
    const char *dc_bus = dc_bus_state_names(&self->dc_bus);
    snprintf(buffer, (size_t)size, "%s%s%s", dc_bus, (*dc_bus != '\0') ? " " : "",
             names);
}

/* Logs --------------------------------------------------------------------- */

static void log_init(Log *self, int width)
{
    self->width = width;
    self->rows = 0;
    self->capacity = 0;
    self->data = NULL;
}

static double *log_append(Log *self)
{
    if (self->rows == self->capacity) {
        int capacity = (self->capacity > 0) ? 2 * self->capacity : 1024;
        double *data = realloc(self->data,
                               (size_t)capacity * (size_t)self->width * sizeof(double));
        if (data == NULL) {
            return NULL;
        }
        self->data = data;
        self->capacity = capacity;
    }
    return self->data + (size_t)self->rows++ * (size_t)self->width;
}

static void log_free(Log *self)
{
    free(self->data);
    log_init(self, self->width);
}

/* Number of doubles in a row of the logged fields, given as space-separated names
 * with the suffixes ":c" (complex, two doubles), ":b" (bool, one double), ":n" (None,
 * not logged), or ":<n>" (an array of n doubles), plus one for the time */
static int log_fields_width(const char *fields)
{
    int width = 1;
    const char *p = fields;
    while (*p != '\0') {
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        int n = 1;
        while (*p != '\0' && *p != ' ' && *p != ':') {
            p++;
        }
        if (*p == ':') {
            p++;
            if (*p == 'c') {
                n = 2;
            } else if (*p == 'n') {
                n = 0;
            } else if (*p >= '0' && *p <= '9') {
                n = (int)strtol(p, NULL, 10);
            }
            while (*p != '\0' && *p != ' ') {
                p++;
            }
        }
        width += n;
    }
    return width;
}

/* Cursor for writing a row of a log */
static void log_put(double **p, double value)
{
    *(*p)++ = value;
}

static void log_put_complex(double **p, double complex value)
{
    log_put(p, creal(value));
    log_put(p, cimag(value));
}

/* Simulation --------------------------------------------------------------- */

static void simulation_init(Simulation *self, Model *mdl, ControlSystem *ctrl,
                            const SolverCfg *cfg, double t0, const double complex *x0,
                            double t_start)
{
    self->mdl = mdl;
    self->ctrl = ctrl;
    self->cfg = *cfg;
    self->t0 = t0;
    for (int i = 0; i < mdl->n; i++) {
        self->x[i] = x0[i];
    }
    self->t_sample = t_start;
    self->n_events = 0;
    self->i_event = 0;
    log_init(&self->log, 1 + 2 * mdl->n + 4);
}

static void simulation_rhs(void *data, double t, const double complex *x,
                           double complex *dx)
{
    const Model *mdl = data;
    mdl->rhs(mdl, t, x, dx);
}

/* Save a solution point with the switching state (Model.save) */
static int simulation_save(Simulation *self, double t, const double complex *x)
{
    double *p = log_append(&self->log);
    if (p == NULL) {
        return SIM_ERROR_MEMORY;
    }
    log_put(&p, t);
    for (int i = 0; i < self->mdl->n; i++) {
        log_put_complex(&p, x[i]);
    }
    log_put_complex(&p, self->mdl->converter.q_c_ab);
    log_put_complex(&p, self->mdl->converter.s_c_ab);
    return SIM_OK;
}

static int all_finite(const double complex *x, int n)
{
    for (int i = 0; i < n; i++) {
        if (!isfinite(creal(x[i])) || !isfinite(cimag(x[i]))) {
            return 0;
        }
    }
    return 1;
}

/* Integrate from t0 to t1 and save the solution (Simulation._integrate). With N_eval,
 * the evenly spaced points of the interval are saved, excluding its end point, at
 * which the dense output gives the final states, as t_eval of solve_ivp. */
static int simulation_integrate(Simulation *self, double t1, int N_eval)
{
    double t0 = self->t0;
    if (t1 <= t0) {
        return SIM_OK;
    }
    Model *mdl = self->mdl;
    RK45 solver;
    /* The whole interval is tried as the first step */
    double first_step = fmin(t1 - t0, self->cfg.max_step);
    rk45_init(&solver, simulation_rhs, mdl, mdl->n, t0, self->x, t1, first_step,
              &self->cfg);
    double complex x_eval[SIM_MAX_STATES];
    double dt_eval = (N_eval > 0) ? (t1 - t0) / N_eval : 0.0;
    int i_eval = 0, status = SIM_OK;
    if (N_eval == 0) {
        status = simulation_save(self, t0, self->x);
    }
    for (int result = RK45_RUNNING; status == SIM_OK && result == RK45_RUNNING;) {
        result = rk45_step(&solver);
        if (result == RK45_FAILED) {
            return SIM_ERROR_STEP_SIZE;
        }
        if (!all_finite(solver.x, mdl->n)) {
            return SIM_ERROR_NOT_FINITE;
        }
        if (N_eval == 0) {
            status = simulation_save(self, solver.t, solver.x);
            continue;
        }
        for (; i_eval <= N_eval && status == SIM_OK; i_eval++) {
            double t = (i_eval < N_eval) ? i_eval * dt_eval + t0 : t1;
            if (t > solver.t) {
                break;
            }
            rk45_dense_output(&solver, t, x_eval);
            if (i_eval < N_eval) {
                status = simulation_save(self, t, x_eval);
            }
        }
    }
    if (status != SIM_OK) {
        return status;
    }
    const double complex *x1 = (N_eval == 0) ? solver.x : x_eval;
    for (int i = 0; i < mdl->n; i++) {
        self->x[i] = x1[i];
    }
    self->t0 = t1;
    mdl->set_states(mdl, self->x);
    return SIM_OK;
}

static int simulation_run(Simulation *self, double t_stop, int N_eval)
{
    Model *mdl = self->mdl;
    for (;;) {
        double t0 = self->t0;
        int due = self->t_sample <= t0 + SIM_T_TOL;
        if (due && t0 > t_stop) {
            return SIM_OK;
        }
        if (due) {
            /* Control, computational delay, and PWM */
            double T_s, d_abc[3];
            int status = self->ctrl->run(self->ctrl, mdl, t0, self->x, &T_s, d_abc);
            if (status != SIM_OK) {
                return status;
            }
            if (!(T_s > SIM_T_TOL)) {
                return SIM_ERROR_SAMPLING;
            }
            if (!isfinite(d_abc[0]) || !isfinite(d_abc[1]) || !isfinite(d_abc[2])) {
                return SIM_ERROR_NOT_FINITE;
            }
            SwitchingSequence seq;
            status = modulator_modulate(&mdl->modulator, T_s, d_abc, &seq);
            if (status != SIM_OK) {
                return status;
            }
            /* Switching instants, accumulated from the interval durations */
            double t_sw = t0;
            for (int k = 0; k < seq.n; k++) {
                if (k > 0) {
                    t_sw += seq.t_step[k - 1];
                }
                self->t_event[k] = t_sw;
                for (int j = 0; j < 3; j++) {
                    self->q_abc[k][j] = seq.q_abc[k][j];
                    self->b_abc[k][j] = seq.b_abc[k][j];
                }
            }
            self->n_events = seq.n;
            self->i_event = 0;
            self->t_sample = t_sw + seq.t_step[seq.n - 1];
        }
        /* Set the switching state due at this instant */
        while (self->i_event < self->n_events &&
               self->t_event[self->i_event] <= t0 + SIM_T_TOL) {
            int k = self->i_event++;
            double complex i_c_ab = mdl->converter_current(mdl, self->x);
            converter_set_gate_signals(&mdl->converter, self->q_abc[k], self->b_abc[k],
                                       i_c_ab);
        }
        /* Integrate until the next sampling or switching instant */
        double t1 = self->t_sample;
        if (self->i_event < self->n_events) {
            t1 = fmin(t1, self->t_event[self->i_event]);
        }
        int status = simulation_integrate(self, t1, N_eval);
        if (status != SIM_OK) {
            return status;
        }
    }
}

static void simulation_free(Simulation *self)
{
    log_free(&self->log);
}

/* API for Python (motulator_export.native) --------------------------------- */

/* The systems of the API embed their simulation as the first member, so that a
 * pointer to a system is a pointer to its simulation. */

int sim_run(Simulation *self, double t_stop, int N_eval)
{
    return simulation_run(self, t_stop, N_eval);
}

/* Model time and states (real and imaginary parts) */
double sim_state(const Simulation *self, double *x)
{
    for (int i = 0; i < self->mdl->n; i++) {
        x[2 * i] = creal(self->x[i]);
        x[2 * i + 1] = cimag(self->x[i]);
    }
    return self->t0;
}

/* Log of the model (control = 0) or of the control system (control = 1) */
Log *sim_log(Simulation *self, int control)
{
    return control ? &self->ctrl->log : &self->log;
}

/* Names of the states of the model (control = 0) or of the logged fields of the
 * control system (control = 1), separated by spaces */
const char *sim_fields(const Simulation *self, int control)
{
    return control ? self->ctrl->fields : self->mdl->states;
}

int log_rows(const Log *self)
{
    return self->rows;
}

int log_width(const Log *self)
{
    return self->width;
}

const double *log_data(const Log *self)
{
    return self->data;
}

void log_clear(Log *self)
{
    self->rows = 0;
}

/* Steps from `before` to after[k] at t > time[k] (StepSignal) */
void signal_steps(Signal *self, double before, int n, const double *time,
                  const double *after)
{
    signal_constant(self, before);
    self->n = (n < SIGNAL_MAX_STEPS) ? n : SIGNAL_MAX_STEPS;
    for (int k = 0; k < self->n; k++) {
        self->time[k] = time[k];
        self->after[k] = after[k];
    }
}

/* Function of time, e.g., a Python callable */
void signal_function(Signal *self, SignalFunction fcn)
{
    signal_constant(self, 0.0);
    self->fcn = fcn;
}

int signal_max_steps(void)
{
    return SIGNAL_MAX_STEPS;
}

/* VoltageSourceConverter: the DC-bus voltage u_dc (constant, without a DC-bus model)
 * and the threshold voltage u_th */
void set_converter(Simulation *self, double u_dc, double u_th)
{
    self->mdl->converter.u_dc = u_dc;
    self->mdl->converter.u_th = u_th;
}

/* PWM model (CarrierComparison if carrier is nonzero, else ZOH) and the delay of the
 * model, see modulator_init */
void set_modulator(Simulation *self, int carrier, double N, double t_d, int delay,
                   int single_update)
{
    modulator_init(&self->mdl->modulator, carrier, N, t_d, delay, single_update);
}

/* DC-bus model (DC_BUS_NONE, DC_BUS_CAPACITOR, or DC_BUS_DIODE_BRIDGE) with its
 * parameters (see DCBus), updating the states of the model */
void set_dc_bus(Simulation *self, int type, double C_dc, double L_dc, double u_g,
                double w_g)
{
    DCBus *dc_bus = &self->mdl->dc_bus;
    dc_bus->type = type;
    dc_bus->C_dc = C_dc;
    dc_bus->L_dc = L_dc;
    dc_bus->u_g = u_g;
    dc_bus->w_g = w_g;
    self->mdl->init_states(self->mdl);
}
