/*
 * Simulation of a continuous-time system model with a discrete-time control system,
 * ported from motulator. See sim.h.
 */

#include <float.h>
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
    if (self->enabled) {
        return 1.5 * creal(self->q_c_ab * conj(i_c_ab));
    }
    /* With blocked gates, the diodes rectify the phase currents */
    double i_abc[3];
    complex2abc(i_c_ab, i_abc);
    return -0.5 * (fabs(i_abc[0]) + fabs(i_abc[1]) + fabs(i_abc[2]));
}

/* Time constant (s) at which the residual current of a floating leg decays */
#define AC_PORT_TAU 1e-4
/* Tolerances of the diode-state transitions: current (A) and voltage (V) */
#define AC_PORT_I_TOL 1e-9
#define AC_PORT_U_TOL 1e-9

/* Transformation from the real and imaginary parts of a space vector to the phase
 * quantities (_A of _ac_port.py), whose transpose times 2/3 is the inverse (_B) */
static void ac_port_transform(double A[3][2])
{
    double r = sqrt(3.0) / 2.0;
    A[0][0] = 1.0;
    A[0][1] = 0.0;
    A[1][0] = -0.5;
    A[1][1] = r;
    A[2][0] = -0.5;
    A[2][1] = -r;
}

/* Solve the n-by-n linear system M x = b (n at most 3) by Gaussian elimination with
 * partial pivoting (numpy.linalg.solve), b being overwritten by x */
static void solve_small(int n, double M[3][3], double b[3])
{
    for (int j = 0; j < n; j++) {
        int pivot = j;
        for (int i = j + 1; i < n; i++) {
            if (fabs(M[i][j]) > fabs(M[pivot][j])) {
                pivot = i;
            }
        }
        if (pivot != j) {
            for (int k = 0; k < n; k++) {
                double tmp = M[j][k];
                M[j][k] = M[pivot][k];
                M[pivot][k] = tmp;
            }
            double tmp = b[j];
            b[j] = b[pivot];
            b[pivot] = tmp;
        }
        for (int i = j + 1; i < n; i++) {
            double l = M[i][j] / M[j][j];
            for (int k = j; k < n; k++) {
                M[i][k] -= l * M[j][k];
            }
            b[i] -= l * b[j];
        }
    }
    for (int i = n - 1; i >= 0; i--) {
        double sum = b[i];
        for (int k = i + 1; k < n; k++) {
            sum -= M[i][k] * b[k];
        }
        b[i] = sum / M[i][i];
    }
}

static void leg_voltages(const ACPort *port, double u_dc, const double diodes[3],
                         double u_abc[3])
{
    int floating[3], n_float = 0;
    for (int k = 0; k < 3; k++) {
        floating[k] = diodes[k] == 0.0;
        n_float += floating[k];
    }
    if (n_float == 3) {
        /* All the legs float: the voltage drives the current to zero, centered
         * between the rails */
        double M[3][3] = {{port->Y[0][0], port->Y[0][1], 0.0},
                          {port->Y[1][0], port->Y[1][1], 0.0},
                          {0.0, 0.0, 0.0}};
        double b[3] = {creal(port->i), cimag(port->i), 0.0};
        solve_small(2, M, b);
        double complex u = port->e - complex_from(b[0], b[1]) / AC_PORT_TAU;
        complex2abc(u, u_abc);
        double u_max = fmax(fmax(u_abc[0], u_abc[1]), u_abc[2]);
        double u_min = fmin(fmin(u_abc[0], u_abc[1]), u_abc[2]);
        for (int k = 0; k < 3; k++) {
            u_abc[k] += (u_dc - u_max - u_min) / 2.0;
        }
        return;
    }
    for (int k = 0; k < 3; k++) {
        u_abc[k] = (diodes[k] < 0.0) ? u_dc : 0.0;
    }
    if (n_float == 0) {
        return;
    }
    /* The current derivatives are affine in the leg voltages, M u_abc + c, from which
     * the voltages of the floating legs are solved */
    double A[3][2], AY[3][2], M[3][3], c[3], i_abc[3];
    ac_port_transform(A);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            AY[i][j] = A[i][0] * port->Y[0][j] + A[i][1] * port->Y[1][j];
        }
        for (int j = 0; j < 3; j++) {
            M[i][j] =
                AY[i][0] * (2.0 / 3.0 * A[j][0]) + AY[i][1] * (2.0 / 3.0 * A[j][1]);
        }
    }
    double Ye[2];
    for (int j = 0; j < 2; j++) {
        Ye[j] = port->Y[j][0] * creal(port->e) + port->Y[j][1] * cimag(port->e);
    }
    for (int i = 0; i < 3; i++) {
        c[i] = -A[i][0] * Ye[0] + -A[i][1] * Ye[1];
    }
    complex2abc(port->i, i_abc);
    int f[3], n_f = 0;
    for (int k = 0; k < 3; k++) {
        if (floating[k]) {
            f[n_f++] = k;
        }
    }
    double M_f[3][3], rhs[3];
    for (int i = 0; i < n_f; i++) {
        double coupling = 0.0;
        for (int k = 0; k < 3; k++) {
            if (!floating[k]) {
                coupling += M[f[i]][k] * u_abc[k];
            }
        }
        rhs[i] = -i_abc[f[i]] / AC_PORT_TAU - c[f[i]] - coupling;
        for (int j = 0; j < n_f; j++) {
            M_f[i][j] = M[f[i]][f[j]];
        }
    }
    solve_small(n_f, M_f, rhs);
    for (int i = 0; i < n_f; i++) {
        u_abc[f[i]] = rhs[i];
    }
}

static double diode_margin(const ACPort *port, double u_dc, const double diodes[3],
                           const double u_abc[3])
{
    double margin = INFINITY, i_abc[3];
    complex2abc(port->i, i_abc);
    int conducting = 0;
    double i_margin = INFINITY, u_margin = INFINITY;
    for (int k = 0; k < 3; k++) {
        if (diodes[k] != 0.0) {
            conducting++;
            i_margin = fmin(i_margin, diodes[k] * i_abc[k]);
        } else {
            u_margin = fmin(u_margin, fmin(u_abc[k], u_dc - u_abc[k]));
        }
    }
    if (conducting > 0) {
        margin = i_margin + AC_PORT_I_TOL;
    }
    if (conducting < 3) {
        margin = fmin(margin, u_margin + AC_PORT_U_TOL);
    }
    return margin;
}

static void update_diodes(const ACPort *port, double u_dc, double diodes[3], int latch)
{
    double i_abc[3];
    complex2abc(port->i, i_abc);
    int positive = 0, negative = 0;
    for (int k = 0; k < 3; k++) {
        if (latch) {
            diodes[k] = (fabs(i_abc[k]) > AC_PORT_I_TOL) ? sign(i_abc[k]) : 0.0;
        } else if (diodes[k] * i_abc[k] <= 0.0) {
            diodes[k] = 0.0;
        }
        positive |= diodes[k] > 0.0;
        negative |= diodes[k] < 0.0;
    }
    /* The current circulates between the legs conducting in opposite directions.
     * Otherwise, the conducting legs carry only the residual currents of the floating
     * legs, so they float too. */
    if (!(positive && negative)) {
        for (int k = 0; k < 3; k++) {
            diodes[k] = 0.0;
        }
    }
    for (int n = 0; n < 3; n++) { /* Each round clamps at least one leg */
        double u_abc[3];
        leg_voltages(port, u_dc, diodes, u_abc);
        int clamped = 0;
        for (int k = 0; k < 3; k++) {
            if (diodes[k] == 0.0 && (u_abc[k] < 0.0 || u_abc[k] > u_dc)) {
                diodes[k] = (u_abc[k] < 0.0) ? 1.0 : -1.0;
                clamped = 1;
            }
        }
        if (!clamped) {
            break;
        }
    }
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

/* Voltage (V) between the rails of the converter, including the drops of the diodes
 * (VoltageSourceConverter._rail_voltage) */
static double model_rail_voltage(const Model *self, const double complex *x)
{
    return model_dc_voltage(self, x) + 2.0 * self->converter.u_th;
}

static double complex model_converter_voltage(const Model *self, double t,
                                              const double complex *x, double *margin)
{
    const VoltageSourceConverter *converter = &self->converter;
    if (converter->enabled) {
        if (margin != NULL) {
            *margin = INFINITY;
        }
        return converter_voltage(converter, model_dc_voltage(self, x));
    }
    ACPort port;
    self->ac_port(self, t, x, &port);
    double u_dc = model_rail_voltage(self, x), u_abc[3];
    leg_voltages(&port, u_dc, converter->diodes, u_abc);
    if (margin != NULL) {
        *margin = diode_margin(&port, u_dc, converter->diodes, u_abc);
    }
    return abc2complex(u_abc);
}

static void model_set_enable(Model *self, double t, const double complex *x,
                             int enabled)
{
    VoltageSourceConverter *converter = &self->converter;
    if (!enabled && converter->enabled) {
        ACPort port;
        self->ac_port(self, t, x, &port);
        update_diodes(&port, model_rail_voltage(self, x), converter->diodes, 1);
    }
    converter->enabled = enabled;
}

/* Update the diode states of the converter with blocked gates at a transition
 * (VoltageSourceConverter.update_diodes) */
static void model_update_diodes(Model *self, double t, const double complex *x)
{
    ACPort port;
    self->ac_port(self, t, x, &port);
    update_diodes(&port, model_rail_voltage(self, x), self->converter.diodes, 0);
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
    log_init(&self->log, 1 + 2 * mdl->n + 7);
}

static void simulation_rhs(void *data, double t, const double complex *x,
                           double complex *dx)
{
    const Model *mdl = data;
    mdl->rhs(mdl, t, x, dx);
}

/* Save a solution point with the switching state, the gate enable, and the voltage of
 * the converter with blocked gates, evaluated at the point (Model.save) */
static int simulation_save(Simulation *self, double t, const double complex *x)
{
    const Model *mdl = self->mdl;
    const VoltageSourceConverter *converter = &mdl->converter;
    double *p = log_append(&self->log);
    if (p == NULL) {
        return SIM_ERROR_MEMORY;
    }
    log_put(&p, t);
    for (int i = 0; i < mdl->n; i++) {
        log_put_complex(&p, x[i]);
    }
    log_put_complex(&p, converter->q_c_ab);
    log_put_complex(&p, converter->s_c_ab);
    log_put(&p, converter->enabled);
    double complex u_c_ab = NAN;
    if (!converter->enabled) {
        double complex x_set[SIM_MAX_STATES];
        for (int i = 0; i < mdl->n; i++) {
            x_set[i] = x[i];
        }
        mdl->set_states(mdl, x_set); /* As Model._evaluate_outputs */
        u_c_ab = model_converter_voltage(mdl, t, x_set, NULL);
    }
    log_put_complex(&p, u_c_ab);
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

/* Event of the transitions of the diode states of the converter (_DiodeEvent): the
 * margin of the diode states on the dense output of the latest step */
typedef struct {
    const Model *mdl;
    const RK45 *solver;
} DiodeEvent;

static double diode_event(double t, void *data)
{
    const DiodeEvent *event = data;
    double complex x[SIM_MAX_STATES];
    rk45_dense_output(event->solver, t, x);
    event->mdl->set_states(event->mdl, x); /* As the event function of motulator */
    double margin;
    model_converter_voltage(event->mdl, t, x, &margin);
    return margin;
}

/* Margin of the diode states at the time t and the states x, set as by Model.set_states
 * (the converter with blocked gates) */
static double simulation_margin(const Model *mdl, double t, const double complex *x)
{
    double complex x_set[SIM_MAX_STATES];
    for (int i = 0; i < mdl->n; i++) {
        x_set[i] = x[i];
    }
    mdl->set_states(mdl, x_set);
    double margin;
    model_converter_voltage(mdl, t, x_set, &margin);
    return margin;
}

/* Integrate from t0 to t1 and save the solution (Simulation._integrate). With N_eval,
 * the evenly spaced points of the interval are saved, excluding its end point, at
 * which the dense output gives the final states, as t_eval of solve_ivp. With the
 * gates of the converter blocked, the transitions of its diode states are located as
 * the terminal events of solve_ivp (the margin decreasing to zero, the root found by
 * brentq with the tolerances 4*eps), at which the diode states are updated and the
 * integration is restarted. */
static int simulation_integrate(Simulation *self, double t1, int N_eval)
{
    double t_start = self->t0;
    if (t1 <= t_start) {
        return SIM_OK;
    }
    Model *mdl = self->mdl;
    int events = !mdl->converter.enabled;
    /* The evaluation points over the whole interval, kept at the restarts */
    double dt_eval = (N_eval > 0) ? (t1 - t_start) / N_eval : 0.0;
    int i_eval = 0;
    for (int transition = 0; transition < SIM_MAX_TRANSITIONS; transition++) {
        double t0 = self->t0;
        int rows = self->log.rows; /* Points saved before this solution */
        RK45 solver;
        /* The whole interval is tried as the first step */
        double first_step = fmin(t1 - t0, self->cfg.max_step);
        rk45_init(&solver, simulation_rhs, mdl, mdl->n, t0, self->x, t1, first_step,
                  &self->cfg);
        DiodeEvent event = {mdl, &solver};
        double g = events ? simulation_margin(mdl, t0, self->x) : 0.0;
        double complex x_eval[SIM_MAX_STATES], x_event[SIM_MAX_STATES];
        const double complex *x_end = self->x;
        double t_end = t1;
        int status = SIM_OK, terminated = 0;
        if (N_eval == 0) {
            status = simulation_save(self, t0, self->x);
        }
        for (int result = RK45_RUNNING; status == SIM_OK && result == RK45_RUNNING;) {
            result = rk45_step(&solver);
            if (result == RK45_FAILED) {
                status = SIM_ERROR_STEP_SIZE;
                break;
            }
            if (!all_finite(solver.x, mdl->n)) {
                status = SIM_ERROR_NOT_FINITE;
                break;
            }
            double t = solver.t;
            const double complex *x = solver.x;
            if (events) {
                double g_new = simulation_margin(mdl, solver.t, solver.x);
                /* Active with the margin decreasing (find_active_events) */
                if (g >= 0.0 && g_new <= 0.0) {
                    t = brentq_tol(diode_event, solver.t_old, solver.t, &event,
                                   4.0 * DBL_EPSILON, 4.0 * DBL_EPSILON);
                    rk45_dense_output(&solver, t, x_event);
                    x = x_event;
                    terminated = 1;
                }
                g = g_new;
            }
            if (N_eval == 0) {
                status = simulation_save(self, t, x);
                x_end = x;
            } else {
                for (; i_eval <= N_eval && status == SIM_OK; i_eval++) {
                    double t_i = (i_eval < N_eval) ? i_eval * dt_eval + t_start : t1;
                    if (t_i > t) {
                        break;
                    }
                    rk45_dense_output(&solver, t_i, x_eval);
                    if (i_eval < N_eval) {
                        status = simulation_save(self, t_i, x_eval);
                    }
                }
                x_end = terminated ? x_event : x_eval;
            }
            if (terminated) {
                t_end = t;
                break;
            }
        }
        if (status != SIM_OK) {
            /* Nothing of the failed solution is saved, as in motulator, so the model
             * remains at its start */
            self->log.rows = rows;
            return status;
        }
        for (int i = 0; i < mdl->n; i++) {
            self->x[i] = x_end[i];
        }
        self->t0 = t_end;
        mdl->set_states(mdl, self->x);
        if (!terminated) {
            return SIM_OK;
        }
        model_update_diodes(mdl, self->t0, self->x);
    }
    return SIM_ERROR_TRANSITIONS;
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
            /* The gate enable takes effect at the sampling instant, so that the
             * control system measures the converter with blocked gates */
            int enabled = self->ctrl->get_enable(self->ctrl);
            if (enabled != mdl->converter.enabled) {
                model_set_enable(mdl, t0, self->x, enabled);
            }
            /* Control, computational delay, and PWM */
            double T_s, d_abc[3];
            int status =
                self->ctrl->run(self->ctrl, mdl, t0, self->x, enabled, &T_s, d_abc);
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
            if (!mdl->converter.enabled) {
                /* The duty ratios pass through the delay and the PWM model, but the
                 * gate signals are zero for the whole sampling period */
                const double off[3] = {0.0, 0.0, 0.0};
                seq.n = 0;
                append_interval(&seq, T_s, off, off);
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

/* Switching state and current-direction vector of the converter (real and
 * imaginary parts), and its diode states with blocked gates, returning its gate
 * enable */
int sim_converter_state(const Simulation *self, double *q, double *diodes)
{
    const VoltageSourceConverter *converter = &self->mdl->converter;
    q[0] = creal(converter->q_c_ab);
    q[1] = cimag(converter->q_c_ab);
    q[2] = creal(converter->s_c_ab);
    q[3] = cimag(converter->s_c_ab);
    for (int k = 0; k < 3; k++) {
        diodes[k] = converter->diodes[k];
    }
    return converter->enabled;
}

/* Gate enable and diode states of the converter, e.g., of a model whose gates were
 * blocked by an earlier simulation */
void sim_set_converter_state(Simulation *self, int enabled, const double *diodes)
{
    VoltageSourceConverter *converter = &self->mdl->converter;
    converter->enabled = enabled;
    for (int k = 0; k < 3; k++) {
        converter->diodes[k] = diodes[k];
    }
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
