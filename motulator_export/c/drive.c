/*
 * Continuous-time models of machine drives, ported from motulator. See drive.h.
 */

#include "drive.h"

static void sm_magnetic_map(const SynchronousMachine *self, double complex psi_s_dq,
                            double theta_m, double complex *i_s_dq, double *tau_M)
{
    double tau_m; /* Torque per pole pair */
    if (self->harmonics) {
        gradnet_current_map_harmonics(&self->current_map, self->k, psi_s_dq, theta_m,
                                      i_s_dq, &tau_m);
    } else {
        *i_s_dq = complex_from((creal(psi_s_dq) - self->psi_f) / self->L_d,
                               cimag(psi_s_dq) / self->L_q);
        tau_m = 1.5 * cimag(*i_s_dq * conj(psi_s_dq));
    }
    *tau_M = self->n_p * tau_m;
}

static void im_magnetic_map(const InductionMachine *self, double complex psi_s_dq,
                            double complex psi_r_dq, double complex *i_s_dq,
                            double complex *i_r_dq, double *tau_M)
{
    if (self->inv_gamma) {
        *i_s_dq = (psi_s_dq - psi_r_dq) / self->L_sgm;
        *i_r_dq = psi_r_dq / self->L_M - *i_s_dq;
    } else {
        *i_r_dq = (psi_r_dq - psi_s_dq) / self->L_ell;
        *i_s_dq = psi_s_dq / self->L_s - *i_r_dq;
    }
    double tau_m = 1.5 * cimag(*i_s_dq * conj(psi_s_dq)); /* Torque per pole pair */
    *tau_M = self->n_p * tau_m;
}

/* Indices of the electrical rotor angle and of the mechanical rotor angle and speed,
 * which follow it, after the states of the DC bus */
static int drive_theta_m(const Drive *self)
{
    return self->induction ? 2 : 1;
}

static int drive_theta_M(const Drive *self)
{
    return drive_theta_m(self) + 1;
}

static int drive_w_M(const Drive *self)
{
    return drive_theta_m(self) + 2;
}

/* Stator current (A) in rotor coordinates as a function of the stator flux linkage
 * psi_s_dq (Vs) in rotor coordinates and the electrical rotor angle theta_m (rad), at
 * the rotor flux linkage psi_r_dq (Vs) of the induction machine */
static double complex drive_stator_current(const Drive *self, double complex psi_s_dq,
                                           double complex psi_r_dq, double theta_m)
{
    double complex i_s_dq, i_r_dq;
    double tau_M;
    if (self->induction) {
        im_magnetic_map(&self->im, psi_s_dq, psi_r_dq, &i_s_dq, &i_r_dq, &tau_M);
    } else {
        sm_magnetic_map(&self->sm, psi_s_dq, theta_m, &i_s_dq, &tau_M);
    }
    return i_s_dq;
}

/* Outputs of the machine at the states x of the machine, theta_m being the wrapped
 * electrical rotor angle (SynchronousMachine._compute_outputs or
 * InductionMachine._compute_outputs): the stator current i_s_dq (A) in rotor
 * coordinates, the rotor current i_r_dq (A) in rotor coordinates of the induction
 * machine (zero for the synchronous machine), and the torque tau_M (Nm). Returns the
 * stator current in stator coordinates. */
static double complex drive_machine_outputs(const Drive *self, const double complex *x,
                                            double theta_m, double complex *i_s_dq,
                                            double complex *i_r_dq, double *tau_M)
{
    if (self->induction) {
        im_magnetic_map(&self->im, x[DRIVE_psi_s], x[DRIVE_psi_r], i_s_dq, i_r_dq,
                        tau_M);
    } else {
        sm_magnetic_map(&self->sm, x[DRIVE_psi_s], theta_m, i_s_dq, tau_M);
        *i_r_dq = 0.0;
    }
    return *i_s_dq * cexp(I * theta_m);
}

/* Stator current (A) in stator coordinates, at the states set by set_states (the
 * angle is not wrapped again) */
static double complex drive_current(const Model *base, const double complex *x)
{
    const Drive *self = (const Drive *)base;
    const double complex *x_m = x + dc_bus_states(&base->dc_bus);
    double complex i_s_dq, i_r_dq;
    double tau_M;
    double theta_m = creal(x_m[drive_theta_m(self)]);
    return drive_machine_outputs(self, x_m, theta_m, &i_s_dq, &i_r_dq, &tau_M);
}

/* Mechanical rotor speed at the time t: the state of MechanicalSystem, or the
 * external speed of ExternalRotorSpeed (ExternalRotorSpeed.set_outputs) */
static double drive_speed(const Drive *self, double t, const double complex *x_m)
{
    if (self->mechanics.external) {
        return signal_value(&self->mechanics.w_M, t);
    }
    return creal(x_m[drive_w_M(self)]);
}

/* Number of pole pairs and stator resistance (Ohm) of the machine */
static double drive_n_p(const Drive *self)
{
    return self->induction ? self->im.n_p : self->sm.n_p;
}

static double drive_R_s(const Drive *self)
{
    return self->induction ? self->im.R_s : self->sm.R_s;
}

/* State derivatives (DCBus.rhs, SynchronousMachine.rhs or InductionMachine.rhs, and
 * MechanicalSystem.rhs or ExternalRotorSpeed.rhs). The electrical rotor angle is
 * wrapped, as Subsystem.set_states does before the outputs are computed. */
static void drive_rhs(const Model *base, double t, const double complex *x,
                      double complex *dx)
{
    const Drive *self = (const Drive *)base;
    const MechanicalSystem *mechanics = &self->mechanics;
    int k = dc_bus_states(&base->dc_bus);
    const double complex *x_m = x + k;
    double complex *dx_m = dx + k;
    double complex u_s_ab = model_converter_voltage(base, t, x, NULL);
    double w_M = drive_speed(self, t, x_m);

    /* Machine, in rotor coordinates */
    int k_theta_m = drive_theta_m(self);
    double theta_m = wrap(creal(x_m[k_theta_m]));
    double complex i_s_dq, i_r_dq;
    double tau_M;
    double complex i_s_ab =
        drive_machine_outputs(self, x_m, theta_m, &i_s_dq, &i_r_dq, &tau_M);
    double complex u_s_dq = u_s_ab * cexp(-I * theta_m);
    double w_m = drive_n_p(self) * w_M;
    dx_m[DRIVE_psi_s] =
        u_s_dq - drive_R_s(self) * i_s_dq - I * w_m * x_m[DRIVE_psi_s];
    if (self->induction) {
        /* The rotor winding is short-circuited, so only the cage remains */
        dx_m[DRIVE_psi_r] = -self->im.R_r * i_r_dq;
    }
    dx_m[k_theta_m] = w_m;

    /* Mechanics */
    if (mechanics->external) {
        dx_m[drive_theta_M(self)] = w_M;
    } else {
        double tau_L_tot = signal_value(&mechanics->B_L, w_M) * w_M +
                           signal_value(&mechanics->tau_L, t);
        dx_m[drive_theta_M(self)] = x_m[drive_w_M(self)];
        dx_m[drive_w_M(self)] = (tau_M - tau_L_tot) / mechanics->J;
    }

    /* DC bus */
    dc_bus_rhs(&base->dc_bus, x, converter_dc_current(&base->converter, i_s_ab), dx);
}

/* Keep the angles within [-pi, pi) at the start of an integration interval */
static void drive_set_states(const Model *base, double complex *x)
{
    const Drive *self = (const Drive *)base;
    dc_bus_set_states(&base->dc_bus, x);
    double complex *x_m = x + dc_bus_states(&base->dc_bus);
    x_m[drive_theta_m(self)] = wrap(creal(x_m[drive_theta_m(self)]));
    x_m[drive_theta_M(self)] = wrap(creal(x_m[drive_theta_M(self)]));
}

/* Solution of the 2-by-2 system J v = z, the vectors as complex numbers */
static double complex drive_solve(const double J[2][2], double complex z)
{
    double M[3][3] = {{J[0][0], J[0][1], 0.0}, {J[1][0], J[1][1], 0.0}, {0.0}};
    double b[3] = {creal(z), cimag(z), 0.0};
    solve_small(2, M, b);
    return complex_from(b[0], b[1]);
}

/* States of the machine at which its AC port is computed: the stator flux linkage,
 * the rotor flux linkage of the induction machine, and the electrical rotor angle with
 * its complex exponential */
typedef struct {
    const Drive *drive;
    double complex psi_s_dq;
    double complex psi_r_dq;
    double theta_m;
    double complex exp_j_theta_m;
} DriveACPortStates;

/* Stator current in stator coordinates after a step d_psi_s_ab of the stator flux
 * linkage in stator coordinates, the other states being constant (i_s_ab of
 * _compute_ac_port) */
static double complex drive_i_s_ab(const void *data, double complex d_psi_s_ab)
{
    const DriveACPortStates *s = data;
    double complex psi = s->psi_s_dq + d_psi_s_ab * conj(s->exp_j_theta_m);
    return drive_stator_current(s->drive, psi, s->psi_r_dq, s->theta_m) *
           s->exp_j_theta_m;
}

/* Stator current in rotor coordinates as a function of the electrical rotor angle (the
 * real part of theta_m), the other states being constant */
static double complex drive_i_s_theta_m(const void *data, double complex theta_m)
{
    const DriveACPortStates *s = data;
    return drive_stator_current(s->drive, s->psi_s_dq, s->psi_r_dq, creal(theta_m));
}

/* Stator current in rotor coordinates as a function of the rotor flux linkage of the
 * induction machine, the other states being constant */
static double complex drive_i_s_psi_r(const void *data, double complex psi_r_dq)
{
    const DriveACPortStates *s = data;
    return drive_stator_current(s->drive, s->psi_s_dq, psi_r_dq, s->theta_m);
}

/* AC port of the machine in stator coordinates at the states s, the electrical rotor
 * speed w_m, and the rate of change d_i_s_dq of the stator current in rotor
 * coordinates due to the other states (_compute_ac_port) */
static void drive_compute_ac_port(const DriveACPortStates *s, double w_m,
                                  double complex d_i_s_dq, ACPort *port)
{
    double complex exp_j_theta_m = s->exp_j_theta_m;
    double complex i_s_dq =
        drive_stator_current(s->drive, s->psi_s_dq, s->psi_r_dq, s->theta_m);
    /* Inverse differential inductance in stator coordinates */
    jacobian(drive_i_s_ab, s, 0.0, AC_PORT_H, port->Y);
    /* Current derivative at a constant stator flux linkage in rotor coordinates, i.e.,
     * at u_s = R_s*i_s + 1j*w_m*psi_s, due to the other states, the spatial harmonics,
     * and the rotation of the coordinates */
    d_i_s_dq += derivative(drive_i_s_theta_m, s, s->theta_m, w_m, AC_PORT_H);
    double complex d_i_s_ab = (d_i_s_dq + I * w_m * i_s_dq) * exp_j_theta_m;
    /* The current derivative is Y (u_s - e), which gives the EMF e */
    double complex u = drive_solve(port->Y, d_i_s_ab);
    double R_s = drive_R_s(s->drive);
    port->i = i_s_dq * exp_j_theta_m;
    port->e = (R_s * i_s_dq + I * w_m * s->psi_s_dq) * exp_j_theta_m - u;
}

/* AC port of the machine for the converter with blocked gates
 * (SynchronousMachine.compute_ac_port or InductionMachine.compute_ac_port) */
static void drive_ac_port(const Model *base, double t, const double complex *x,
                          ACPort *port)
{
    const Drive *self = (const Drive *)base;
    const double complex *x_m = x + dc_bus_states(&base->dc_bus);
    double w_m = drive_n_p(self) * drive_speed(self, t, x_m);
    double theta_m = wrap(creal(x_m[drive_theta_m(self)]));
    DriveACPortStates s = {self, x_m[DRIVE_psi_s], 0.0, theta_m, cexp(I * theta_m)};
    double complex d_i_s_dq = 0.0;
    if (self->induction) {
        /* Current derivative due to the rotor flux linkage, driven by the cage */
        s.psi_r_dq = x_m[DRIVE_psi_r];
        double complex i_s_dq, i_r_dq;
        double tau_M;
        im_magnetic_map(&self->im, s.psi_s_dq, s.psi_r_dq, &i_s_dq, &i_r_dq, &tau_M);
        double complex d_psi_r_dq = -self->im.R_r * i_r_dq;
        d_i_s_dq = derivative(drive_i_s_psi_r, &s, s.psi_r_dq, d_psi_r_dq, AC_PORT_H);
    }
    drive_compute_ac_port(&s, w_m, d_i_s_dq, port);
}

static void drive_init_states(Model *base)
{
    Drive *self = (Drive *)base;
    const char *names;
    /* The states of the machine and of the mechanics after those of the DC bus */
    base->n = dc_bus_states(&base->dc_bus) + drive_theta_M(self) + 1;
    if (self->mechanics.external) {
        names = self->induction ? IM_DRIVE_STATES_EXT : SM_DRIVE_STATES_EXT;
    } else {
        base->n += 1; /* The speed */
        names = self->induction ? IM_DRIVE_STATES : SM_DRIVE_STATES;
    }
    model_state_names(base, names, self->state_names, (int)sizeof(self->state_names));
    base->states = self->state_names;
}

static void drive_init(Drive *self)
{
    Model *base = &self->base;
    base->rhs = drive_rhs;
    base->converter_current = drive_current;
    base->set_states = drive_set_states;
    base->init_states = drive_init_states;
    base->ac_port = drive_ac_port;
    base->converter.enabled = 1;
    base->dc_bus.type = DC_BUS_NONE;
    self->induction = 0;
    self->mechanics.external = 0;
    drive_init_states(base);
    signal_constant(&self->mechanics.B_L, 0.0);
    signal_constant(&self->mechanics.tau_L, 0.0);
    signal_constant(&self->mechanics.w_M, 0.0);
}

static double complex drive_meas_current(const Drive *self, const double complex *x)
{
    double i_abc[3];
    complex2abc(drive_current(&self->base, x), i_abc);
    return abc2complex(i_abc);
}

static double drive_meas_speed(const Drive *self, double t, const double complex *x)
{
    return drive_speed(self, t, x + dc_bus_states(&self->base.dc_bus));
}

static double drive_meas_position(const Drive *self, const double complex *x)
{
    /* The angle of the state, wrapped by set_states, is wrapped again by
     * VectorControlSystem.get_measurement */
    const double complex *x_m = x + dc_bus_states(&self->base.dc_bus);
    return wrap(creal(x_m[drive_theta_M(self)]));
}

/* API for Python (motulator_export.native) --------------------------------- */

/* The systems of the drives embed their simulation as the first member, and its model
 * is a Drive. */

static Drive *drive_of(Simulation *self)
{
    return (Drive *)self->mdl;
}

/* GradNet from its parameters (export_gradnet of motulator_export.plecs._sm) */
void gradnet_set(GradNet *net, int in_dim, int mu_dim, int embed_dim, const double *W,
                 const double *b, const double *mu_log, const double *bias,
                 int activation, double beta_log, int p, double in_base,
                 double out_base)
{
    net->in_dim = in_dim;
    net->mu_dim = mu_dim;
    net->embed_dim = embed_dim;
    for (int j = 0; j < embed_dim; j++) {
        for (int k = 0; k < in_dim; k++) {
            net->W[j][k] = W[in_dim * j + k];
        }
        net->b[j] = b[j];
    }
    for (int k = 0; k < in_dim; k++) {
        net->mu_log[k] = (k < mu_dim) ? mu_log[k] : 0.0;
        net->bias[k] = bias[k];
    }
    net->activation = (GradNetActivation)activation;
    net->beta_log = beta_log;
    net->p = p;
    net->in_base = in_base;
    net->out_base = out_base;
}

/* SynchronousMachine with SynchronousMachinePars */
void set_machine(Simulation *self, double n_p, double R_s, double L_d, double L_q,
                 double psi_f)
{
    Drive *drive = drive_of(self);
    SynchronousMachine *machine = &drive->sm;
    drive->induction = 0;
    machine->n_p = n_p;
    machine->R_s = R_s;
    machine->harmonics = 0;
    machine->L_d = L_d;
    machine->L_q = L_q;
    machine->psi_f = psi_f;
    drive_init_states(&drive->base);
}

/* GradNet current map of the synchronous machine, set by gradnet_set */
GradNet *machine_current_map(Simulation *self)
{
    return &drive_of(self)->sm.current_map;
}

/* SynchronousMachine with SpatialSaturatedSynchronousMachinePars: the current map
 * with spatial harmonics of order k (CurrentMapWithHarmonics), set by
 * machine_current_map */
void set_machine_current_map(Simulation *self, double n_p, double R_s, int k)
{
    Drive *drive = drive_of(self);
    SynchronousMachine *machine = &drive->sm;
    drive->induction = 0;
    machine->n_p = n_p;
    machine->R_s = R_s;
    machine->harmonics = 1;
    machine->k = k;
    drive_init_states(&drive->base);
}

/* InductionMachine with InductionMachineInvGammaPars */
void set_induction_machine_inv_gamma(Simulation *self, double n_p, double R_s,
                                     double R_R, double L_sgm, double L_M)
{
    Drive *drive = drive_of(self);
    InductionMachine *machine = &drive->im;
    drive->induction = 1;
    machine->n_p = n_p;
    machine->R_s = R_s;
    machine->R_r = R_R;
    machine->inv_gamma = 1;
    machine->L_sgm = L_sgm;
    machine->L_M = L_M;
    drive_init_states(&drive->base);
}

/* InductionMachine with InductionMachinePars, whose stator inductance L_s is constant
 * and whose cage is the rotor resistance R_r */
void set_induction_machine(Simulation *self, double n_p, double R_s, double R_r,
                           double L_ell, double L_s)
{
    Drive *drive = drive_of(self);
    InductionMachine *machine = &drive->im;
    drive->induction = 1;
    machine->n_p = n_p;
    machine->R_s = R_s;
    machine->R_r = R_r;
    machine->inv_gamma = 0;
    machine->L_ell = L_ell;
    machine->L_s = L_s;
    drive_init_states(&drive->base);
}

/* MechanicalSystem with the moment of inertia J */
void set_mechanics(Simulation *self, double J)
{
    Drive *drive = drive_of(self);
    drive->mechanics.J = J;
    drive->mechanics.external = 0;
    drive_init_states(&drive->base);
}

/* ExternalRotorSpeed, whose speed is set by signal_steps or signal_function on the
 * signal of external_rotor_speed */
void set_external_rotor_speed(Simulation *self)
{
    Drive *drive = drive_of(self);
    drive->mechanics.external = 1;
    drive_init_states(&drive->base);
}

/* External rotor speed of ExternalRotorSpeed, a function of time */
Signal *external_rotor_speed(Simulation *self)
{
    return &drive_of(self)->mechanics.w_M;
}

/* Friction coefficient of the mechanics as a function of the speed, set by
 * signal_steps or signal_function */
Signal *friction(Simulation *self)
{
    return &drive_of(self)->mechanics.B_L;
}

/* External load torque of the mechanics, set by signal_steps or signal_function */
Signal *load_torque(Simulation *self)
{
    return &drive_of(self)->mechanics.tau_L;
}
