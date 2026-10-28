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

/* Stator current (A) in stator coordinates and the torque (Nm) of the machine, at the
 * states x[0] and x[1] of the machine, theta_m being the wrapped angle of a
 * synchronous machine (SynchronousMachine.compute_outputs or
 * InductionMachine.compute_outputs). The rotor current of the induction machine is
 * given in i_r_ab (if not NULL). */
static double complex drive_machine_outputs(const Drive *self, const double complex *x,
                                            double theta_m, double *tau_M,
                                            double complex *i_r_ab)
{
    if (self->induction) {
        const InductionMachine *im = &self->im;
        double complex psi_s_ab = x[DRIVE_psi_s];
        double complex i_r = (x[DRIVE_psi_r] - psi_s_ab) / im->L_ell;
        /* Current into the magnetic circuit, equal to the stator current without the
         * core losses */
        double complex i_m_ab = psi_s_ab / im->L_s - i_r;
        *tau_M = 1.5 * im->n_p * cimag(i_m_ab * conj(psi_s_ab));
        if (i_r_ab != NULL) {
            *i_r_ab = i_r;
        }
        return i_m_ab;
    }
    double complex i_s_dq;
    sm_magnetic_map(&self->sm, x[DRIVE_psi_s], theta_m, &i_s_dq, tau_M);
    return i_s_dq * cexp(I * theta_m);
}

/* Stator current (A) in stator coordinates, at the states set by set_states (the
 * angle is not wrapped again) */
static double complex drive_current(const Model *base, const double complex *x)
{
    const Drive *self = (const Drive *)base;
    const double complex *x_m = x + dc_bus_states(&base->dc_bus);
    double tau_M;
    return drive_machine_outputs(self, x_m, creal(x_m[DRIVE_theta_m]), &tau_M, NULL);
}

/* Mechanical rotor speed at the time t: the state of MechanicalSystem, or the
 * external speed of ExternalRotorSpeed (ExternalRotorSpeed.set_outputs) */
static double drive_speed(const Drive *self, double t, const double complex *x_m)
{
    if (self->mechanics.external) {
        return signal_value(&self->mechanics.w_M, t);
    }
    return creal(x_m[DRIVE_w_M]);
}

/* State derivatives (DCBus.rhs, SynchronousMachine.rhs or InductionMachine.rhs, and
 * MechanicalSystem.rhs or ExternalRotorSpeed.rhs). The angle of the synchronous
 * machine is wrapped, as Subsystem.set_states does before the outputs are computed.
 */
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

    /* Machine */
    double tau_M;
    double complex i_s_ab;
    if (self->induction) {
        const InductionMachine *im = &self->im;
        double complex i_r_ab;
        i_s_ab = drive_machine_outputs(self, x_m, 0.0, &tau_M, &i_r_ab);
        dx_m[DRIVE_psi_s] = u_s_ab - im->R_s * i_s_ab;
        dx_m[DRIVE_psi_r] =
            -im->R_r * i_r_ab + I * im->n_p * w_M * x_m[DRIVE_psi_r];
    } else {
        /* In rotor coordinates */
        const SynchronousMachine *sm = &self->sm;
        double theta_m = wrap(creal(x_m[DRIVE_theta_m]));
        double complex psi_s_dq = x_m[DRIVE_psi_s];
        double complex i_s_dq;
        sm_magnetic_map(sm, psi_s_dq, theta_m, &i_s_dq, &tau_M);
        i_s_ab = i_s_dq * cexp(I * theta_m);
        double complex u_s_dq = u_s_ab * cexp(-I * theta_m);
        double w_m = sm->n_p * w_M;
        dx_m[DRIVE_psi_s] = u_s_dq - sm->R_s * i_s_dq - I * w_m * psi_s_dq;
        dx_m[DRIVE_theta_m] = w_m;
    }

    /* Mechanics */
    if (mechanics->external) {
        dx_m[DRIVE_theta_M] = w_M;
    } else {
        double tau_L_tot = signal_value(&mechanics->B_L, w_M) * w_M +
                           signal_value(&mechanics->tau_L, t);
        dx_m[DRIVE_theta_M] = x_m[DRIVE_w_M];
        dx_m[DRIVE_w_M] = (tau_M - tau_L_tot) / mechanics->J;
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
    if (!self->induction) {
        x_m[DRIVE_theta_m] = wrap(creal(x_m[DRIVE_theta_m]));
    }
    x_m[DRIVE_theta_M] = wrap(creal(x_m[DRIVE_theta_M]));
}

/* Step of the central differences of the Jacobians (jacobian of _ac_port.py) */
#define DRIVE_JACOBIAN_STEP 1e-4

/* Jacobian of the stator current with respect to the stator flux linkage, acting on
 * the real and imaginary parts (jacobian of _ac_port.py): of the synchronous machine
 * in rotor coordinates at the angle theta_m, or of the induction machine in stator
 * coordinates at the rotor flux linkage psi_r_ab */
static void drive_current_jacobian(const Drive *self, double complex psi_s,
                                   double theta_m, double complex psi_r_ab,
                                   double J[2][2])
{
    const double h = DRIVE_JACOBIAN_STEP;
    double complex points[4] = {psi_s + h, psi_s - h, psi_s + I * h, psi_s - I * h};
    double complex i[4];
    for (int k = 0; k < 4; k++) {
        if (self->induction) {
            const InductionMachine *im = &self->im;
            i[k] = points[k] / im->L_s + (points[k] - psi_r_ab) / im->L_ell;
        } else {
            double tau_M;
            sm_magnetic_map(&self->sm, points[k], theta_m, &i[k], &tau_M);
        }
    }
    double complex d_x = (i[0] - i[1]) / (2.0 * h);
    double complex d_y = (i[2] - i[3]) / (2.0 * h);
    J[0][0] = creal(d_x);
    J[0][1] = creal(d_y);
    J[1][0] = cimag(d_x);
    J[1][1] = cimag(d_y);
}

/* Solution of the 2-by-2 system J v = z, the vectors as complex numbers */
static double complex drive_solve(const double J[2][2], double complex z)
{
    double M[3][3] = {{J[0][0], J[0][1], 0.0}, {J[1][0], J[1][1], 0.0}, {0.0}};
    double b[3] = {creal(z), cimag(z), 0.0};
    solve_small(2, M, b);
    return complex_from(b[0], b[1]);
}

/* AC port of the machine for the converter with blocked gates
 * (SynchronousMachine.compute_ac_port or InductionMachine.compute_ac_port) */
static void drive_ac_port(const Model *base, double t, const double complex *x,
                          ACPort *port)
{
    const Drive *self = (const Drive *)base;
    const double complex *x_m = x + dc_bus_states(&base->dc_bus);
    double w_M = drive_speed(self, t, x_m);
    if (self->induction) {
        const InductionMachine *im = &self->im;
        double tau_M;
        double complex i_r_ab;
        double complex i_s_ab = drive_machine_outputs(self, x_m, 0.0, &tau_M, &i_r_ab);
        double complex psi_r_ab = x_m[DRIVE_psi_r];
        /* The stator current derivative is Y*(u_s - R_s*i_s) - d_psi_r/dt/L_ell */
        drive_current_jacobian(self, x_m[DRIVE_psi_s], 0.0, psi_r_ab, port->Y);
        double complex d_psi_r_ab = -im->R_r * i_r_ab + I * im->n_p * w_M * psi_r_ab;
        double complex e = drive_solve(port->Y, d_psi_r_ab) / im->L_ell;
        port->i = i_s_ab;
        port->e = im->R_s * i_s_ab + e;
        return;
    }
    const SynchronousMachine *sm = &self->sm;
    const double h = DRIVE_JACOBIAN_STEP;
    double theta_m = wrap(creal(x_m[DRIVE_theta_m]));
    double complex exp_j_theta_m = cexp(I * theta_m);
    double w_m = sm->n_p * w_M;
    double complex psi_s_dq = x_m[DRIVE_psi_s];
    double complex i_s_dq, i_plus, i_minus;
    double tau_M;
    sm_magnetic_map(sm, psi_s_dq, theta_m, &i_s_dq, &tau_M);
    /* Inverse differential inductance in rotor coordinates and the current derivative
     * due to the spatial harmonics at a constant flux */
    double J[2][2];
    drive_current_jacobian(self, psi_s_dq, theta_m, 0.0, J);
    sm_magnetic_map(sm, psi_s_dq, theta_m + h, &i_plus, &tau_M);
    sm_magnetic_map(sm, psi_s_dq, theta_m - h, &i_minus, &tau_M);
    double complex d_i_s_theta = (i_plus - i_minus) / (2.0 * h);
    /* The stator current derivative in stator coordinates is J*(u_s - R_s*i_s -
     * 1j*w_m*psi_s) + w_m*(d_i_s_theta + 1j*i_s), in rotor coordinates */
    double complex i_rot = w_m * (d_i_s_theta + I * i_s_dq);
    double complex v = drive_solve(J, i_rot);
    double complex e_dq = sm->R_s * i_s_dq + I * w_m * psi_s_dq - v;
    double c = creal(exp_j_theta_m), s = cimag(exp_j_theta_m);
    double R[2][2] = {{c, -s}, {s, c}}, RJ[2][2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            RJ[i][j] = R[i][0] * J[0][j] + R[i][1] * J[1][j];
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            port->Y[i][j] = RJ[i][0] * R[j][0] + RJ[i][1] * R[j][1];
        }
    }
    port->i = i_s_dq * exp_j_theta_m;
    port->e = e_dq * exp_j_theta_m;
}

static void drive_init_states(Model *base)
{
    Drive *self = (Drive *)base;
    const char *names;
    if (self->mechanics.external) {
        base->n = dc_bus_states(&base->dc_bus) + 3;
        names = self->induction ? IM_DRIVE_STATES_EXT : SM_DRIVE_STATES_EXT;
    } else {
        base->n = dc_bus_states(&base->dc_bus) + 4;
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
    /* The angle of the state, wrapped by set_states, is wrapped by both
     * MechanicalSystem.meas_position and VectorControlSystem.get_measurement */
    double theta_M = creal(x[dc_bus_states(&self->base.dc_bus) + DRIVE_theta_M]);
    return wrap(wrap(theta_M));
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

/* InductionMachine with the parameters of the Γ model (InductionMachinePars) */
void set_induction_machine(Simulation *self, double n_p, double R_s, double R_r,
                           double L_ell, double L_s)
{
    Drive *drive = drive_of(self);
    InductionMachine *machine = &drive->im;
    drive->induction = 1;
    machine->n_p = n_p;
    machine->R_s = R_s;
    machine->R_r = R_r;
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
