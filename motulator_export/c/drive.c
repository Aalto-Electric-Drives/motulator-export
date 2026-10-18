/*
 * Continuous-time model of a synchronous machine drive, ported from motulator. See
 * drive.h.
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

/* Stator current (A) in stator coordinates, at the states set by set_states (the
 * angle is not wrapped again) */
static double complex sm_drive_current(const Model *base, const double complex *x)
{
    const SynchronousMachineDrive *self = (const SynchronousMachineDrive *)base;
    double theta_m = creal(x[SM_DRIVE_theta_m]);
    double complex i_s_dq;
    double tau_M;
    sm_magnetic_map(&self->machine, x[SM_DRIVE_psi_s_dq], theta_m, &i_s_dq, &tau_M);
    return i_s_dq * cexp(I * theta_m);
}

/* State derivatives (SynchronousMachine.rhs and MechanicalSystem.rhs). The angles are
 * wrapped, as Subsystem.set_states does before the outputs are computed. */
static void sm_drive_rhs(const Model *base, double t, const double complex *x,
                         double complex *dx)
{
    const SynchronousMachineDrive *self = (const SynchronousMachineDrive *)base;
    const SynchronousMachine *machine = &self->machine;
    const MechanicalSystem *mechanics = &self->mechanics;
    double complex psi_s_dq = x[SM_DRIVE_psi_s_dq];
    double theta_m = wrap(creal(x[SM_DRIVE_theta_m]));
    double w_M = creal(x[SM_DRIVE_w_M]);

    /* Machine, in rotor coordinates */
    double complex i_s_dq;
    double tau_M;
    sm_magnetic_map(machine, psi_s_dq, theta_m, &i_s_dq, &tau_M);
    double complex u_s_dq = converter_voltage(&base->converter) * cexp(-I * theta_m);
    double w_m = machine->n_p * w_M;
    dx[SM_DRIVE_psi_s_dq] = u_s_dq - machine->R_s * i_s_dq - I * w_m * psi_s_dq;
    dx[SM_DRIVE_theta_m] = w_m;

    /* Mechanics */
    double tau_L_tot = mechanics->B_L * w_M + signal_value(&mechanics->tau_L, t);
    dx[SM_DRIVE_theta_M] = x[SM_DRIVE_w_M];
    dx[SM_DRIVE_w_M] = (tau_M - tau_L_tot) / mechanics->J;
}

/* Keep the angles within [-pi, pi) at the start of an integration interval */
static void sm_drive_set_states(const Model *base, double complex *x)
{
    (void)base;
    x[SM_DRIVE_theta_m] = wrap(creal(x[SM_DRIVE_theta_m]));
    x[SM_DRIVE_theta_M] = wrap(creal(x[SM_DRIVE_theta_M]));
}

static void sm_drive_init(SynchronousMachineDrive *self)
{
    Model *base = &self->base;
    base->n = 4;
    base->states = SM_DRIVE_STATES;
    base->rhs = sm_drive_rhs;
    base->converter_current = sm_drive_current;
    base->set_states = sm_drive_set_states;
    signal_constant(&self->mechanics.tau_L, 0.0);
}

static double complex sm_drive_meas_current(const SynchronousMachineDrive *self,
                                            const double complex *x)
{
    double i_abc[3];
    complex2abc(sm_drive_current(&self->base, x), i_abc);
    return abc2complex(i_abc);
}

static double sm_drive_meas_position(const SynchronousMachineDrive *self,
                                     const double complex *x)
{
    (void)self;
    /* The angle of the state, wrapped by set_states, is wrapped by both
     * MechanicalSystem.meas_position and VectorControlSystem.get_measurement */
    return wrap(wrap(creal(x[SM_DRIVE_theta_M])));
}
