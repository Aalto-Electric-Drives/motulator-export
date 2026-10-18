/*
 * Native simulation of synchronous machine drives with flux-vector control: the
 * translation unit of the shared library of motulator_export.native, with its API
 * for Python (ctypes).
 *
 * The system (System) consists of the simulation (sim.h), the drive model (drive.h),
 * and the control system: VectorControlSystem with FluxVectorController and a speed
 * controller (sm_flux_vector.h), with the speed reference. The API functions mirror
 * the constructors of motulator. The control system is constructed from its
 * parameters when the simulation starts (start), after which sim_run of sim.c runs
 * the simulation.
 */

#include "common.c"
#include "gradnet.c"
#include "sm_parameters.c"
#include "sm_control_loci.c"
#include "sm_flux_vector.c"
#include "sim.c"
#include "drive.c"

/* Signals logged at each step (the signal groups of ControlSystem.save), see
 * log_fields_width */
#define FVC_FIELDS                                                                     \
    "ref.T_s ref.tau_M ref.psi_s ref.i_s:c ref.u_s:c ref.u_s_lim:c ref.w_M "           \
    "ref.d_abc:3 ref.enabled:b fbk.u_dc fbk.i_s:c fbk.u_s:c fbk.u_s_zoh:c "            \
    "fbk.psi_s:c fbk.e_o:c fbk.eps fbk.eps_f fbk.psi_a:c fbk.tau_M fbk.tau_L fbk.w_c " \
    "fbk.w_m fbk.w_M fbk.theta_c fbk.theta_m fbk.psi_f fbk.h"

/* Vector control system with the speed reference */
typedef struct {
    ControlSystem base;
    VectorControlSystem ctrl;
    Signal w_M_ref;
    double t; /* Controller time */
} FluxVectorControlSystem;

/* Limited voltage reference (V) in stationary coordinates, excluding the compensation
 * of the duty-ratio error (PWM.limited_voltage). With the compensation, the minimum
 * pulses are not supported. */
static double complex limited_voltage(const VectorControlSystem *ctrl)
{
    const PWM *pwm = &ctrl->pwm;
    double u_dc = ctrl->fbk.u_dc;
    if (!(pwm_has_error_model(pwm) && pwm->feedforward) || u_dc <= 0.0) {
        return ctrl->ref.u_c_ab;
    }
    /* Duty ratios of the voltage reference (vector_control_system_compute_output) */
    double complex u_s_ab_ref = cexp(I * ctrl->fbk.theta_c) * ctrl->ref.u_s;
    u_s_ab_ref *= cexp(I * ctrl->k_comp * ctrl->ref.T_s * ctrl->fbk.w_c);
    double d_ref[3];
    pwm_duty_ratios(u_s_ab_ref, u_dc, pwm->mme, d_ref);
    return abc2complex(d_ref) * u_dc;
}

/* Save the signals of the step (FVC_FIELDS) */
static int fvc_save(FluxVectorControlSystem *self)
{
    const VectorControlSystem *ctrl = &self->ctrl;
    const References *ref = &ctrl->ref;
    const ObserverOutputs *fbk = &ctrl->fbk;
    double *p = log_append(&self->base.log);
    if (p == NULL) {
        return SIM_ERROR_MEMORY;
    }
    log_put(&p, self->t);
    log_put(&p, ref->T_s);
    log_put(&p, ref->tau_M);
    log_put(&p, ref->psi_s);
    log_put_complex(&p, 0.0); /* Current reference, not used */
    log_put_complex(&p, ref->u_s);
    log_put_complex(&p, cexp(-I * fbk->theta_c) * limited_voltage(ctrl));
    log_put(&p, ref->w_M);
    for (int k = 0; k < 3; k++) {
        log_put(&p, ref->d_abc[k]);
    }
    log_put(&p, 1.0); /* Gate enable */
    log_put(&p, fbk->u_dc);
    log_put_complex(&p, fbk->i_s);
    log_put_complex(&p, fbk->u_s);
    log_put_complex(&p, fbk->u_s_zoh);
    log_put_complex(&p, fbk->psi_s);
    log_put_complex(&p, fbk->e_o);
    log_put(&p, fbk->eps);
    log_put(&p, fbk->eps_f);
    log_put_complex(&p, fbk->psi_a);
    log_put(&p, fbk->tau_M);
    log_put(&p, fbk->tau_L);
    log_put(&p, fbk->w_c);
    log_put(&p, fbk->w_m);
    log_put(&p, fbk->w_M);
    log_put(&p, fbk->theta_c);
    log_put(&p, fbk->theta_m);
    log_put(&p, fbk->psi_f);
    log_put(&p, fbk->h);
    return SIM_OK;
}

/* Control step (ControlSystem.run_control_loop with the measurements of
 * VectorControlSystem.get_measurement) */
static int fvc_run(ControlSystem *base, const Model *mdl, const double complex *x,
                   double *T_s, double d_abc[3])
{
    FluxVectorControlSystem *self = (FluxVectorControlSystem *)base;
    const SynchronousMachineDrive *drive = (const SynchronousMachineDrive *)mdl;
    VectorControlSystem *ctrl = &self->ctrl;
    Measurements meas;
    meas.i_c_ab = sm_drive_meas_current(drive, x);
    meas.u_dc = mdl->converter.u_dc;
    meas.theta_M = sm_drive_meas_position(drive, x);
    meas.u_c_ab = 0.0;
    double w_M_ref = signal_value(&self->w_M_ref, self->t);
    if (!isfinite(w_M_ref)) {
        return SIM_ERROR_SIGNAL;
    }
    vector_control_system_compute_output(ctrl, &meas, w_M_ref, 1);
    int status = fvc_save(self);
    if (status != SIM_OK) {
        return status;
    }
    vector_control_system_update(ctrl);
    *T_s = ctrl->ref.T_s;
    self->t = fmod(self->t + *T_s, 1e9); /* Avoid overflow, as ControlSystem.update */
    for (int k = 0; k < 3; k++) {
        d_abc[k] = ctrl->ref.d_abc[k];
    }
    return SIM_OK;
}

/* System ------------------------------------------------------------------- */

typedef struct {
    Simulation sim; /* First member, see the API of sim.c */
    SynchronousMachineDrive mdl;
    FluxVectorControlSystem ctrl;
    /* Parameters of the control system, from which it is constructed at the start */
    SynchronousMachinePars est_par;
    FluxVectorControllerCfg cfg;
    PIController speed_ctrl;
    PWM pwm; /* Settings of the PWM */
} System;

/* API for Python (motulator_export.native.sm) ------------------------------ */

System *system_new(void)
{
    System *self = calloc(1, sizeof(System));
    if (self == NULL) {
        return NULL;
    }
    sm_drive_init(&self->mdl);
    modulator_init(&self->mdl.base.modulator, 0, 0.0, 0.0, 1, 0);
    self->sim.mdl = &self->mdl.base;
    self->sim.ctrl = &self->ctrl.base;
    FluxVectorControlSystem *ctrl = &self->ctrl;
    ctrl->base.run = fvc_run;
    ctrl->base.fields = FVC_FIELDS;
    log_init(&ctrl->base.log, log_fields_width(FVC_FIELDS));
    signal_constant(&ctrl->w_M_ref, 0.0);
    pwm_init(&self->pwm, 1.5);
    return self;
}

void system_free(System *self)
{
    if (self != NULL) {
        simulation_free(&self->sim);
        log_free(&self->ctrl.base.log);
        free(self);
    }
}

/* GradNet from its parameters (export_gradnet of motulator_export.plecs.sm) */
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

/* System model: SynchronousMachine with SynchronousMachinePars */
void set_machine(System *self, double n_p, double R_s, double L_d, double L_q,
                 double psi_f)
{
    SynchronousMachine *machine = &self->mdl.machine;
    machine->n_p = n_p;
    machine->R_s = R_s;
    machine->harmonics = 0;
    machine->L_d = L_d;
    machine->L_q = L_q;
    machine->psi_f = psi_f;
}

/* GradNet current map of the machine, set by gradnet_set */
GradNet *machine_current_map(System *self)
{
    return &self->mdl.machine.current_map;
}

/* SynchronousMachine with SpatialSaturatedSynchronousMachinePars: the current map
 * with spatial harmonics of order k (CurrentMapWithHarmonics), set by
 * machine_current_map */
void set_machine_current_map(System *self, double n_p, double R_s, int k)
{
    SynchronousMachine *machine = &self->mdl.machine;
    machine->n_p = n_p;
    machine->R_s = R_s;
    machine->harmonics = 1;
    machine->k = k;
}

/* MechanicalSystem with a constant friction coefficient */
void set_mechanics(System *self, double J, double B_L)
{
    self->mdl.mechanics.J = J;
    self->mdl.mechanics.B_L = B_L;
}

/* External load torque of the mechanics, set by signal_steps or signal_function */
Signal *load_torque(System *self)
{
    return &self->mdl.mechanics.tau_L;
}

/* VoltageSourceConverter with a constant DC-bus voltage */
void set_converter(System *self, double u_dc, double u_th)
{
    self->mdl.base.converter.u_dc = u_dc;
    self->mdl.base.converter.u_th = u_th;
}

/* PWM model (CarrierComparison if carrier is nonzero, else ZOH) and the delay of the
 * model, see modulator_init */
void set_modulator(System *self, int carrier, double N, double t_d, int delay,
                   int single_update)
{
    modulator_init(&self->mdl.base.modulator, carrier, N, t_d, delay, single_update);
}

/* Machine model of the control system: SynchronousMachinePars */
void set_control_machine(System *self, double n_p, double R_s, double L_d, double L_q,
                         double psi_f)
{
    self->est_par = synchronous_machine_pars(n_p, R_s, L_d, L_q, psi_f);
}

/* GradNet flux map of the control system, set by gradnet_set */
GradNet *control_flux_map(System *self)
{
    return &self->est_par.flux_map;
}

/* Machine model of the control system: SaturatedSynchronousMachinePars with the flux
 * map (control_flux_map) */
void set_control_machine_flux_map(System *self, double n_p, double R_s)
{
    GradNet flux_map = self->est_par.flux_map;
    self->est_par = saturated_synchronous_machine_pars(n_p, R_s, &flux_map);
}

/* FluxVectorControllerCfg, NAN for None; the observer gain is k_o0 + k_o1*abs(w_m) */
void set_flux_vector_controller(System *self, double i_s_max, double alpha_tau,
                                double alpha_psi, double alpha_i, double alpha_o,
                                double k_o0, double k_o1, double psi_s_min,
                                double psi_s_max, double k_u, double k_mtpv, double J,
                                double T_s, int sensorless)
{
    FluxVectorControllerCfg *cfg = &self->cfg;
    *cfg = flux_vector_controller_cfg(i_s_max);
    cfg->alpha_tau = alpha_tau;
    cfg->alpha_psi = alpha_psi;
    cfg->alpha_i = alpha_i;
    cfg->alpha_o = alpha_o;
    cfg->k_o[0] = k_o0;
    cfg->k_o[1] = k_o1;
    cfg->psi_s_min = psi_s_min;
    cfg->psi_s_max = psi_s_max;
    cfg->k_u = k_u;
    cfg->k_mtpv = k_mtpv;
    cfg->J = J;
    cfg->T_s = T_s;
    cfg->sensorless = sensorless;
}

/* Speed controller (PIController, e.g., SpeedController) by its gains */
void set_speed_controller(System *self, double k_p, double k_t, double alpha_i,
                          double u_max)
{
    pi_init(&self->speed_ctrl, k_p, k_t, alpha_i, u_max);
}

/* PWM of the control system: the overmodulation (MME if mme is nonzero, else MPE),
 * the duty-ratio error model InverterNonlinearity(t_d, u_th, sign) with sign =
 * tanh(i/i_0), or np.sign if i_0 = 0, with its feedforward compensation, the minimum
 * duty ratio, and the single update */
void set_pwm(System *self, int mme, double t_d, double u_th, double i_0,
             int feedforward, double d_min, int single_update)
{
    pwm_set_overmodulation(&self->pwm, mme);
    pwm_set_nonlinearity(&self->pwm, t_d, u_th, i_0, feedforward);
    pwm_set_min_pulse(&self->pwm, d_min);
    pwm_set_single_update(&self->pwm, single_update);
}

/* Speed reference, set by signal_steps or signal_function */
Signal *speed_reference(System *self)
{
    return &self->ctrl.w_M_ref;
}

/* Construct the control system and start the simulation at the time t0 with the
 * states x0 (real and imaginary parts) of the model, the first sampling instant
 * t_start, and the controller time t_ctrl */
void start(System *self, double t0, const double *x0, double t_start, double t_ctrl,
           double max_step, double rtol, double atol)
{
    FluxVectorControlSystem *ctrl = &self->ctrl;
    vector_control_system_init(&ctrl->ctrl, self->est_par, &self->cfg,
                               self->speed_ctrl);
    PWM *pwm = &ctrl->ctrl.pwm;
    pwm_set_overmodulation(pwm, self->pwm.mme);
    pwm_set_nonlinearity(pwm, self->pwm.t_d, self->pwm.u_th, self->pwm.i_0,
                         self->pwm.feedforward);
    pwm_set_min_pulse(pwm, self->pwm.d_min);
    pwm_set_single_update(pwm, self->pwm.single_update);
    ctrl->t = t_ctrl;

    double complex x[SIM_MAX_STATES];
    for (int i = 0; i < self->mdl.base.n; i++) {
        x[i] = complex_from(x0[2 * i], x0[2 * i + 1]);
    }
    SolverCfg cfg = {max_step, rtol, atol};
    simulation_init(&self->sim, &self->mdl.base, &ctrl->base, &cfg, t0, x, t_start);
}
