/*
 * Native simulation of synchronous machine drives with flux-vector control: the
 * translation unit of the shared library of motulator_export.native, with its API
 * for Python (ctypes).
 *
 * The system (System) consists of the simulation (sim.h), the drive model (drive.h)
 * with a synchronous machine, and the control system: VectorControlSystem with
 * FluxVectorController and a speed controller (sm_flux_vector.h), with the speed
 * reference. The API functions mirror the constructors of motulator; those of the
 * model are in sim.c and drive.c. The control system is constructed from its
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
#include "native.c"

/* Signals logged at each step (the signal groups of ControlSystem.save), see
 * log_fields_width */
#define FVC_FIELDS                                                                     \
    "ref.T_s ref.tau_M ref.psi_s ref.i_s:c ref.u_s:c ref.u_s_lim:c ref.w_M "           \
    "ref.d_abc:3 ref.enabled:b fbk.u_dc fbk.i_s:c fbk.u_s:c fbk.u_s_zoh:c "            \
    "fbk.psi_s:c fbk.e_o:c fbk.eps fbk.eps_f fbk.psi_a:c fbk.tau_M fbk.tau_L fbk.w_c " \
    "fbk.w_m fbk.w_M fbk.theta_c fbk.theta_m fbk.psi_f fbk.h fbk.caught:b"

/* Vector control system with the speed reference */
typedef struct {
    ControlSystem base;
    VectorControlSystem ctrl;
    Signal w_M_ref;
    double t; /* Controller time */
    Signal enable; /* Gate enable (ControlSystem.set_enable) */
    int enabled;   /* Gate enable of the latest step */
} FluxVectorControlSystem;

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
    double complex u_s_lim =
        native_limited_voltage(&ctrl->pwm, ref->u_c_ab, ref->u_s, fbk->theta_c,
                               ctrl->k_comp, ref->T_s, fbk->w_c, fbk->u_dc);
    log_put_complex(&p, cexp(-I * fbk->theta_c) * u_s_lim);
    log_put(&p, ref->w_M);
    for (int k = 0; k < 3; k++) {
        log_put(&p, ref->d_abc[k]);
    }
    log_put(&p, self->enabled); /* Gate enable */
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
    log_put(&p, fbk->caught);
    return SIM_OK;
}

/* Gate enable at the controller time (ControlSystem.get_enable) */
static int fvc_get_enable(const ControlSystem *base)
{
    const FluxVectorControlSystem *self = (const FluxVectorControlSystem *)base;
    return signal_value(&self->enable, self->t) != 0.0;
}

/* Control step (ControlSystem.run_control_loop with the measurements of
 * VectorControlSystem.get_measurement) */
static int fvc_run(ControlSystem *base, const Model *mdl, double t,
                   const double complex *x, int enabled, double *T_s,
                   double d_abc[3])
{
    FluxVectorControlSystem *self = (FluxVectorControlSystem *)base;
    self->enabled = enabled;
    (void)t;
    const Drive *drive = (const Drive *)mdl;
    VectorControlSystem *ctrl = &self->ctrl;
    Measurements meas;
    meas.i_c_ab = drive_meas_current(drive, x);
    meas.u_dc = model_dc_voltage(mdl, x);
    meas.theta_M = drive_meas_position(drive, x);
    double w_M_ref = signal_value(&self->w_M_ref, self->t);
    if (!isfinite(w_M_ref)) {
        return SIM_ERROR_SIGNAL;
    }
    vector_control_system_compute_output(ctrl, &meas, w_M_ref, enabled);
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
    Drive mdl;
    FluxVectorControlSystem ctrl;
    /* Parameters of the control system, from which it is constructed at the start */
    SynchronousMachinePars est_par;
    FluxVectorControllerCfg cfg;
    PIController speed_ctrl;
    PWM pwm; /* Settings of the PWM */
} System;

/* API for Python (motulator_export.native._sm) ----------------------------- */

System *system_new(void)
{
    System *self = calloc(1, sizeof(System));
    if (self == NULL) {
        return NULL;
    }
    drive_init(&self->mdl);
    modulator_init(&self->mdl.base.modulator, 0, 0.0, 0.0, 1, 0);
    self->sim.mdl = &self->mdl.base;
    self->sim.ctrl = &self->ctrl.base;
    FluxVectorControlSystem *ctrl = &self->ctrl;
    ctrl->base.run = fvc_run;
    ctrl->base.get_enable = fvc_get_enable;
    signal_constant(&ctrl->enable, 1.0);
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

/* FluxVectorControllerCfg, NAN for None, in the order of CFG_FIELDS of
 * plecs/_sm_control.py; the observer gain is k_o0 + k_o1*abs(w_m) */
void set_flux_vector_controller(System *self, double i_s_max, double alpha_tau,
                                double alpha_psi, double alpha_i, double alpha_o,
                                double k_o0, double k_o1, double psi_s_min,
                                double psi_s_max, double k_u, double k_mtpv, double J,
                                int sensorless, double T_s)
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

/* PWM of the control system, see native_pwm_settings */
void set_pwm(System *self, int mme, double t_d, double u_th, double i_0,
             int feedforward, double d_min, int single_update)
{
    native_pwm_settings(&self->pwm, mme, t_d, u_th, i_0, feedforward, d_min,
                        single_update);
}

/* Speed reference, set by signal_steps or signal_function */
Signal *speed_reference(System *self)
{
    return &self->ctrl.w_M_ref;
}

/* Gate enable of the control system (ControlSystem.set_enable), set by signal_steps
 * or signal_function */
Signal *enable_signal(System *self)
{
    return &self->ctrl.enable;
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
    native_pwm_copy_settings(&ctrl->ctrl.pwm, &self->pwm);
    ctrl->t = t_ctrl;

    double complex x[SIM_MAX_STATES];
    for (int i = 0; i < self->mdl.base.n; i++) {
        x[i] = complex_from(x0[2 * i], x0[2 * i + 1]);
    }
    SolverCfg cfg = {max_step, rtol, atol};
    simulation_init(&self->sim, &self->mdl.base, &ctrl->base, &cfg, t0, x, t_start);
}
