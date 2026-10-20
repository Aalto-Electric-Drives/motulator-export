/*
 * Native simulation of induction machine drives with current-vector control or
 * observer-based V/Hz control: the translation unit of the shared library of
 * motulator_export.native, with its API for Python (ctypes).
 *
 * The system (System) consists of the simulation (sim.h), the drive model (drive.h)
 * with an induction machine, and the control system with the speed reference:
 * VectorControlSystem with CurrentVectorController and a speed controller
 * (im_current_vector.h), or VHzControlSystem with ObserverBasedVHzController
 * (im_flux_vector.h). The API functions mirror the constructors of motulator; those
 * of the model are in sim.c and drive.c. The control system is constructed from its
 * parameters when the simulation starts (start), after which sim_run of sim.c runs
 * the simulation.
 */

#include "common.c"
#include "gradnet.c"
#include "im_current_vector.c"
#include "im_flux_vector.c"
#include "sim.c"
#include "drive.c"
#include "native.c"

/* Signals logged at each step (the signal groups of ControlSystem.save), see
 * log_fields_width. Both control systems have the feedback signals of the flux
 * observer. */
#define IM_FIELDS                                                                      \
    "ref.T_s ref.tau_M ref.psi_s ref.i_s:c ref.u_s:c ref.u_s_lim:c ref.w_M "           \
    "ref.d_abc:3 ref.enabled:b fbk.u_dc fbk.i_s:c fbk.u_s:c fbk.u_s_zoh:c "            \
    "fbk.psi_s:c fbk.psi_R:c fbk.tau_M fbk.tau_L fbk.w_c fbk.w_s fbk.w_r fbk.w_m "     \
    "fbk.w_M fbk.theta_c fbk.e_o:c fbk.eps fbk.h"

/* Vector control system or V/Hz control system with the speed reference */
typedef struct {
    ControlSystem base;
    int vhz; /* VHzControlSystem (else VectorControlSystem) */
    IMVectorControlSystem cvc;
    IMVHzControlSystem vhz_ctrl;
    Signal w_M_ref;
    double t; /* Controller time */
} IMControlSystem;

/* Save the signals of the step (IM_FIELDS) */
static int im_save(IMControlSystem *self)
{
    const IMObserverOutputs *fbk;
    double T_s, tau_M, psi_s, w_M;
    const double *d_abc;
    double complex i_s, u_s, u_s_lim;
    if (self->vhz) {
        const IMVHzControlSystem *ctrl = &self->vhz_ctrl;
        const IMVHzReferences *ref = &ctrl->ref;
        fbk = &ctrl->fbk;
        T_s = ref->T_s;
        tau_M = ref->tau_M;
        psi_s = ref->psi_s;
        i_s = 0.0; /* Current reference, not used */
        u_s = ref->u_s;
        w_M = ref->w_M;
        d_abc = ref->d_abc;
        u_s_lim = native_limited_voltage(&ctrl->pwm, ref->u_c_ab, ref->u_s,
                                         fbk->theta_c, ctrl->k_comp, ref->T_s,
                                         fbk->w_c, fbk->u_dc);
    } else {
        const IMVectorControlSystem *ctrl = &self->cvc;
        const IMReferences *ref = &ctrl->ref;
        fbk = &ctrl->fbk;
        T_s = ref->T_s;
        tau_M = ref->tau_M;
        psi_s = 0.0; /* Flux reference, not used */
        i_s = ref->i_s;
        u_s = ref->u_s;
        w_M = ref->w_M;
        d_abc = ref->d_abc;
        u_s_lim = native_limited_voltage(&ctrl->pwm, ref->u_c_ab, ref->u_s,
                                         fbk->theta_c, ctrl->k_comp, ref->T_s,
                                         fbk->w_c, fbk->u_dc);
    }
    double *p = log_append(&self->base.log);
    if (p == NULL) {
        return SIM_ERROR_MEMORY;
    }
    log_put(&p, self->t);
    log_put(&p, T_s);
    log_put(&p, tau_M);
    log_put(&p, psi_s);
    log_put_complex(&p, i_s);
    log_put_complex(&p, u_s);
    log_put_complex(&p, cexp(-I * fbk->theta_c) * u_s_lim);
    log_put(&p, w_M);
    for (int k = 0; k < 3; k++) {
        log_put(&p, d_abc[k]);
    }
    log_put(&p, 1.0); /* Gate enable */
    log_put(&p, fbk->u_dc);
    log_put_complex(&p, fbk->i_s);
    log_put_complex(&p, fbk->u_s);
    log_put_complex(&p, fbk->u_s_zoh);
    log_put_complex(&p, fbk->psi_s);
    log_put_complex(&p, fbk->psi_R);
    log_put(&p, fbk->tau_M);
    log_put(&p, fbk->tau_L);
    log_put(&p, fbk->w_c);
    log_put(&p, fbk->w_s);
    log_put(&p, fbk->w_r);
    log_put(&p, fbk->w_m);
    log_put(&p, fbk->w_M);
    log_put(&p, fbk->theta_c);
    log_put_complex(&p, fbk->e_o);
    log_put(&p, fbk->eps);
    log_put(&p, fbk->h);
    return SIM_OK;
}

/* Control step (ControlSystem.run_control_loop with the measurements of
 * VectorControlSystem.get_measurement or VHzControlSystem.get_measurement) */
static int im_run(ControlSystem *base, const Model *mdl, double t,
                  const double complex *x, double *T_s, double d_abc[3])
{
    IMControlSystem *self = (IMControlSystem *)base;
    (void)t;
    const Drive *drive = (const Drive *)mdl;
    double w_M_ref = signal_value(&self->w_M_ref, self->t);
    if (!isfinite(w_M_ref)) {
        return SIM_ERROR_SIGNAL;
    }
    double complex i_c_ab = drive_meas_current(drive, x);
    double u_dc = model_dc_voltage(mdl, x);
    if (self->vhz) {
        IMVHzMeasurements meas = {i_c_ab, u_dc, 0.0};
        im_vhz_control_system_compute_output(&self->vhz_ctrl, &meas, w_M_ref, 1);
    } else {
        /* The measured speed is used only in the sensored mode */
        IMMeasurements meas = {i_c_ab, u_dc, drive_meas_speed(drive, x), 0.0};
        im_vector_control_system_compute_output(&self->cvc, &meas, w_M_ref, 1);
    }
    int status = im_save(self);
    if (status != SIM_OK) {
        return status;
    }
    const double *d;
    if (self->vhz) {
        im_vhz_control_system_update(&self->vhz_ctrl);
        *T_s = self->vhz_ctrl.ref.T_s;
        d = self->vhz_ctrl.ref.d_abc;
    } else {
        im_vector_control_system_update(&self->cvc);
        *T_s = self->cvc.ref.T_s;
        d = self->cvc.ref.d_abc;
    }
    self->t = fmod(self->t + *T_s, 1e9); /* Avoid overflow, as ControlSystem.update */
    for (int k = 0; k < 3; k++) {
        d_abc[k] = d[k];
    }
    return SIM_OK;
}

/* System ------------------------------------------------------------------- */

typedef struct {
    Simulation sim; /* First member, see the API of sim.c */
    Drive mdl;
    IMControlSystem ctrl;
    /* Parameters of the control system, from which it is constructed at the start */
    InductionMachineInvGammaPars par;
    IMCurrentVectorControllerCfg cvc_cfg;
    PIController speed_ctrl;
    IMVHzControllerCfg vhz_cfg;
    double slew_rate;
    PWM pwm; /* Settings of the PWM */
} System;

/* API for Python (motulator_export.native._im) ----------------------------- */

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
    IMControlSystem *ctrl = &self->ctrl;
    ctrl->base.run = im_run;
    ctrl->base.fields = IM_FIELDS;
    log_init(&ctrl->base.log, log_fields_width(IM_FIELDS));
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

/* Machine model of the control system: InductionMachineInvGammaPars */
void set_control_machine(System *self, double n_p, double R_s, double R_R,
                         double L_sgm, double L_M)
{
    InductionMachineInvGammaPars par = {n_p, R_s, R_R, L_sgm, L_M};
    self->par = par;
}

/* CurrentVectorControllerCfg, NAN for None, in VectorControlSystem */
void set_current_vector_controller(System *self, double psi_s_nom, double i_s_max,
                                   double alpha_c, double alpha_i, double alpha_o,
                                   double w_s_nom, double k_u, double k_fw, double J,
                                   int sensorless, double T_s)
{
    IMCurrentVectorControllerCfg *cfg = &self->cvc_cfg;
    *cfg = im_current_vector_controller_cfg(psi_s_nom, i_s_max);
    cfg->alpha_c = alpha_c;
    cfg->alpha_i = alpha_i;
    cfg->alpha_o = alpha_o;
    cfg->w_s_nom = w_s_nom;
    cfg->k_u = k_u;
    cfg->k_fw = k_fw;
    cfg->J = J;
    cfg->T_s = T_s;
    cfg->sensorless = sensorless;
    self->ctrl.vhz = 0;
}

/* Speed controller (PIController, e.g., SpeedController) by its gains */
void set_speed_controller(System *self, double k_p, double k_t, double alpha_i,
                          double u_max)
{
    pi_init(&self->speed_ctrl, k_p, k_t, alpha_i, u_max);
}

/* ObserverBasedVHzControllerCfg in VHzControlSystem with the slew rate (mechanical
 * rad/s^2) of the speed reference */
void set_vhz_controller(System *self, double psi_s_nom, double i_s_max,
                        double alpha_psi, double alpha_tau, double alpha_f, double k_u,
                        double k_b, double T_s, double slew_rate)
{
    IMVHzControllerCfg *cfg = &self->vhz_cfg;
    *cfg = im_vhz_controller_cfg(psi_s_nom, i_s_max);
    cfg->alpha_psi = alpha_psi;
    cfg->alpha_tau = alpha_tau;
    cfg->alpha_f = alpha_f;
    cfg->k_u = k_u;
    cfg->k_b = k_b;
    cfg->T_s = T_s;
    self->slew_rate = slew_rate;
    self->ctrl.vhz = 1;
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

/* Construct the control system and start the simulation at the time t0 with the
 * states x0 (real and imaginary parts) of the model, the first sampling instant
 * t_start, and the controller time t_ctrl */
void start(System *self, double t0, const double *x0, double t_start, double t_ctrl,
           double max_step, double rtol, double atol)
{
    IMControlSystem *ctrl = &self->ctrl;
    if (ctrl->vhz) {
        im_vhz_control_system_init(&ctrl->vhz_ctrl, self->par, &self->vhz_cfg,
                                   self->slew_rate);
        native_pwm_copy_settings(&ctrl->vhz_ctrl.pwm, &self->pwm);
    } else {
        im_vector_control_system_init(&ctrl->cvc, self->par, &self->cvc_cfg,
                                      self->speed_ctrl);
        native_pwm_copy_settings(&ctrl->cvc.pwm, &self->pwm);
    }
    ctrl->t = t_ctrl;

    double complex x[SIM_MAX_STATES];
    for (int i = 0; i < self->mdl.base.n; i++) {
        x[i] = complex_from(x0[2 * i], x0[2 * i + 1]);
    }
    SolverCfg cfg = {max_step, rtol, atol};
    simulation_init(&self->sim, &self->mdl.base, &ctrl->base, &cfg, t0, x, t_start);
}
