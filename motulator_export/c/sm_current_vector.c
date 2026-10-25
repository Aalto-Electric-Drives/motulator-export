/*
 * Current-vector control of synchronous machine drives, see sm_current_vector.h.
 */

#include "sm_current_vector.h"

/* Configuration ------------------------------------------------------------ */

static SMCurrentVectorControllerCfg sm_current_vector_controller_cfg(double i_s_max)
{
    SMCurrentVectorControllerCfg cfg;
    cfg.i_s_max = i_s_max;
    cfg.alpha_c = 2.0 * M_PI * 200.0;
    cfg.alpha_i = NAN;
    cfg.alpha_o = NAN;
    cfg.alpha_ref = 2.0 * M_PI * 100.0;
    cfg.k_o[0] = NAN;
    cfg.k_o[1] = NAN;
    cfg.psi_s_min = NAN;
    cfg.psi_s_max = INFINITY;
    cfg.k_u = 0.9;
    cfg.k_mtpv = 0.9;
    cfg.J = NAN;
    cfg.T_s = 125e-6;
    cfg.sensorless = 1;
    cfg.discrete = 0;
    return cfg;
}

/* CurrentController and DiscreteCurrentController --------------------------- */

static void sm_current_ctrl_init(SMCurrentController *self, SynchronousMachinePars par,
                                 double alpha_c, double alpha_i, int discrete,
                                 double T_s)
{
    self->par = par;
    self->discrete = discrete;
    alpha_i = isnan(alpha_i) ? alpha_c : alpha_i;
    double k_t = alpha_c;
    double k_i = alpha_c * alpha_i;
    double k_p = alpha_c + alpha_i;
    complex_pi_init(&self->pi, k_p, k_i, k_t);
    discrete_complex_init(&self->dc, alpha_c, T_s);
}

/* The currents mapped to the flux linkages, which takes the saliency and the
 * magnetic saturation into account */
static double complex sm_current_ctrl_compute_output(SMCurrentController *self,
                                                     double complex i_ref,
                                                     double complex i)
{
    double complex psi_ref = psi_s_dq(&self->par, i_ref) - self->par.psi_f;
    double complex psi = psi_s_dq(&self->par, i) - self->par.psi_f;
    if (self->discrete) {
        return discrete_complex_compute_output(&self->dc, psi_ref, psi);
    }
    return complex_pi_compute_output(&self->pi, psi_ref, psi, 0.0);
}

static void sm_current_ctrl_update(SMCurrentController *self, double T_s,
                                   double complex u, double w_c)
{
    if (self->discrete) {
        discrete_complex_update(&self->dc, T_s, u, w_c);
    } else {
        complex_pi_update(&self->pi, T_s, u, w_c);
    }
}

/* Preset the integral state to the steady-state output u (preset) */
static void sm_current_ctrl_preset(SMCurrentController *self, double complex u,
                                   double w_c)
{
    if (self->discrete) {
        discrete_complex_preset(&self->dc, u, w_c);
    } else {
        self->pi.u_i = u;
    }
}

/* CurrentVectorController ---------------------------------------------------- */

static void sm_current_vector_ctrl_init(SMCurrentVectorController *self,
                                        SynchronousMachinePars par,
                                        const SMCurrentVectorControllerCfg *cfg)
{
    self->T_s = cfg->T_s;
    self->sensorless = cfg->sensorless;
    reference_gen_init(&self->reference_gen, &par, cfg->i_s_max, cfg->psi_s_min,
                       cfg->psi_s_max, cfg->k_u, cfg->k_mtpv, NULL);
    self->reference_gen.alpha = cfg->alpha_ref;
    sm_current_ctrl_init(&self->current_ctrl, par, cfg->alpha_c, cfg->alpha_i,
                         cfg->discrete, cfg->T_s);
    create_speed_flux_observer(&self->observer, par, cfg->alpha_o, cfg->T_s, cfg->k_o,
                               cfg->sensorless, cfg->J);
}

/* Restart from the zero-torque operating point of the caught speed (get_feedback):
 * in standby, the states tracked the speed estimate held by the observer (zero
 * current, since the catch requires magnets) */
static void sm_current_vector_ctrl_caught(SMCurrentVectorController *self,
                                          const ObserverOutputs *fbk)
{
    if (fbk->caught) {
        self->reference_gen.i_s_ref = 0.0;
        sm_current_ctrl_preset(&self->current_ctrl, I * fbk->w_m * fbk->psi_f,
                               fbk->w_c);
    }
}

static void sm_current_vector_ctrl_compute_output(SMCurrentVectorController *self,
                                                  double tau_M_ref,
                                                  const ObserverOutputs *fbk,
                                                  References *ref)
{
    ref->T_s = self->T_s;
    reference_gen_compute_flux_and_torque_refs(&self->reference_gen, tau_M_ref,
                                               fbk->w_m, fbk->u_dc, &ref->psi_s,
                                               &ref->tau_M);
    ref->i_s = reference_gen_compute_current_ref(&self->reference_gen, ref->psi_s,
                                                 ref->tau_M);
    ref->u_s = sm_current_ctrl_compute_output(&self->current_ctrl, ref->i_s, fbk->i_s);
}

static void sm_current_vector_ctrl_update(SMCurrentVectorController *self,
                                          const References *ref,
                                          const ObserverOutputs *fbk)
{
    speed_flux_observer_update(&self->observer, ref->T_s, fbk);
    /* The discrete-time controller takes its limited output, except with blocked
     * gates, when the open-circuit voltage of the observer is the realized one */
    double complex u_s =
        (self->current_ctrl.discrete && ref->enabled) ? ref->u_s_lim : fbk->u_s;
    sm_current_ctrl_update(&self->current_ctrl, ref->T_s, u_s, fbk->w_c);
    reference_gen_update(&self->reference_gen, ref->T_s);
}

/* VectorControlSystem -------------------------------------------------------- */

static void sm_cvc_control_system_init(SMCVCControlSystem *self,
                                       SynchronousMachinePars par,
                                       const SMCurrentVectorControllerCfg *cfg,
                                       PIController speed_ctrl)
{
    pwm_init(&self->pwm, 1.5);
    /* k_comp of CurrentVectorController: the discrete-time current controller
     * compensates for the delays itself */
    self->k_comp = cfg->discrete ? 0.0 : 1.5;
    sm_current_vector_ctrl_init(&self->vector_ctrl, par, cfg);
    self->speed_ctrl = speed_ctrl;
    self->speed_ctrl.u_i = 0.0;
    self->speed_ctrl.v = 0.0;
    self->enabled = 1;
    self->release_step = 0;
    self->torque_mode = 0;
}

static void sm_cvc_control_system_compute_output(SMCVCControlSystem *self,
                                                 const Measurements *meas,
                                                 double ext_ref, int enabled)
{
    ObserverOutputs *fbk = &self->fbk;
    References *ref = &self->ref;
    self->enabled = enabled;
    int standby = !enabled || self->release_step;
    int sensorless = self->vector_ctrl.sensorless;
    vector_control_feedback(&self->vector_ctrl.observer, sensorless, &self->pwm, meas,
                            standby, fbk);
    sm_current_vector_ctrl_caught(&self->vector_ctrl, fbk);

    /* Speed controller (or in torque-control mode, the torque reference) and
     * vector controller, or in standby, the vector controller with zero torque
     * reference and the zero vector of a sensorless drive */
    double tau_M_ref = 0.0;
    if (!standby && self->torque_mode) {
        tau_M_ref = ext_ref;
    } else if (!standby) {
        tau_M_ref = pi_compute_output(&self->speed_ctrl, ext_ref, fbk->w_M, 0.0);
    }
    sm_current_vector_ctrl_compute_output(&self->vector_ctrl, tau_M_ref, fbk, ref);
    vector_control_modulate(&self->pwm, self->k_comp, sensorless && standby, meas, fbk,
                            ref);
    ref->w_M = self->torque_mode ? NAN : ext_ref; /* None in torque mode */
    ref->enabled = enabled;
}

static void sm_cvc_control_system_update(SMCVCControlSystem *self)
{
    pwm_update(&self->pwm, self->ref.T_s, self->ref.d_abc);
    sm_current_vector_ctrl_update(&self->vector_ctrl, &self->ref, &self->fbk);
    if (!self->torque_mode && self->enabled && !self->release_step) {
        pi_update(&self->speed_ctrl, self->ref.T_s, self->ref.tau_M);
    }
    self->release_step = self->vector_ctrl.sensorless && !self->enabled;
}

/* Signal vector of the references -------------------------------------------- */

static void cvc_references_unpack(const double u[CVC_REF_WIDTH], References *ref)
{
    References zero = {0};
    *ref = zero;
    ref->psi_s = u[CVC_REF_psi_s];
    ref->tau_M = u[CVC_REF_tau_M];
    ref->i_s = complex_from(u[CVC_REF_i_s], u[CVC_REF_i_s + 1]);
    ref->u_s = complex_from(u[CVC_REF_u_s], u[CVC_REF_u_s + 1]);
}
