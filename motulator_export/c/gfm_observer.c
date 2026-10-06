/*
 * Disturbance-observer-based grid-forming control of grid converters, ported from
 * motulator. See gfm_observer.h.
 */

#include "gfm_observer.h"

static GFMControllerCfg gfm_controller_cfg(double i_max, double L)
{
    GFMControllerCfg cfg;
    cfg.i_max = i_max;
    cfg.L = L;
    cfg.R = 0.0;
    cfg.R_a = NAN;
    cfg.k_v = NAN;
    cfg.alpha_o = 2.0 * M_PI * 50.0;
    cfg.alpha_c = 2.0 * M_PI * 400.0;
    cfg.u_nom = sqrt(2.0 / 3.0) * 400.0;
    cfg.w_nom = 2.0 * M_PI * 50.0;
    cfg.T_s = 125e-6;
    cfg.i_d_max = NAN;
    cfg.alpha_l = 2.0 * M_PI * 50.0;
    return cfg;
}

static void gfm_control_system_init(GFMControlSystem *self,
                                    const GFMControllerCfg *cfg)
{
    self->cfg = *cfg;
    /* Resolve the defaults, as in
     * ObserverBasedGridFormingControllerCfg.__post_init__ */
    self->R_a = isnan(cfg->R_a) ? 0.25 * cfg->u_nom / cfg->i_max : cfg->R_a;
    self->k_v = isnan(cfg->k_v) ? cfg->alpha_o / cfg->w_nom : cfg->k_v;
    self->k_c = cfg->alpha_c * cfg->L;
    self->p_max = isnan(cfg->i_d_max) ? INFINITY : 1.5 * cfg->u_nom * cfg->i_d_max;
    /* Observer */
    self->w_g = cfg->w_nom;
    self->theta_c = 0.0;
    self->u_gp = cfg->u_nom;
    pwm_init(&self->pwm, 1.5);
}

static void gfm_control_system_compute_output(GFMControlSystem *self,
                                              const GFMMeasurements *meas,
                                              double p_g_ref, double v_c_ref)
{
    const GFMControllerCfg *cfg = &self->cfg;
    GFMFeedbacks *fbk = &self->fbk;
    GFMReferences *ref = &self->ref;

    /* Feedback signals from the observer (Observer.compute_output) */
    fbk->theta_c = self->theta_c;
    double complex rot = cexp(-I * fbk->theta_c);
    fbk->i_c = rot * meas->i_c_ab;
    fbk->u_c = rot * pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 1);
    fbk->v_c = self->u_gp - (cfg->alpha_o - I * self->w_g) * cfg->L * fbk->i_c;
    fbk->u_g = self->u_gp - cfg->alpha_o * cfg->L * fbk->i_c;
    fbk->w_c = self->w_g;
    double complex s_g = 1.5 * fbk->u_g * conj(fbk->i_c);
    fbk->p_g = creal(s_g);
    fbk->q_g = cimag(s_g);
    fbk->u_dc = meas->u_dc;

    /* Limit the active-power reference */
    ref->T_s = cfg->T_s;
    ref->p_g = copysign(fmin(fabs(p_g_ref), self->p_max), p_g_ref);
    ref->v_c = v_c_ref;

    /* Complex gains and the feedback correction for the grid-forming mode */
    double complex exp_j_theta = cexp(I * carg(fbk->v_c));
    double complex k_p = exp_j_theta * self->R_a / (1.5 * ref->v_c);
    double complex k_v = exp_j_theta * (1.0 - I * self->k_v);
    double complex e_c =
        k_p * (ref->p_g - fbk->p_g) + k_v * (ref->v_c - cabs(fbk->v_c));

    /* Transparent current limitation (CurrentLimiter) */
    ref->i_c = fbk->i_c + e_c / self->k_c;
    if (cabs(ref->i_c) > cfg->i_max) {
        ref->i_c = cfg->i_max * ref->i_c / cabs(ref->i_c);
    }
    e_c = self->k_c * (ref->i_c - fbk->i_c);

    /* Voltage reference */
    ref->u_c = e_c + fbk->v_c + cfg->R * fbk->i_c;

    /* Duty ratios for the PWM */
    double complex u_c_ab_ref = cexp(I * fbk->theta_c) * ref->u_c;
    ref->u_c_ab = pwm_compute_output(&self->pwm, ref->T_s, u_c_ab_ref, fbk->u_dc,
                                     fbk->w_c, meas->i_c_ab, ref->d_abc);
}

static void gfm_control_system_update(GFMControlSystem *self)
{
    const GFMControllerCfg *cfg = &self->cfg;
    const GFMFeedbacks *fbk = &self->fbk;
    const GFMReferences *ref = &self->ref;
    double T_s = ref->T_s;
    pwm_update(&self->pwm, ref->d_abc);
    /* Observer */
    self->u_gp += T_s * cfg->alpha_o * (fbk->u_c - fbk->v_c - cfg->R * fbk->i_c);
    self->theta_c = wrap(self->theta_c + T_s * self->w_g);
    /* Maximum active power, prioritizing the reactive current */
    if (!isnan(cfg->i_d_max)) {
        double abs_u_c = fmax(cabs(ref->u_c), 1e-6);
        double i_q = cimag(ref->u_c * conj(fbk->i_c)) / abs_u_c;
        double i_d_max_sqr = cfg->i_d_max * cfg->i_d_max;
        double i_d_lim_sqr = fmin(fmax(i_d_max_sqr - i_q * i_q, 0.0), i_d_max_sqr);
        double p_max = 1.5 * abs_u_c * sqrt(i_d_lim_sqr);
        self->p_max += T_s * cfg->alpha_l * (p_max - self->p_max);
    }
}
