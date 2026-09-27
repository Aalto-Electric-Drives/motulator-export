/*
 * Grid-following current-vector control of grid converters, ported from motulator.
 * See gfl_current_vector.h.
 */

#include "gfl_current_vector.h"

static GFLControllerCfg gfl_controller_cfg(double i_max, double L)
{
    GFLControllerCfg cfg;
    cfg.i_max = i_max;
    cfg.L = L;
    cfg.alpha_c = 2.0 * M_PI * 400.0;
    cfg.alpha_i = cfg.alpha_c;
    cfg.u_nom = sqrt(2.0 / 3.0) * 400.0;
    cfg.w_nom = 2.0 * M_PI * 50.0;
    cfg.alpha_pll = 2.0 * M_PI * 20.0;
    cfg.T_s = 125e-6;
    return cfg;
}

static void gfl_control_system_init(GFLControlSystem *self, const GFLControllerCfg *cfg)
{
    self->cfg = *cfg;
    /* CurrentController: 2DOF PI gains from the bandwidths and the inductance */
    double k_t = cfg->alpha_c * cfg->L;
    double k_i = cfg->alpha_c * cfg->alpha_i * cfg->L;
    double k_p = (cfg->alpha_c + cfg->alpha_i) * cfg->L;
    complex_pi_init(&self->current_ctrl, k_p, k_i, k_t);
    /* PLL */
    self->k_p_pll = 2.0 * cfg->alpha_pll;
    self->k_i_pll = cfg->alpha_pll * cfg->alpha_pll;
    self->w_g = cfg->w_nom;
    self->theta_c = 0.0;
    self->u_g = cfg->u_nom;
    pwm_init(&self->pwm, 1.5);
}

static void gfl_control_system_compute_output(GFLControlSystem *self,
                                              const GridMeasurements *meas,
                                              double p_g_ref, double q_g_ref)
{
    GFLFeedbacks *fbk = &self->fbk;
    GFLReferences *ref = &self->ref;

    /* Feedback signals from the PLL (PLL.compute_output) */
    fbk->theta_c = self->theta_c;
    fbk->w_g = self->w_g;
    double complex rot = cexp(-I * fbk->theta_c);
    fbk->i_c = rot * meas->i_c_ab;
    fbk->u_c = rot * self->pwm.realized_voltage;
    fbk->u_g_meas = rot * meas->u_g_ab;
    fbk->u_g = self->u_g;
    fbk->eps = (self->u_g > 0.0) ? cimag(fbk->u_g_meas) / self->u_g : 0.0;
    fbk->w_c = fbk->w_g + self->k_p_pll * fbk->eps;
    double complex s_g = 1.5 * fbk->u_g * conj(fbk->i_c);
    fbk->p_g = creal(s_g);
    fbk->q_g = cimag(s_g);
    fbk->u_dc = meas->u_dc;

    /* Current reference, limited to i_max (CurrentLimiter) */
    ref->T_s = self->cfg.T_s;
    ref->p_g = p_g_ref;
    ref->q_g = q_g_ref;
    ref->i_c = (p_g_ref - I * q_g_ref) / (1.5 * fbk->u_g);
    if (cabs(ref->i_c) > self->cfg.i_max) {
        ref->i_c = self->cfg.i_max * ref->i_c / cabs(ref->i_c);
    }

    /* Voltage reference from the current controller, with the PCC voltage as the
     * feedforward */
    ref->u_c = complex_pi_compute_output(&self->current_ctrl, ref->i_c, fbk->i_c,
                                         fbk->u_g);

    /* Duty ratios for the PWM */
    double complex u_c_ab_ref = cexp(I * fbk->theta_c) * ref->u_c;
    ref->u_c_ab = pwm_compute_output(&self->pwm, ref->T_s, u_c_ab_ref, fbk->u_dc,
                                     fbk->w_c, ref->d_abc);
}

static void gfl_control_system_update(GFLControlSystem *self)
{
    const GFLFeedbacks *fbk = &self->fbk;
    double T_s = self->ref.T_s;
    pwm_update(&self->pwm, self->ref.u_c_ab);
    /* Current controller: the realized voltage in controller coordinates */
    complex_pi_update(&self->current_ctrl, T_s, fbk->u_c, fbk->w_c);
    /* PLL */
    self->theta_c = wrap(self->theta_c + T_s * fbk->w_c);
    self->w_g += T_s * self->k_i_pll * fbk->eps;
    self->u_g += T_s * self->k_p_pll * (creal(fbk->u_g_meas) - self->u_g);
}
