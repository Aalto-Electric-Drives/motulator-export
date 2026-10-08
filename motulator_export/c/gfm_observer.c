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

/* Observer ----------------------------------------------------------------- */

static void gfm_observer_init(GFMObserver *self, double u_nom, double w_nom,
                              double alpha_o, double L, double R)
{
    self->alpha_o = alpha_o;
    self->L = L;
    self->R = R;
    self->w_g = w_nom;
    self->theta_c = 0.0;
    self->u_gp = u_nom;
}

static void gfm_observer_compute_output(const GFMObserver *self,
                                        double complex u_c_ab, double complex i_c_ab,
                                        GFMFeedbacks *out)
{
    out->theta_c = self->theta_c;

    /* Measured values in synchronous coordinates */
    double complex rot = cexp(-I * out->theta_c);
    out->i_c = rot * i_c_ab;
    out->u_c = rot * u_c_ab;

    /* Estimates of the quasi-static converter voltage and the grid voltage */
    out->v_c = self->u_gp - (self->alpha_o - I * self->w_g) * self->L * out->i_c;
    out->u_g = self->u_gp - self->alpha_o * self->L * out->i_c;
    out->w_c = self->w_g;

    /* Active and reactive powers */
    double complex s_g = 1.5 * out->u_g * conj(out->i_c);
    out->p_g = creal(s_g);
    out->q_g = cimag(s_g);
}

static void gfm_observer_update(GFMObserver *self, double T_s, const GFMFeedbacks *out)
{
    self->u_gp += T_s * self->alpha_o * (out->u_c - out->v_c - self->R * out->i_c);
    self->theta_c = wrap(self->theta_c + T_s * self->w_g);
}

/* ObserverBasedGridFormingController --------------------------------------- */

static void gfm_controller_init(GFMController *self, const GFMControllerCfg *cfg)
{
    gfm_observer_init(&self->observer, cfg->u_nom, cfg->w_nom, cfg->alpha_o, cfg->L,
                      cfg->R);
    self->i_max = cfg->i_max;
    self->R_a = isnan(cfg->R_a) ? 0.25 * cfg->u_nom / cfg->i_max : cfg->R_a;
    self->k_v = isnan(cfg->k_v) ? cfg->alpha_o / cfg->w_nom : cfg->k_v;
    self->k_c = cfg->alpha_c * cfg->L;
    self->i_d_max = cfg->i_d_max;
    self->alpha_l = cfg->alpha_l;
    self->p_max = isnan(cfg->i_d_max) ? INFINITY : 1.5 * cfg->u_nom * cfg->i_d_max;
}

static void gfm_current_reference(const GFMController *self, double p_g_ref,
                                  double v_c_ref, const GFMFeedbacks *fbk,
                                  GFMReferences *ref)
{
    /* Limit the active-power reference */
    ref->p_g = copysign(fmin(fabs(p_g_ref), self->p_max), p_g_ref);
    ref->v_c = v_c_ref;

    /* Complex gains and the feedback correction for the grid-forming mode */
    double complex exp_j_theta = cexp(I * carg(fbk->v_c));
    double complex k_p = exp_j_theta * self->R_a / (1.5 * ref->v_c);
    double complex k_v = exp_j_theta * (1.0 - I * self->k_v);
    double complex e_c =
        k_p * (ref->p_g - fbk->p_g) + k_v * (ref->v_c - cabs(fbk->v_c));

    /* Current reference of the transparent current limitation */
    ref->i_c = fbk->i_c + e_c / self->k_c;
}

static void gfm_voltage_reference(const GFMController *self, const GFMFeedbacks *fbk,
                                  GFMReferences *ref)
{
    double complex e_c = self->k_c * (ref->i_c - fbk->i_c);
    ref->u_c = e_c + fbk->v_c + self->observer.R * fbk->i_c;
}

static void gfm_power_limit_update(GFMController *self, double T_s,
                                   const GFMReferences *ref, const GFMFeedbacks *fbk)
{
    if (isnan(self->i_d_max)) {
        return;
    }
    /* Maximum active power, prioritizing the reactive current */
    double abs_u_c = fmax(cabs(ref->u_c), 1e-6);
    double i_q = cimag(ref->u_c * conj(fbk->i_c)) / abs_u_c;
    double i_d_max_sqr = self->i_d_max * self->i_d_max;
    double i_d_lim_sqr = fmin(fmax(i_d_max_sqr - i_q * i_q, 0.0), i_d_max_sqr);
    double p_max = 1.5 * abs_u_c * sqrt(i_d_lim_sqr);
    self->p_max += T_s * self->alpha_l * (p_max - self->p_max);
}

/* GFMControlSystem --------------------------------------------------------- */

static void gfm_control_system_init(GFMControlSystem *self,
                                    const GFMControllerCfg *cfg)
{
    self->cfg = *cfg;
    gfm_controller_init(&self->ctrl, cfg);
    pwm_init(&self->pwm, 1.5);
}

static void gfm_control_system_compute_output(GFMControlSystem *self,
                                              const GFMMeasurements *meas,
                                              double p_g_ref, double v_c_ref)
{
    GFMFeedbacks *fbk = &self->fbk;
    GFMReferences *ref = &self->ref;

    /* Feedback signals from the observer */
    double complex u_c_ab =
        pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 1);
    gfm_observer_compute_output(&self->ctrl.observer, u_c_ab, meas->i_c_ab, fbk);
    fbk->u_dc = meas->u_dc;

    /* References, with the transparent current limitation (CurrentLimiter) */
    ref->T_s = self->cfg.T_s;
    gfm_current_reference(&self->ctrl, p_g_ref, v_c_ref, fbk, ref);
    ref->i_c = current_limiter(self->ctrl.i_max, ref->i_c);
    gfm_voltage_reference(&self->ctrl, fbk, ref);

    /* Duty ratios for the PWM */
    double complex u_c_ab_ref = cexp(I * fbk->theta_c) * ref->u_c;
    ref->u_c_ab = pwm_compute_output(&self->pwm, ref->T_s, u_c_ab_ref, fbk->u_dc,
                                     fbk->w_c, meas->i_c_ab, ref->d_abc);
}

static void gfm_control_system_update(GFMControlSystem *self)
{
    double T_s = self->ref.T_s;
    pwm_update(&self->pwm, self->ref.d_abc);
    gfm_observer_update(&self->ctrl.observer, T_s, &self->fbk);
    gfm_power_limit_update(&self->ctrl, T_s, &self->ref, &self->fbk);
}

/* Signal vectors of the modular control system ---------------------------- */

#define COMPLEX_AT(u, i) complex_from((u)[i], (u)[(i) + 1])
#define SET_COMPLEX(y, i, z)                                                       \
    do {                                                                           \
        (y)[i] = creal(z);                                                         \
        (y)[(i) + 1] = cimag(z);                                                   \
    } while (0)

static void gfm_measurements_pack(const GFMMeasurements *meas, double y[MEAS_WIDTH])
{
    SET_COMPLEX(y, MEAS_i_c_ab, meas->i_c_ab);
    y[MEAS_u_dc] = meas->u_dc;
}

static void gfm_measurements_unpack(const double u[MEAS_WIDTH], GFMMeasurements *meas)
{
    meas->i_c_ab = COMPLEX_AT(u, MEAS_i_c_ab);
    meas->u_dc = u[MEAS_u_dc];
}

static void gfm_feedbacks_pack(const GFMFeedbacks *fbk, double y[FBK_WIDTH])
{
    SET_COMPLEX(y, FBK_i_c, fbk->i_c);
    SET_COMPLEX(y, FBK_u_c, fbk->u_c);
    SET_COMPLEX(y, FBK_v_c, fbk->v_c);
    SET_COMPLEX(y, FBK_u_g, fbk->u_g);
    y[FBK_p_g] = fbk->p_g;
    y[FBK_q_g] = fbk->q_g;
    y[FBK_theta_c] = fbk->theta_c;
    y[FBK_w_c] = fbk->w_c;
}

static void gfm_feedbacks_unpack(const double u[FBK_WIDTH], GFMFeedbacks *fbk)
{
    fbk->i_c = COMPLEX_AT(u, FBK_i_c);
    fbk->u_c = COMPLEX_AT(u, FBK_u_c);
    fbk->v_c = COMPLEX_AT(u, FBK_v_c);
    fbk->u_g = COMPLEX_AT(u, FBK_u_g);
    fbk->p_g = u[FBK_p_g];
    fbk->q_g = u[FBK_q_g];
    fbk->theta_c = u[FBK_theta_c];
    fbk->w_c = u[FBK_w_c];
    fbk->u_dc = 0.0;
}

static void gfm_references_unpack(const double u[REF_WIDTH], GFMReferences *ref)
{
    GFMReferences zero = {0};
    *ref = zero;
    ref->p_g = u[REF_p_g];
    ref->v_c = u[REF_v_c];
    ref->i_c = COMPLEX_AT(u, REF_i_c);
    ref->u_c = COMPLEX_AT(u, REF_u_c);
}

#undef COMPLEX_AT
#undef SET_COMPLEX
