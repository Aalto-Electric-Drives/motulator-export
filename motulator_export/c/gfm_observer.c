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

static void gfm_observer_init(GFMObserver *self, double u_nom, double w_nom, double L,
                              double alpha_o, double T_s, double R)
{
    self->L = L;
    self->R = R;
    self->w_g = w_nom;
    /* Coefficients of the hold-equivalent model */
    self->phi = cexp(-I * w_nom * T_s);
    self->gamma_c = T_s * self->phi / L;
    self->gamma_g = (self->phi - 1.0) / (I * w_nom * L);
    self->k_o = (1.0 - exp(-alpha_o * T_s)) / self->gamma_g; /* Observer gain */
    self->theta_c = 0.0;
    self->u_g = u_nom;
    self->i_c = 0.0;
}

static void gfm_observer_compute_output(const GFMObserver *self,
                                        double complex u_c_ab, double complex i_c_ab,
                                        GFMFeedbacks *out)
{
    out->theta_c = self->theta_c;
    out->w_c = self->w_g;

    /* Measured values in synchronous coordinates */
    double complex rot = cexp(-I * out->theta_c);
    out->i_c = rot * i_c_ab;
    out->u_c_zoh = rot * u_c_ab;

    /* Correct the predicted grid-voltage estimate */
    out->u_g = self->u_g + self->k_o * (out->i_c - self->i_c);
    out->v_c = out->u_g + I * self->w_g * self->L * out->i_c;

    /* Active and reactive powers */
    double complex s_g = 1.5 * out->u_g * conj(out->i_c);
    out->p_g = creal(s_g);
    out->q_g = cimag(s_g);
}

static void gfm_observer_update(GFMObserver *self, double T_s, const GFMFeedbacks *out)
{
    /* Predict the current, the grid voltage being constant in these coordinates */
    self->i_c = self->phi * out->i_c + self->gamma_c * out->u_c_zoh +
                self->gamma_g * (out->u_g + self->R * out->i_c);
    self->u_g = out->u_g;
    self->theta_c = wrap(self->theta_c + T_s * self->w_g);
}

/* ObserverBasedGridFormingController --------------------------------------- */

static void gfm_controller_init(GFMController *self, const GFMControllerCfg *cfg)
{
    gfm_observer_init(&self->observer, cfg->u_nom, cfg->w_nom, cfg->L, cfg->alpha_o,
                      cfg->T_s, cfg->R);
    self->i_max = cfg->i_max;
    self->R_a = isnan(cfg->R_a) ? 0.25 * cfg->u_nom / cfg->i_max : cfg->R_a;
    self->k_v = isnan(cfg->k_v) ? cfg->alpha_o / cfg->w_nom : cfg->k_v;
    self->k_c = cfg->alpha_c * cfg->L;
    self->power_limitation = !isnan(cfg->i_d_max);
    if (self->power_limitation) {
        power_reference_limiter_init(&self->power_limiter, cfg->i_d_max,
                                     cfg->alpha_l, cfg->u_nom);
    }
}

static void gfm_current_reference(const GFMController *self, double p_g_ref,
                                  double v_c_ref, const GFMFeedbacks *fbk,
                                  GFMReferences *ref)
{
    ref->p_g = p_g_ref;
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

/* GFMControlSystem --------------------------------------------------------- */

static void gfm_control_system_init(GFMControlSystem *self,
                                    const GFMControllerCfg *cfg)
{
    self->cfg = *cfg;
    gfm_controller_init(&self->ctrl, cfg);
    pwm_init(&self->pwm, 1.5);
    self->k_comp = 1.5; /* k_comp of ObserverBasedGridFormingController */
}

static void gfm_control_system_compute_output(GFMControlSystem *self,
                                              const GFMMeasurements *meas,
                                              double p_g_ref, double v_c_ref,
                                              int enabled)
{
    GFMFeedbacks *fbk = &self->fbk;
    GFMReferences *ref = &self->ref;

    /* Feedback signals from the observer, which uses the voltage of the ongoing
     * sampling period, while the control laws use the voltage averaged over the
     * previous and ongoing periods. With blocked gates, both are the measured PCC
     * voltage. */
    double complex u_c_ab_zoh = meas->u_g_ab;
    double complex u_c_ab = meas->u_g_ab;
    if (enabled) {
        u_c_ab_zoh = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 0);
        u_c_ab = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 1);
    }
    gfm_observer_compute_output(&self->ctrl.observer, u_c_ab_zoh, meas->i_c_ab, fbk);
    fbk->u_c = cexp(-I * fbk->theta_c) * u_c_ab;
    fbk->u_dc = meas->u_dc;

    ref->T_s = self->cfg.T_s;
    if (enabled) {
        /* References, with the limited active-power reference
         * (PowerReferenceLimiter) and the transparent current limitation
         * (CurrentLimiter) */
        if (self->ctrl.power_limitation) {
            p_g_ref = power_reference_limiter_compute_output(
                &self->ctrl.power_limiter, p_g_ref);
        }
        gfm_current_reference(&self->ctrl, p_g_ref, v_c_ref, fbk, ref);
        ref->i_c = current_limiter(self->ctrl.i_max, ref->i_c);
        gfm_voltage_reference(&self->ctrl, fbk, ref);
    } else {
        /* Standby output: the voltage reference equals the estimated quasi-static
         * converter voltage */
        ref->p_g = fbk->p_g;
        ref->v_c = cabs(fbk->v_c);
        ref->i_c = fbk->i_c;
        ref->u_c = fbk->v_c + self->ctrl.observer.R * fbk->i_c;
    }

    /* Duty ratios for the PWM, the angle of the voltage reference advanced by
     * k_comp*T_s*w_c */
    double complex u_c_ab_ref = cexp(I * fbk->theta_c) * ref->u_c;
    u_c_ab_ref *= cexp(I * self->k_comp * ref->T_s * fbk->w_c);
    ref->u_c_ab = pwm_compute_output(&self->pwm, ref->T_s, u_c_ab_ref, fbk->u_dc,
                                     fbk->w_c, meas->i_c_ab, ref->d_abc);
}

static void gfm_control_system_update(GFMControlSystem *self)
{
    double T_s = self->ref.T_s;
    pwm_update(&self->pwm, T_s, self->ref.d_abc);
    gfm_observer_update(&self->ctrl.observer, T_s, &self->fbk);
    if (self->ctrl.power_limitation) {
        power_reference_limiter_update(&self->ctrl.power_limiter, T_s, self->ref.u_c,
                                       self->fbk.i_c, 0.0);
    }
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
    SET_COMPLEX(y, MEAS_u_g_ab, meas->u_g_ab);
}

static void gfm_measurements_unpack(const double u[MEAS_WIDTH], GFMMeasurements *meas)
{
    meas->i_c_ab = COMPLEX_AT(u, MEAS_i_c_ab);
    meas->u_dc = u[MEAS_u_dc];
    meas->u_g_ab = COMPLEX_AT(u, MEAS_u_g_ab);
}

static void gfm_feedbacks_pack(const GFMFeedbacks *fbk, double y[FBK_WIDTH])
{
    SET_COMPLEX(y, FBK_i_c, fbk->i_c);
    SET_COMPLEX(y, FBK_u_c, fbk->u_c);
    SET_COMPLEX(y, FBK_u_c_zoh, fbk->u_c_zoh);
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
    fbk->u_c_zoh = COMPLEX_AT(u, FBK_u_c_zoh);
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
