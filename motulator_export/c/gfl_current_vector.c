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

/* PLL ---------------------------------------------------------------------- */

static void pll_init(PLL *self, double u_nom, double w_nom, double alpha_pll)
{
    self->k_p = 2.0 * alpha_pll;
    self->k_i = alpha_pll * alpha_pll;
    self->w_g = w_nom;
    self->theta_c = 0.0;
    self->u_g = u_nom;
}

static void pll_compute_output(const PLL *self, double complex u_c_ab,
                               double complex i_c_ab, double complex u_g_ab,
                               GFLFeedbacks *out)
{
    out->theta_c = self->theta_c;
    out->w_g = self->w_g;

    /* Coordinate transformations */
    double complex rot = cexp(-I * out->theta_c);
    out->i_c = rot * i_c_ab;
    out->u_c = rot * u_c_ab;
    out->u_g_meas = rot * u_g_ab;
    /* Filtered voltage magnitude */
    out->u_g = self->u_g;

    /* Error signal and the angular speed of the coordinate system */
    out->eps = (self->u_g > 0.0) ? cimag(out->u_g_meas) / self->u_g : 0.0;
    out->w_c = out->w_g + self->k_p * out->eps;

    /* Powers */
    double complex s_g = 1.5 * out->u_g * conj(out->i_c);
    out->p_g = creal(s_g);
    out->q_g = cimag(s_g);
}

static void pll_update(PLL *self, double T_s, const GFLFeedbacks *out)
{
    self->theta_c = wrap(self->theta_c + T_s * out->w_c);
    self->w_g += T_s * self->k_i * out->eps;
    self->u_g += T_s * self->k_p * (creal(out->u_g_meas) - self->u_g);
}

/* CurrentController and CurrentVectorController ---------------------------- */

static ComplexPIController gfl_current_controller(double L, double alpha_c,
                                                  double alpha_i)
{
    ComplexPIController ctrl;
    double k_t = alpha_c * L;
    double k_i = alpha_c * alpha_i * L;
    double k_p = (alpha_c + alpha_i) * L;
    complex_pi_init(&ctrl, k_p, k_i, k_t);
    return ctrl;
}

static double complex gfl_current_reference(double p_g_ref, double q_g_ref,
                                            double u_g)
{
    return (p_g_ref - I * q_g_ref) / (1.5 * u_g);
}

/* GFLControlSystem --------------------------------------------------------- */

static void gfl_control_system_init(GFLControlSystem *self, const GFLControllerCfg *cfg)
{
    self->cfg = *cfg;
    self->current_ctrl = gfl_current_controller(cfg->L, cfg->alpha_c, cfg->alpha_i);
    pll_init(&self->pll, cfg->u_nom, cfg->w_nom, cfg->alpha_pll);
    pwm_init(&self->pwm, 1.5);
}

static void gfl_control_system_compute_output(GFLControlSystem *self,
                                              const GridMeasurements *meas,
                                              double p_g_ref, double q_g_ref)
{
    GFLFeedbacks *fbk = &self->fbk;
    GFLReferences *ref = &self->ref;

    /* Feedback signals from the PLL */
    double complex u_c_ab =
        pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 1);
    pll_compute_output(&self->pll, u_c_ab, meas->i_c_ab, meas->u_g_ab, fbk);
    fbk->u_dc = meas->u_dc;

    /* Current reference, limited to i_max (CurrentLimiter) */
    ref->T_s = self->cfg.T_s;
    ref->p_g = p_g_ref;
    ref->q_g = q_g_ref;
    ref->i_c = current_limiter(self->cfg.i_max,
                               gfl_current_reference(p_g_ref, q_g_ref, fbk->u_g));

    /* Voltage reference from the current controller, with the PCC voltage as the
     * feedforward */
    ref->u_c = complex_pi_compute_output(&self->current_ctrl, ref->i_c, fbk->i_c,
                                         fbk->u_g);

    /* Duty ratios for the PWM */
    double complex u_c_ab_ref = cexp(I * fbk->theta_c) * ref->u_c;
    ref->u_c_ab = pwm_compute_output(&self->pwm, ref->T_s, u_c_ab_ref, fbk->u_dc,
                                     fbk->w_c, meas->i_c_ab, ref->d_abc);
}

static void gfl_control_system_update(GFLControlSystem *self)
{
    const GFLFeedbacks *fbk = &self->fbk;
    double T_s = self->ref.T_s;
    pwm_update(&self->pwm, self->ref.d_abc);
    /* Current controller: the realized voltage in controller coordinates */
    complex_pi_update(&self->current_ctrl, T_s, fbk->u_c, fbk->w_c);
    pll_update(&self->pll, T_s, fbk);
}

/* Signal vectors of the modular control system ---------------------------- */

#define COMPLEX_AT(u, i) complex_from((u)[i], (u)[(i) + 1])
#define SET_COMPLEX(y, i, z)                                                       \
    do {                                                                           \
        (y)[i] = creal(z);                                                         \
        (y)[(i) + 1] = cimag(z);                                                   \
    } while (0)

static void grid_measurements_pack(const GridMeasurements *meas, double y[MEAS_WIDTH])
{
    SET_COMPLEX(y, MEAS_i_c_ab, meas->i_c_ab);
    SET_COMPLEX(y, MEAS_u_g_ab, meas->u_g_ab);
    y[MEAS_u_dc] = meas->u_dc;
}

static void grid_measurements_unpack(const double u[MEAS_WIDTH],
                                     GridMeasurements *meas)
{
    meas->i_c_ab = COMPLEX_AT(u, MEAS_i_c_ab);
    meas->u_g_ab = COMPLEX_AT(u, MEAS_u_g_ab);
    meas->u_dc = u[MEAS_u_dc];
}

static void gfl_feedbacks_pack(const GFLFeedbacks *fbk, double y[FBK_WIDTH])
{
    SET_COMPLEX(y, FBK_i_c, fbk->i_c);
    SET_COMPLEX(y, FBK_u_c, fbk->u_c);
    y[FBK_u_g] = fbk->u_g;
    SET_COMPLEX(y, FBK_u_g_meas, fbk->u_g_meas);
    y[FBK_theta_c] = fbk->theta_c;
    y[FBK_w_g] = fbk->w_g;
    y[FBK_w_c] = fbk->w_c;
    y[FBK_p_g] = fbk->p_g;
    y[FBK_q_g] = fbk->q_g;
    y[FBK_eps] = fbk->eps;
}

static void gfl_feedbacks_unpack(const double u[FBK_WIDTH], GFLFeedbacks *fbk)
{
    fbk->i_c = COMPLEX_AT(u, FBK_i_c);
    fbk->u_c = COMPLEX_AT(u, FBK_u_c);
    fbk->u_g = u[FBK_u_g];
    fbk->u_g_meas = COMPLEX_AT(u, FBK_u_g_meas);
    fbk->theta_c = u[FBK_theta_c];
    fbk->w_g = u[FBK_w_g];
    fbk->w_c = u[FBK_w_c];
    fbk->p_g = u[FBK_p_g];
    fbk->q_g = u[FBK_q_g];
    fbk->eps = u[FBK_eps];
    fbk->u_dc = 0.0;
}

static void gfl_references_unpack(const double u[REF_WIDTH], GFLReferences *ref)
{
    GFLReferences zero = {0};
    *ref = zero;
    ref->p_g = u[REF_p_g];
    ref->q_g = u[REF_q_g];
    ref->i_c = COMPLEX_AT(u, REF_i_c);
    ref->u_c = COMPLEX_AT(u, REF_u_c);
}

#undef COMPLEX_AT
#undef SET_COMPLEX
