/*
 * Current-vector control of induction machine drives, ported from motulator. See
 * im_current_vector.h.
 */

#include "im_current_vector.h"

/* Derived parameters (properties of InductionMachineInvGammaPars) */
static double im_alpha(const InductionMachineInvGammaPars *par)
{
    return par->R_R / par->L_M;
}

static double im_w_rb(const InductionMachineInvGammaPars *par)
{
    return (par->L_sgm > 0.0) ? par->R_R * (1.0 / par->L_sgm + 1.0 / par->L_M) : 0.0;
}

/* Configuration ------------------------------------------------------------ */

static IMCurrentVectorControllerCfg im_current_vector_controller_cfg(double psi_s_nom,
                                                                     double i_s_max)
{
    IMCurrentVectorControllerCfg cfg;
    cfg.psi_s_nom = psi_s_nom;
    cfg.i_s_max = i_s_max;
    cfg.alpha_c = 2.0 * M_PI * 200.0;
    cfg.alpha_i = NAN;
    cfg.alpha_o = NAN;
    cfg.w_s_nom = 2.0 * M_PI * 50.0;
    cfg.k_u = 0.95;
    cfg.k_fw = NAN;
    cfg.J = NAN;
    cfg.sensorless = 1;
    cfg.T_s = 125e-6;
    return cfg;
}

/* IMFluxObserver ----------------------------------------------------------- */

static void im_flux_observer_init(IMFluxObserver *self,
                                  InductionMachineInvGammaPars par, int sensorless)
{
    self->par = par;
    self->sensorless = sensorless;
    self->psi_s = 0.0;
    self->theta_c = 0.0;
    self->T_s_old = 0.0;
    self->old_i_s = 0.0;
}

/* Default observer gain of create_speed_flux_observer */
static double complex im_observer_gain(const IMFluxObserver *self, double w_m)
{
    double alpha = im_alpha(&self->par);
    if (self->sensorless) {
        return (0.5 * alpha + 0.2 * fabs(w_m)) / (alpha - I * w_m);
    }
    return 1.0 + 0.2 * fabs(w_m) / (alpha - I * w_m);
}

static void im_flux_observer_compute_output(const IMFluxObserver *self,
                                            double complex u_s_ab,
                                            double complex i_s_ab, double w_M,
                                            double eps_ext, double h,
                                            IMObserverOutputs *out)
{
    /* The weight h of the external speed error signal eps_ext selects between the
     * sensorless (h = 0) and sensored (h = 1) modes */
    const InductionMachineInvGammaPars *par = &self->par;
    out->psi_s = self->psi_s;
    out->theta_c = self->theta_c;
    out->h = h;

    /* Current and voltage vectors in estimated rotor flux coordinates */
    out->i_s = cexp(-I * out->theta_c) * i_s_ab;
    out->u_s = cexp(-I * out->theta_c) * u_s_ab;

    /* Mechanical and electrical angular speeds of the rotor */
    out->w_M = w_M;
    out->w_m = par->n_p * w_M;

    /* Rotor flux estimate */
    out->psi_R = out->psi_s - par->L_sgm * self->old_i_s;

    /* Slip angular frequency */
    double complex prod = out->psi_s * conj(out->psi_R);
    out->w_r = (creal(prod) > 0.0) ? im_w_rb(par) * cimag(prod) / creal(prod) : 0.0;

    /* Angular speed of the coordinate system equals synchronous angular frequency */
    out->w_c = out->w_s = out->w_m + out->w_r;

    /* Estimation error */
    double complex d_i_s =
        (self->T_s_old > 0.0) ? (out->i_s - self->old_i_s) / self->T_s_old : 0.0;
    double R_sgm = par->R_s + par->R_R;
    out->e_o = par->L_sgm * d_i_s - out->u_s +
               (R_sgm + I * out->w_c * par->L_sgm) * out->i_s -
               (im_alpha(par) - I * out->w_m) * out->psi_R;

    /* Torque estimate */
    out->tau_M =
        (par->L_sgm > 0.0) ? 1.5 * par->n_p * cimag(out->i_s * conj(out->psi_s)) : 0.0;

    /* Rotor speed error signal for the speed observer. The model-based part is
     * faded out as the external error signal takes over. */
    double eps_o =
        (cabs(out->psi_R) > 0.0) ? -cimag(out->e_o / out->psi_R) / par->n_p : 0.0;
    out->eps = h * eps_ext + (1.0 - h) * eps_o;
}

static void im_flux_observer_update(IMFluxObserver *self, double T_s,
                                    const IMObserverOutputs *out)
{
    const InductionMachineInvGammaPars *par = &self->par;

    /* Observer gains. The conjugate-error gain is faded out together with the
     * model-based error signal. */
    double complex k_o1 = im_observer_gain(self, out->w_m);
    double complex proj =
        (cabs(out->psi_R) > 0.0) ? out->psi_R / conj(out->psi_R) : 0.0;
    double complex k_o2 = (1.0 - out->h) * k_o1 * proj;

    /* Update the states */
    double complex v_err = k_o1 * out->e_o + k_o2 * conj(out->e_o);
    double complex d_psi_s =
        out->u_s - par->R_s * out->i_s - I * out->w_c * out->psi_s + v_err;
    self->psi_s += T_s * d_psi_s;
    self->theta_c = wrap(self->theta_c + T_s * out->w_c);

    /* Sampling period and current for the next sampling period */
    self->T_s_old = T_s;
    self->old_i_s = out->i_s;
}

/* IMSpeedFluxObserver ------------------------------------------------------ */

static void im_speed_flux_observer_init(IMSpeedFluxObserver *self,
                                        InductionMachineInvGammaPars par,
                                        double alpha_o, int sensorless, double J)
{
    /* Observer gains for critically damped dynamics */
    double k_w, k_tau;
    if (J <= 0.0) {
        k_w = alpha_o;
        k_tau = 0.0;
    } else {
        k_w = 2.0 * alpha_o;
        k_tau = J * alpha_o * alpha_o;
    }
    speed_observer_init(&self->speed_observer, k_w, k_tau, J);
    im_flux_observer_init(&self->flux_observer, par, sensorless);
}

static void im_speed_flux_observer_compute_output(const IMSpeedFluxObserver *self,
                                                  double complex u_s_ab,
                                                  double complex i_s_ab,
                                                  double eps_ext, double h,
                                                  IMObserverOutputs *out)
{
    double w_M = self->speed_observer.w_M;
    double tau_L = self->speed_observer.tau_L;
    im_flux_observer_compute_output(&self->flux_observer, u_s_ab, i_s_ab, w_M,
                                    eps_ext, h, out);
    out->tau_L = tau_L;
}

static void im_speed_flux_observer_update(IMSpeedFluxObserver *self, double T_s,
                                          const IMObserverOutputs *out)
{
    speed_observer_update(&self->speed_observer, T_s, out->eps, out->tau_M);
    im_flux_observer_update(&self->flux_observer, T_s, out);
}

/* IMCurrentReferenceGenerator ---------------------------------------------- */

static void im_reference_gen_init(IMCurrentReferenceGenerator *self,
                                  InductionMachineInvGammaPars par, double psi_s_nom,
                                  double i_s_max, double w_s_nom, double k_u,
                                  double k_fw)
{
    self->par = par;
    self->i_s_max = i_s_max;
    self->k_u = k_u;
    self->i_sd_nom = psi_s_nom / (par.L_M + par.L_sgm);
    if (isnan(k_fw) || k_fw == 0.0) {
        k_fw = 2.0 * par.R_R / (w_s_nom * par.L_sgm * par.L_sgm);
    }
    self->k_fw = k_fw;
    self->i_sd_ref = self->i_sd_nom;
}

static void im_reference_gen_compute_output(const IMCurrentReferenceGenerator *self,
                                            double tau_M_ref, double psi_R,
                                            double complex *i_s_ref,
                                            double *tau_M_ref_lim)
{
    const InductionMachineInvGammaPars *par = &self->par;
    double i_sd_ref = self->i_sd_ref;

    /* q-axis current reference */
    double i_sq_ref = (psi_R > 0.0) ? tau_M_ref / (1.5 * par->n_p * psi_R) : 0.0;

    /* q-axis current limit: priority given to the d component, and the breakdown
     * torque limit */
    double i_sq_max1 = sqrt(self->i_s_max * self->i_s_max - i_sd_ref * i_sd_ref);
    double i_sq_max2 = psi_R / par->L_sgm + i_sd_ref;
    double i_sq_max = fmin(i_sq_max1, i_sq_max2);
    i_sq_ref = clip(i_sq_ref, -i_sq_max, i_sq_max);

    *i_s_ref = i_sd_ref + I * i_sq_ref;

    /* Limited torque (for the speed controller) */
    *tau_M_ref_lim = 1.5 * par->n_p * psi_R * i_sq_ref;
}

static void im_reference_gen_update(IMCurrentReferenceGenerator *self, double T_s,
                                    double complex u_s_ref, double u_dc)
{
    /* Field weakening based on the unlimited reference voltage */
    double u_s_max = self->k_u * u_dc / sqrt(3.0);
    self->i_sd_ref += T_s * self->k_fw * (u_s_max - cabs(u_s_ref));
    self->i_sd_ref = clip(self->i_sd_ref, -self->i_s_max, self->i_sd_nom);
}

/* IMCurrentVectorController ------------------------------------------------ */

static void im_current_vector_ctrl_init(IMCurrentVectorController *self,
                                        InductionMachineInvGammaPars par,
                                        const IMCurrentVectorControllerCfg *cfg)
{
    /* Resolve the defaults, as in CurrentVectorControllerCfg.__post_init__ and
     * CurrentVectorController.__init__ */
    double alpha_i = isnan(cfg->alpha_i) ? cfg->alpha_c : cfg->alpha_i;
    double J = isnan(cfg->J) ? 0.0 : cfg->J; /* Zero means None */
    double alpha_o = cfg->alpha_o;
    if (isnan(alpha_o)) {
        double alpha = 2.0 * M_PI * 60.0;
        alpha_o = (J > 0.0) ? 0.5 * alpha : alpha;
    }

    self->T_s = cfg->T_s;
    self->sensorless = cfg->sensorless;
    im_reference_gen_init(&self->reference_gen, par, cfg->psi_s_nom, cfg->i_s_max,
                          cfg->w_s_nom, cfg->k_u, cfg->k_fw);
    /* CurrentController: 2DOF PI gains from the bandwidths and the leakage
     * inductance */
    double k_t = cfg->alpha_c * par.L_sgm;
    double k_i = cfg->alpha_c * alpha_i * par.L_sgm;
    double k_p = (cfg->alpha_c + alpha_i) * par.L_sgm;
    complex_pi_init(&self->current_ctrl, k_p, k_i, k_t);
    im_speed_flux_observer_init(&self->observer, par, alpha_o, cfg->sensorless, J);
}

static void im_current_vector_ctrl_compute_output(IMCurrentVectorController *self,
                                                  double tau_M_ref,
                                                  const IMObserverOutputs *fbk,
                                                  IMReferences *ref)
{
    ref->T_s = self->T_s;
    im_reference_gen_compute_output(&self->reference_gen, tau_M_ref,
                                    cabs(fbk->psi_R), &ref->i_s, &ref->tau_M);
    /* Transform the reference to the same coordinates as the feedback */
    ref->i_s = cexp(I * carg(fbk->psi_R)) * ref->i_s;
    ref->u_s = complex_pi_compute_output(&self->current_ctrl, ref->i_s, fbk->i_s, 0.0);
}

static void im_current_vector_ctrl_update(IMCurrentVectorController *self,
                                          const IMReferences *ref,
                                          const IMObserverOutputs *fbk)
{
    im_speed_flux_observer_update(&self->observer, ref->T_s, fbk);
    im_reference_gen_update(&self->reference_gen, ref->T_s, ref->u_s, fbk->u_dc);
    complex_pi_update(&self->current_ctrl, ref->T_s, fbk->u_s, fbk->w_c);
}

/* IMVectorControlSystem ---------------------------------------------------- */

static void im_vector_control_system_init(IMVectorControlSystem *self,
                                          InductionMachineInvGammaPars par,
                                          const IMCurrentVectorControllerCfg *cfg,
                                          PIController speed_ctrl)
{
    pwm_init(&self->pwm, 1.5);
    im_current_vector_ctrl_init(&self->vector_ctrl, par, cfg);
    self->speed_ctrl = speed_ctrl;
    self->speed_ctrl.u_i = 0.0;
    self->speed_ctrl.v = 0.0;
}

static void im_vector_control_system_compute_output(IMVectorControlSystem *self,
                                                    const IMMeasurements *meas,
                                                    double w_M_ref)
{
    IMObserverOutputs *fbk = &self->fbk;
    IMReferences *ref = &self->ref;

    /* Feedback signals */
    double complex u_c_ab = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc);
    const IMSpeedFluxObserver *observer = &self->vector_ctrl.observer;
    if (self->vector_ctrl.sensorless) {
        im_speed_flux_observer_compute_output(observer, u_c_ab, meas->i_c_ab, 0.0,
                                              0.0, fbk);
    } else {
        /* Speed error from the measured rotor speed */
        double eps = meas->w_M - observer->speed_observer.w_M;
        im_speed_flux_observer_compute_output(observer, u_c_ab, meas->i_c_ab, eps,
                                              1.0, fbk);
    }
    fbk->u_dc = meas->u_dc;

    /* Speed controller and vector controller */
    double tau_M_ref = pi_compute_output(&self->speed_ctrl, w_M_ref, fbk->w_M, 0.0);
    im_current_vector_ctrl_compute_output(&self->vector_ctrl, tau_M_ref, fbk, ref);

    /* Duty ratios for the PWM */
    double complex u_s_ab_ref = cexp(I * fbk->theta_c) * ref->u_s;
    ref->u_c_ab = pwm_compute_output(&self->pwm, ref->T_s, u_s_ab_ref, fbk->u_dc,
                                     fbk->w_c, meas->i_c_ab, ref->d_abc);
    ref->w_M = w_M_ref;
}

static void im_vector_control_system_update(IMVectorControlSystem *self)
{
    pwm_update(&self->pwm, self->ref.u_c_ab, self->ref.d_abc);
    im_current_vector_ctrl_update(&self->vector_ctrl, &self->ref, &self->fbk);
    pi_update(&self->speed_ctrl, self->ref.T_s, self->ref.tau_M);
}
