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
    cfg.discrete = 0;
    return cfg;
}

/* IMFluxObserver ----------------------------------------------------------- */

static void im_flux_observer_init(IMFluxObserver *self,
                                  InductionMachineInvGammaPars par, int sensorless)
{
    self->par = par;
    self->sensorless = sensorless;
    self->unit_gain = 0;
    self->psi_s = 0.0;
    self->theta_c = 0.0;
    self->T_s_old = 0.0;
    self->psi_R_pred = 0.0;
}

/* Default observer gain of create_speed_flux_observer (default_sensorless_k_o in the
 * sensorless mode) */
static double complex im_observer_gain(const IMFluxObserver *self, double w_m)
{
    double alpha = im_alpha(&self->par);
    if (self->unit_gain) {
        return 1.0;
    }
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

    /* Current vector in synchronous coordinates. The voltage averaged over the
     * previous and ongoing sampling periods is set by the control system
     * (DriveControlSystem.get_feedback). */
    out->i_s = cexp(-I * out->theta_c) * i_s_ab;
    out->u_s = 0.0;

    /* Mechanical and electrical angular speeds of the rotor */
    out->w_M = w_M;
    out->w_m = par->n_p * w_M;

    /* Rotor flux estimate. If the voltage is unknown (NAN), the flux estimate follows
     * the current model alone, so the estimation error is zero. */
    if (isnan(creal(u_s_ab)) && self->T_s_old > 0.0) {
        out->psi_s = self->psi_R_pred + par->L_sgm * out->i_s;
    }
    out->psi_R = out->psi_s - par->L_sgm * out->i_s;

    /* Slip angular frequency */
    double complex prod = out->psi_s * conj(out->psi_R);
    out->w_r = (creal(prod) > 0.0) ? im_w_rb(par) * cimag(prod) / creal(prod) : 0.0;

    /* Angular speed of the coordinate system equals synchronous angular frequency */
    out->w_c = out->w_s = out->w_m + out->w_r;

    /* Voltage vector in the observer coordinates, or the open-circuit voltage if
     * unknown (NAN, blocked gates), averaged over a period of the length of the
     * previous one. The rotor flux decays as in the current model below, so their
     * predictions match. */
    if (!isnan(creal(u_s_ab))) {
        out->u_s_zoh = cexp(-I * out->theta_c) * u_s_ab;
    } else if (self->T_s_old > 0.0) {
        double complex Phi =
            cexp(I * out->w_m * self->T_s_old) * (1.0 - im_alpha(par) * self->T_s_old);
        out->u_s_zoh = (Phi - 1.0) / self->T_s_old * out->psi_R + par->R_s * out->i_s;
    } else {
        out->u_s_zoh =
            (I * out->w_m - im_alpha(par)) * out->psi_R + par->R_s * out->i_s;
    }

    /* Estimation error, i.e., the difference between the rotor flux predictions of
     * the current model and the voltage model */
    out->e_o = (self->T_s_old > 0.0) ? (self->psi_R_pred - out->psi_R) / self->T_s_old
                                     : 0.0;

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

    /* Correct the flux estimate at the sampling instant. The estimation error has
     * accumulated over the previous sampling period, whose length scales it. */
    double complex v_err = k_o1 * out->e_o + k_o2 * conj(out->e_o);
    double complex psi_s = out->psi_s + self->T_s_old * v_err;
    double complex psi_R = psi_s - par->L_sgm * out->i_s;

    /* Predict the voltage model and the current model to the next sampling instant.
     * The stator voltage is constant in stator coordinates, while the other terms
     * are constant in the observer coordinates. */
    double complex Phi, Gamma;
    hold_equivalent(T_s, out->w_c, &Phi, &Gamma);
    self->psi_s = Phi * (psi_s + T_s * out->u_s_zoh) - Gamma * par->R_s * out->i_s;
    hold_equivalent(T_s, out->w_c - out->w_m, &Phi, &Gamma);
    self->psi_R_pred =
        Phi * psi_R + Gamma * (par->R_R * out->i_s - im_alpha(par) * psi_R);
    self->theta_c = wrap(self->theta_c + T_s * out->w_c);

    /* Sampling period for the next sampling period */
    self->T_s_old = T_s;
}

/* IMSpeedFluxObserver ------------------------------------------------------ */

static void im_speed_flux_observer_init(IMSpeedFluxObserver *self,
                                        InductionMachineInvGammaPars par,
                                        double alpha_o, double T_s, int sensorless,
                                        double J)
{
    /* Observer gains for critically damped dynamics */
    alpha_o = matched_pole(alpha_o, T_s);
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

/* Speed and flux observer with the defaults of create_speed_flux_observer: alpha_o
 * NAN for None, and J NAN (or zero) for None */
static void im_create_speed_flux_observer(IMSpeedFluxObserver *self,
                                          InductionMachineInvGammaPars par,
                                          double alpha_o, double T_s, int sensorless,
                                          double J)
{
    J = isnan(J) ? 0.0 : J; /* Zero means None */
    if (isnan(alpha_o)) {
        /* With J, the double pole at alpha_o/2 keeps the speed gain about the same */
        alpha_o = sensorless ? 2.0 * M_PI * 60.0 : 2.0 * M_PI * 400.0;
        alpha_o = (J > 0.0) ? 0.5 * alpha_o : alpha_o;
    }
    im_speed_flux_observer_init(self, par, alpha_o, T_s, sensorless, J);
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
    if (isnan(k_fw)) {
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

/* CurrentController and DiscreteCurrentController -------------------------- */

static void im_current_ctrl_init(IMCurrentController *self,
                                 const InductionMachineInvGammaPars *par,
                                 double alpha_c, double alpha_i, int discrete,
                                 double T_s)
{
    self->L_sgm = par->L_sgm;
    self->discrete = discrete;
    double k_t = alpha_c * par->L_sgm;
    double k_i = alpha_c * alpha_i * par->L_sgm;
    double k_p = (alpha_c + alpha_i) * par->L_sgm;
    complex_pi_init(&self->pi, k_p, k_i, k_t);
    discrete_complex_init(&self->dc, alpha_c, T_s);
}

static double complex im_current_ctrl_compute_output(IMCurrentController *self,
                                                     double complex i_ref,
                                                     double complex i)
{
    if (self->discrete) {
        /* The currents mapped to the leakage flux linkages */
        return discrete_complex_compute_output(&self->dc, self->L_sgm * i_ref,
                                               self->L_sgm * i);
    }
    return complex_pi_compute_output(&self->pi, i_ref, i, 0.0);
}

static void im_current_ctrl_update(IMCurrentController *self, double T_s,
                                   double complex u, double w_c)
{
    if (self->discrete) {
        discrete_complex_update(&self->dc, T_s, u, w_c);
    } else {
        complex_pi_update(&self->pi, T_s, u, w_c);
    }
}

/* IMCurrentVectorController ------------------------------------------------ */

static void im_current_vector_ctrl_init(IMCurrentVectorController *self,
                                        InductionMachineInvGammaPars par,
                                        const IMCurrentVectorControllerCfg *cfg)
{
    /* Resolve the default, as in CurrentVectorController.__init__ */
    double alpha_i = isnan(cfg->alpha_i) ? cfg->alpha_c : cfg->alpha_i;
    self->T_s = cfg->T_s;
    self->sensorless = cfg->sensorless;
    im_reference_gen_init(&self->reference_gen, par, cfg->psi_s_nom, cfg->i_s_max,
                          cfg->w_s_nom, cfg->k_u, cfg->k_fw);
    im_current_ctrl_init(&self->current_ctrl, &par, cfg->alpha_c, alpha_i,
                         cfg->discrete, cfg->T_s);
    im_create_speed_flux_observer(&self->observer, par, cfg->alpha_o, cfg->T_s,
                                  cfg->sensorless, cfg->J);
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
    ref->u_s = im_current_ctrl_compute_output(&self->current_ctrl, ref->i_s, fbk->i_s);
}

static void im_current_vector_ctrl_update(IMCurrentVectorController *self,
                                          const IMReferences *ref,
                                          const IMObserverOutputs *fbk)
{
    im_speed_flux_observer_update(&self->observer, ref->T_s, fbk);
    im_reference_gen_update(&self->reference_gen, ref->T_s, ref->u_s, fbk->u_dc);
    /* The discrete-time controller takes its limited output, except with blocked
     * gates, when the open-circuit voltage of the observer is the realized one */
    double complex u_s =
        (self->current_ctrl.discrete && ref->enabled) ? ref->u_s_lim : fbk->u_s;
    im_current_ctrl_update(&self->current_ctrl, ref->T_s, u_s, fbk->w_c);
}

/* VectorControlSystem helpers --------------------------------------------- */

/* Feedback signals (DriveControlSystem.get_feedback). The observer uses the voltage
 * of the ongoing sampling period, while the control laws use the voltage averaged
 * over the previous and ongoing periods. In standby (with blocked gates, and at the
 * first sampling instant after the release of a sensorless drive,
 * VectorControlSystem._standby), the voltage is unknown (NAN), and the observer runs
 * on its open-circuit model, whose voltage is used for the control laws too. */
static void im_vector_control_feedback(const IMSpeedFluxObserver *observer,
                                       int sensorless, const PWM *pwm,
                                       const IMMeasurements *meas, int standby,
                                       IMObserverOutputs *fbk)
{
    double complex u_c_ab_zoh = NAN;
    double complex u_c_ab = NAN;
    if (!standby) {
        u_c_ab_zoh = pwm_realized_voltage(pwm, meas->i_c_ab, meas->u_dc, 0);
        u_c_ab = pwm_realized_voltage(pwm, meas->i_c_ab, meas->u_dc, 1);
    }
    if (sensorless) {
        im_speed_flux_observer_compute_output(observer, u_c_ab_zoh, meas->i_c_ab, 0.0,
                                              0.0, fbk);
    } else {
        /* Speed error from the measured rotor speed (compute_feedback) */
        double eps = meas->w_M - observer->speed_observer.w_M;
        im_speed_flux_observer_compute_output(observer, u_c_ab_zoh, meas->i_c_ab, eps,
                                              1.0, fbk);
    }
    fbk->u_s = standby ? fbk->u_s_zoh : cexp(-I * fbk->theta_c) * u_c_ab;
    fbk->u_dc = meas->u_dc;
}

/* Duty ratios for the PWM (modulate), the angle of the voltage reference u_s_ref
 * advanced by k_comp*T_s*w_c, or the zero vector (zero_vector nonzero). Returns the
 * limited voltage reference in the controller coordinates (u_s_lim). */
static double complex im_vector_control_modulate(PWM *pwm, double k_comp,
                                                 int zero_vector, double T_s,
                                                 double complex u_s_ref,
                                                 const IMMeasurements *meas,
                                                 const IMObserverOutputs *fbk,
                                                 double d_abc[3])
{
    double complex u_s_ab_ref = cexp(I * fbk->theta_c) * u_s_ref;
    u_s_ab_ref *= cexp(I * k_comp * T_s * fbk->w_c);
    if (zero_vector) {
        u_s_ab_ref = 0.0;
    }
    double complex u_lim_ab;
    pwm_compute_output(pwm, T_s, u_s_ab_ref, fbk->u_dc, fbk->w_c, meas->i_c_ab, d_abc,
                       &u_lim_ab);
    return cexp(-I * fbk->theta_c) * u_lim_ab;
}

/* IMVectorControlSystem ---------------------------------------------------- */

static void im_vector_control_system_init(IMVectorControlSystem *self,
                                          InductionMachineInvGammaPars par,
                                          const IMCurrentVectorControllerCfg *cfg,
                                          PIController speed_ctrl)
{
    pwm_init(&self->pwm, 1.5);
    /* k_comp of CurrentVectorController: the discrete-time current controller
     * compensates for the delays itself */
    self->k_comp = cfg->discrete ? 0.0 : 1.5;
    im_current_vector_ctrl_init(&self->vector_ctrl, par, cfg);
    self->speed_ctrl = speed_ctrl;
    self->speed_ctrl.u_i = 0.0;
    self->speed_ctrl.v = 0.0;
    self->enabled = 1;
    self->release_step = 0;
}

static void im_vector_control_system_compute_output(IMVectorControlSystem *self,
                                                    const IMMeasurements *meas,
                                                    double w_M_ref, int enabled)
{
    IMObserverOutputs *fbk = &self->fbk;
    IMReferences *ref = &self->ref;
    self->enabled = enabled;
    int standby = !enabled || self->release_step;
    int sensorless = self->vector_ctrl.sensorless;
    im_vector_control_feedback(&self->vector_ctrl.observer, sensorless, &self->pwm,
                               meas, standby, fbk);

    /* Speed controller and vector controller, or in standby, the vector controller
     * with zero torque reference and the zero vector of a sensorless drive */
    double tau_M_ref = 0.0;
    if (!standby) {
        tau_M_ref = pi_compute_output(&self->speed_ctrl, w_M_ref, fbk->w_M, 0.0);
    }
    im_current_vector_ctrl_compute_output(&self->vector_ctrl, tau_M_ref, fbk, ref);
    ref->u_s_lim = im_vector_control_modulate(&self->pwm, self->k_comp,
                                              sensorless && standby, ref->T_s,
                                              ref->u_s, meas, fbk, ref->d_abc);
    ref->w_M = w_M_ref;
    ref->enabled = enabled;
}

static void im_vector_control_system_update(IMVectorControlSystem *self)
{
    pwm_update(&self->pwm, self->ref.T_s, self->ref.d_abc);
    im_current_vector_ctrl_update(&self->vector_ctrl, &self->ref, &self->fbk);
    if (self->enabled && !self->release_step) {
        pi_update(&self->speed_ctrl, self->ref.T_s, self->ref.tau_M);
    }
    self->release_step = self->vector_ctrl.sensorless && !self->enabled;
}

/* Signal vectors of the modular control system ---------------------------- */

#define COMPLEX_AT(u, i) complex_from((u)[i], (u)[(i) + 1])
#define SET_COMPLEX(y, i, z)                                                       \
    do {                                                                           \
        (y)[i] = creal(z);                                                         \
        (y)[(i) + 1] = cimag(z);                                                   \
    } while (0)

static void im_measurements_pack(const IMMeasurements *meas, double y[MEAS_WIDTH])
{
    SET_COMPLEX(y, MEAS_i_c_ab, meas->i_c_ab);
    y[MEAS_u_dc] = meas->u_dc;
    y[MEAS_w_M] = meas->w_M;
}

static void im_measurements_unpack(const double u[MEAS_WIDTH], IMMeasurements *meas)
{
    meas->i_c_ab = COMPLEX_AT(u, MEAS_i_c_ab);
    meas->u_dc = u[MEAS_u_dc];
    meas->w_M = u[MEAS_w_M];
}

static void im_observer_outputs_pack(const IMObserverOutputs *fbk, double y[FBK_WIDTH])
{
    SET_COMPLEX(y, FBK_i_s, fbk->i_s);
    SET_COMPLEX(y, FBK_u_s, fbk->u_s);
    SET_COMPLEX(y, FBK_u_s_zoh, fbk->u_s_zoh);
    SET_COMPLEX(y, FBK_psi_s, fbk->psi_s);
    SET_COMPLEX(y, FBK_psi_R, fbk->psi_R);
    y[FBK_tau_M] = fbk->tau_M;
    y[FBK_w_c] = fbk->w_c;
    y[FBK_w_s] = fbk->w_s;
    y[FBK_w_r] = fbk->w_r;
    y[FBK_w_m] = fbk->w_m;
    y[FBK_w_M] = fbk->w_M;
    y[FBK_theta_c] = fbk->theta_c;
    SET_COMPLEX(y, FBK_e_o, fbk->e_o);
    y[FBK_eps] = fbk->eps;
    y[FBK_h] = fbk->h;
}

static void im_observer_outputs_unpack(const double u[FBK_WIDTH],
                                       IMObserverOutputs *fbk)
{
    fbk->u_dc = 0.0;
    fbk->i_s = COMPLEX_AT(u, FBK_i_s);
    fbk->u_s = COMPLEX_AT(u, FBK_u_s);
    fbk->u_s_zoh = COMPLEX_AT(u, FBK_u_s_zoh);
    fbk->psi_s = COMPLEX_AT(u, FBK_psi_s);
    fbk->psi_R = COMPLEX_AT(u, FBK_psi_R);
    fbk->tau_M = u[FBK_tau_M];
    fbk->tau_L = 0.0;
    fbk->w_c = u[FBK_w_c];
    fbk->w_s = u[FBK_w_s];
    fbk->w_r = u[FBK_w_r];
    fbk->w_m = u[FBK_w_m];
    fbk->w_M = u[FBK_w_M];
    fbk->theta_c = u[FBK_theta_c];
    fbk->e_o = COMPLEX_AT(u, FBK_e_o);
    fbk->eps = u[FBK_eps];
    fbk->h = u[FBK_h];
}

static void im_references_unpack(const double u[REF_WIDTH], IMReferences *ref)
{
    IMReferences zero = {0};
    *ref = zero;
    ref->i_s = COMPLEX_AT(u, REF_i_s);
    ref->tau_M = u[REF_tau_M];
    ref->u_s = COMPLEX_AT(u, REF_u_s);
}

#undef COMPLEX_AT
#undef SET_COMPLEX
