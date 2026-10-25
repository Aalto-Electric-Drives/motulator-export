/*
 * Flux-vector control and observer-based V/Hz control of induction machine drives,
 * ported from motulator. See im_flux_vector.h.
 */

#include "im_flux_vector.h"

/* Minimum as the built-in min of Python: the first argument unless the second one is
 * smaller, so that a NaN argument is skipped unless it is the first one */
static double py_min(double a, double b)
{
    return (b < a) ? b : a;
}

/* Maximum as the built-in max of Python, see py_min */
static double py_max(double a, double b)
{
    return (b > a) ? b : a;
}

/* Configuration ------------------------------------------------------------ */

static IMFluxVectorControllerCfg im_flux_vector_controller_cfg(double psi_s_nom,
                                                               double i_s_max)
{
    IMFluxVectorControllerCfg cfg;
    cfg.psi_s_nom = psi_s_nom;
    cfg.i_s_max = i_s_max;
    cfg.alpha_tau = 2.0 * M_PI * 100.0;
    cfg.alpha_psi = NAN;
    cfg.alpha_i = NAN;
    cfg.alpha_o = NAN;
    cfg.tau_M_max = INFINITY;
    cfg.k_u = 0.9;
    cfg.k_b = 0.9;
    cfg.J = NAN;
    cfg.sensorless = 1;
    cfg.T_s = 125e-6;
    return cfg;
}

static IMVHzControllerCfg im_vhz_controller_cfg(double psi_s_nom, double i_s_max)
{
    IMVHzControllerCfg cfg;
    cfg.psi_s_nom = psi_s_nom;
    cfg.i_s_max = i_s_max;
    cfg.alpha_psi = 2.0 * M_PI * 100.0;
    cfg.alpha_tau = 2.0 * M_PI * 20.0;
    cfg.alpha_f = 2.0 * M_PI * 1.0;
    cfg.k_u = 0.9;
    cfg.k_b = 0.9;
    cfg.T_s = 250e-6;
    return cfg;
}

/* IMFluxTorqueController ---------------------------------------------------- */

static void im_flux_torque_ctrl_init(IMFluxTorqueController *self,
                                     InductionMachineInvGammaPars par,
                                     double alpha_psi, double alpha_tau,
                                     double alpha_i)
{
    self->par = par;
    self->alpha_psi = alpha_psi;
    self->alpha_tau = alpha_tau;
    self->alpha_i = alpha_i;
    self->x_psi = 0.0;
    self->x_tau = 0.0;
    self->i_a = 0.0;
    self->u_ff = 0.0;
}

static double complex im_flux_torque_ctrl_compute_output(IMFluxTorqueController *self,
                                                         double psi_s_ref,
                                                         double tau_M_ref,
                                                         const IMObserverOutputs *fbk)
{
    const InductionMachineInvGammaPars *par = &self->par;
    double psi_s_abs = cabs(fbk->psi_s);

    /* Auxiliary current (L_sgm = 0 in open-loop V/Hz control, where only the flux is
     * controlled) */
    double complex i_a = (par->L_sgm > 0.0) ? fbk->psi_R / par->L_sgm : 0.0;

    /* Torque-production factor and directions */
    double c_tau = 1.5 * par->n_p * creal(i_a * conj(fbk->psi_s));
    double complex t_psi = (c_tau > 0.0) ? 1.5 * par->n_p * psi_s_abs * i_a / c_tau : 1.0;
    double complex t_tau = (c_tau > 0.0) ? I * fbk->psi_s / c_tau : 0.0;

    /* Error signals */
    double e_psi = psi_s_ref - psi_s_abs;
    double e_tau = tau_M_ref - fbk->tau_M;
    double complex e_u = self->alpha_psi * e_psi * t_psi + self->alpha_tau * e_tau * t_tau;
    double complex u_i = self->x_psi * t_psi + self->x_tau * t_tau;
    double complex e_v = u_i - self->alpha_i * (psi_s_abs * t_psi + fbk->tau_M * t_tau);

    /* Voltage reference, consisting of the feedforward and integral action (u_ff)
     * and the proportional action (e_u) */
    double complex u_ff = par->R_s * fbk->i_s + I * fbk->w_s * fbk->psi_s + e_v;

    /* Workspace variables for the update function */
    self->i_a = i_a;
    self->u_ff = u_ff;

    return u_ff + e_u;
}

static void im_flux_torque_ctrl_update(IMFluxTorqueController *self, double T_s,
                                       const IMObserverOutputs *fbk)
{
    const InductionMachineInvGammaPars *par = &self->par;
    double psi_s_abs = cabs(fbk->psi_s);
    /* Error signal and gains */
    double complex u_err = fbk->u_s - self->u_ff;
    double k_psi = (psi_s_abs > 0.0) ? self->alpha_i / psi_s_abs : 0.0;
    double k_tau = 1.5 * par->n_p * self->alpha_i;
    /* Update the integral states */
    self->x_psi += T_s * k_psi * creal(fbk->psi_s * conj(u_err));
    self->x_tau += T_s * k_tau * creal(I * self->i_a * conj(u_err));
}

/* IMReferenceGenerator ------------------------------------------------------ */

static void im_flux_reference_gen_init(IMReferenceGenerator *self,
                                       InductionMachineInvGammaPars par,
                                       double psi_s_nom, double i_s_max,
                                       double tau_M_max, double k_u, double k_b)
{
    self->par = par;
    self->psi_s_nom = psi_s_nom;
    self->i_s_max = i_s_max;
    self->tau_M_max = tau_M_max;
    self->k_u = k_u;
    self->k_b = k_b;
}

static void im_reference_gen_flux_and_torque(const IMReferenceGenerator *self,
                                             double tau_M_ref, double w_s,
                                             double psi_R, double u_dc,
                                             double *psi_s_ref, double *tau_M_ref_lim)
{
    const InductionMachineInvGammaPars *par = &self->par;

    /* Field weakening */
    double u_s_max = self->k_u * u_dc / sqrt(3.0);
    double psi_s_max = (w_s != 0.0) ? u_s_max / fabs(w_s) : INFINITY;
    /* Current limit */
    double psi_s_cl = par->L_sgm * self->i_s_max + psi_R;
    /* Stator flux reference */
    double psi = py_min(py_min(psi_s_max, self->psi_s_nom), psi_s_cl);

    /* Torque reference limiting */
    if (par->L_sgm > 0.0) { /* L_sgm = 0 in open-loop V/Hz control */
        /* Breakdown torque limit */
        double k = self->k_b * par->L_M / (par->L_M + par->L_sgm);
        double tau_b = 0.75 * par->n_p * k * (psi * psi) / par->L_sgm;
        /* Current limit: the stator flux on the circle psi stays within the circle
         * L_sgm*i_s_max around the rotor flux, which gives the maximum q-component
         * of the stator flux in rotor flux coordinates */
        double tau_M_cl = 0.0;
        if (psi_R > 0.0) {
            double psi_cl = par->L_sgm * self->i_s_max;
            double psi_sd =
                (psi * psi + psi_R * psi_R - psi_cl * psi_cl) / (2.0 * psi_R);
            psi_sd = py_max(psi_sd, 0.0);
            double psi_sq = sqrt(py_max(psi * psi - psi_sd * psi_sd, 0.0));
            tau_M_cl = 1.5 * par->n_p * psi_R * psi_sq / par->L_sgm;
        }
        /* Limited torque reference */
        double tau =
            py_min(py_min(py_min(fabs(tau_M_ref), self->tau_M_max), tau_b), tau_M_cl);
        tau_M_ref = tau * sign(tau_M_ref);
    }
    *psi_s_ref = psi;
    *tau_M_ref_lim = tau_M_ref;
}

/* IMFluxVectorController ----------------------------------------------------- */

static void im_flux_vector_ctrl_init(IMFluxVectorController *self,
                                     InductionMachineInvGammaPars par,
                                     const IMFluxVectorControllerCfg *cfg)
{
    /* Resolve the defaults, as in FluxVectorController.__init__ */
    double alpha_psi = isnan(cfg->alpha_psi) ? cfg->alpha_tau : cfg->alpha_psi;
    double alpha_i = isnan(cfg->alpha_i) ? cfg->alpha_tau : cfg->alpha_i;
    self->T_s = cfg->T_s;
    self->sensorless = cfg->sensorless;
    im_flux_reference_gen_init(&self->reference_gen, par, cfg->psi_s_nom, cfg->i_s_max,
                               cfg->tau_M_max, cfg->k_u, cfg->k_b);
    im_flux_torque_ctrl_init(&self->flux_torque_ctrl, par, alpha_psi, cfg->alpha_tau,
                             alpha_i);
    im_create_speed_flux_observer(&self->observer, par, cfg->alpha_o, cfg->T_s,
                                  cfg->sensorless, cfg->J);
}

static void im_flux_vector_ctrl_compute_output(IMFluxVectorController *self,
                                               double tau_M_ref,
                                               const IMObserverOutputs *fbk,
                                               IMVHzReferences *ref)
{
    ref->T_s = self->T_s;
    im_reference_gen_flux_and_torque(&self->reference_gen, tau_M_ref, fbk->w_s,
                                     cabs(fbk->psi_R), fbk->u_dc, &ref->psi_s,
                                     &ref->tau_M);
    ref->u_s = im_flux_torque_ctrl_compute_output(&self->flux_torque_ctrl, ref->psi_s,
                                                  ref->tau_M, fbk);
}

static void im_flux_vector_ctrl_update(IMFluxVectorController *self,
                                       const IMVHzReferences *ref,
                                       const IMObserverOutputs *fbk)
{
    im_speed_flux_observer_update(&self->observer, ref->T_s, fbk);
    im_flux_torque_ctrl_update(&self->flux_torque_ctrl, ref->T_s, fbk);
}

/* IMFVCControlSystem --------------------------------------------------------- */

static void im_fvc_control_system_init(IMFVCControlSystem *self,
                                       InductionMachineInvGammaPars par,
                                       const IMFluxVectorControllerCfg *cfg,
                                       PIController speed_ctrl)
{
    pwm_init(&self->pwm, 1.5);
    self->k_comp = 1.5; /* k_comp of FluxVectorController */
    im_flux_vector_ctrl_init(&self->vector_ctrl, par, cfg);
    self->speed_ctrl = speed_ctrl;
    self->speed_ctrl.u_i = 0.0;
    self->speed_ctrl.v = 0.0;
    self->enabled = 1;
    self->release_step = 0;
    self->torque_mode = 0;
}

static void im_fvc_control_system_compute_output(IMFVCControlSystem *self,
                                                 const IMMeasurements *meas,
                                                 double ext_ref, int enabled)
{
    IMObserverOutputs *fbk = &self->fbk;
    IMVHzReferences *ref = &self->ref;
    self->enabled = enabled;
    int standby = !enabled || self->release_step;
    int sensorless = self->vector_ctrl.sensorless;
    im_vector_control_feedback(&self->vector_ctrl.observer, sensorless, &self->pwm,
                               meas, standby, fbk);

    /* Speed controller (or in torque-control mode, the torque reference) and
     * vector controller, or in standby, the vector controller with zero torque
     * reference and the zero vector of a sensorless drive */
    double tau_M_ref = 0.0;
    if (!standby && self->torque_mode) {
        tau_M_ref = ext_ref;
    } else if (!standby) {
        tau_M_ref = pi_compute_output(&self->speed_ctrl, ext_ref, fbk->w_M, 0.0);
    }
    im_flux_vector_ctrl_compute_output(&self->vector_ctrl, tau_M_ref, fbk, ref);
    ref->u_s_lim = im_vector_control_modulate(&self->pwm, self->k_comp,
                                              sensorless && standby, ref->T_s,
                                              ref->u_s, meas, fbk, ref->d_abc);
    ref->w_M = self->torque_mode ? NAN : ext_ref; /* None in torque mode */
}

static void im_fvc_control_system_update(IMFVCControlSystem *self)
{
    pwm_update(&self->pwm, self->ref.T_s, self->ref.d_abc);
    im_flux_vector_ctrl_update(&self->vector_ctrl, &self->ref, &self->fbk);
    if (!self->torque_mode && self->enabled && !self->release_step) {
        pi_update(&self->speed_ctrl, self->ref.T_s, self->ref.tau_M);
    }
    self->release_step = self->vector_ctrl.sensorless && !self->enabled;
}

/* IMVHzController ------------------------------------------------------------ */

static void im_vhz_ctrl_init(IMVHzController *self, InductionMachineInvGammaPars par,
                             const IMVHzControllerCfg *cfg)
{
    self->T_s = cfg->T_s;
    self->alpha_f = cfg->alpha_f;
    self->tau_M_lpf = 0.0;
    self->h = 0.0;
    IMReferenceGenerator *gen = &self->reference_gen;
    im_flux_reference_gen_init(gen, par, cfg->psi_s_nom, cfg->i_s_max, INFINITY,
                               cfg->k_u, cfg->k_b);
    im_flux_torque_ctrl_init(&self->flux_torque_ctrl, par, cfg->alpha_psi,
                             cfg->alpha_tau, 0.0);
    /* Sensorless observer with the default gain of create_vhz_observer */
    im_flux_observer_init(&self->observer, par, 1);
    if (isinf(par.L_M) && par.L_M > 0.0) {
        /* Pure open-loop V/Hz control: unit observer gain with h = 1, which disables
         * the model-based correction */
        self->observer.unit_gain = 1;
        self->observer.psi_s = cfg->psi_s_nom;
        gen->k_u = INFINITY;
        self->h = 1.0;
    }
}

/* IMVHzControlSystem ----------------------------------------------------------- */

static void im_vhz_control_system_init(IMVHzControlSystem *self,
                                       InductionMachineInvGammaPars par,
                                       const IMVHzControllerCfg *cfg, double slew_rate)
{
    pwm_init(&self->pwm, 1.5);
    pwm_set_overmodulation(&self->pwm, 1); /* Default of VHzControlSystem: MME */
    self->k_comp = 1.5; /* k_comp of ObserverBasedVHzController */
    im_vhz_ctrl_init(&self->vhz_ctrl, par, cfg);
    rate_limiter_init(&self->rate_limiter, slew_rate);
}

static void im_vhz_control_system_compute_output(IMVHzControlSystem *self,
                                                 const IMVHzMeasurements *meas,
                                                 double w_M_ref, int enabled)
{
    IMVHzController *ctrl = &self->vhz_ctrl;
    IMObserverOutputs *fbk = &self->fbk;
    IMVHzReferences *ref = &self->ref;
    double T_s = ctrl->T_s;

    /* Feedback signals (DriveControlSystem.get_feedback). The observer uses the
     * voltage of the ongoing sampling period, while the control laws use the voltage
     * averaged over the previous and ongoing periods. With blocked gates, the
     * voltage is unknown (NAN), and the observer runs on its open-circuit model,
     * whose voltage is used for the control laws too. */
    double complex u_c_ab_zoh = NAN;
    double complex u_c_ab = NAN;
    if (enabled) {
        u_c_ab_zoh = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 0);
        u_c_ab = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 1);
    }

    /* Rate-limited speed reference (RateLimiter), stored in the update function */
    w_M_ref = rate_limiter_compute_output(&self->rate_limiter, T_s, w_M_ref);

    /* The rotor speed of the observer is the speed reference */
    im_flux_observer_compute_output(&ctrl->observer, u_c_ab_zoh, meas->i_c_ab, w_M_ref,
                                    0.0, ctrl->h, fbk);
    fbk->u_s = enabled ? cexp(-I * fbk->theta_c) * u_c_ab : fbk->u_s_zoh;
    fbk->u_dc = meas->u_dc;

    /* References, or with blocked gates, the standby references: the flux and
     * torque controller tracks the estimated flux and torque */
    ref->T_s = T_s;
    if (enabled) {
        im_reference_gen_flux_and_torque(&ctrl->reference_gen, ctrl->tau_M_lpf,
                                         fbk->w_s, cabs(fbk->psi_R), fbk->u_dc,
                                         &ref->psi_s, &ref->tau_M);
    } else {
        ref->tau_M = fbk->tau_M;
        ref->psi_s = cabs(fbk->psi_s);
    }
    ref->u_s = im_flux_torque_ctrl_compute_output(&ctrl->flux_torque_ctrl, ref->psi_s,
                                                  ref->tau_M, fbk);

    /* Duty ratios for the PWM, the angle of the voltage reference advanced by
     * k_comp*T_s*w_c */
    double complex u_s_ab_ref = cexp(I * fbk->theta_c) * ref->u_s;
    u_s_ab_ref *= cexp(I * self->k_comp * T_s * fbk->w_c);
    double complex u_lim_ab;
    pwm_compute_output(&self->pwm, T_s, u_s_ab_ref, fbk->u_dc, fbk->w_c, meas->i_c_ab,
                       ref->d_abc, &u_lim_ab);
    ref->u_s_lim = cexp(-I * fbk->theta_c) * u_lim_ab;
    ref->w_M = w_M_ref;
}

static void im_vhz_control_system_update(IMVHzControlSystem *self)
{
    IMVHzController *ctrl = &self->vhz_ctrl;
    double T_s = self->ref.T_s;
    pwm_update(&self->pwm, T_s, self->ref.d_abc);
    rate_limiter_update(&self->rate_limiter, self->ref.w_M);
    ctrl->tau_M_lpf += T_s * ctrl->alpha_f * (self->fbk.tau_M - ctrl->tau_M_lpf);
    im_flux_observer_update(&ctrl->observer, T_s, &self->fbk);
}

/* Signal vectors of the control system ------------------------------------ */

static void im_vhz_measurements_pack(const IMVHzMeasurements *meas,
                                     double y[VHZ_MEAS_WIDTH])
{
    y[VHZ_MEAS_i_c_ab] = creal(meas->i_c_ab);
    y[VHZ_MEAS_i_c_ab + 1] = cimag(meas->i_c_ab);
    y[VHZ_MEAS_u_dc] = meas->u_dc;
}

static void im_vhz_measurements_unpack(const double u[VHZ_MEAS_WIDTH],
                                       IMVHzMeasurements *meas)
{
    meas->i_c_ab = complex_from(u[VHZ_MEAS_i_c_ab], u[VHZ_MEAS_i_c_ab + 1]);
    meas->u_dc = u[VHZ_MEAS_u_dc];
}

static void im_vhz_references_unpack(const double u[VHZ_REF_WIDTH],
                                     IMVHzReferences *ref)
{
    IMVHzReferences zero = {0};
    *ref = zero;
    ref->psi_s = u[VHZ_REF_psi_s];
    ref->tau_M = u[VHZ_REF_tau_M];
    ref->u_s = complex_from(u[VHZ_REF_u_s], u[VHZ_REF_u_s + 1]);
}
