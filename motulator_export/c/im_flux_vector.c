/*
 * Observer-based V/Hz control of induction machine drives, ported from motulator. See
 * im_flux_vector.h.
 */

#include "im_flux_vector.h"

/* Minimum as the built-in min of Python: the first argument unless the second one is
 * smaller, so that a NaN argument is skipped unless it is the first one */
static double py_min(double a, double b)
{
    return (b < a) ? b : a;
}

/* Configuration ------------------------------------------------------------ */

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
                                     double alpha_psi, double alpha_tau)
{
    self->par = par;
    self->alpha_psi = alpha_psi;
    self->alpha_tau = alpha_tau;
}

static double complex im_flux_torque_ctrl_compute_output(
    const IMFluxTorqueController *self, double psi_s_ref, double tau_M_ref,
    const IMObserverOutputs *fbk)
{
    const InductionMachineInvGammaPars *par = &self->par;

    /* Auxiliary current, torque-production factor, and directions */
    double complex t_psi = 1.0, t_tau = 0.0;
    if (par->L_sgm > 0.0) { /* L_sgm = 0 in open-loop V/Hz control */
        double complex i_a = fbk->psi_R / par->L_sgm;
        double c_tau = 1.5 * par->n_p * creal(i_a * conj(fbk->psi_s));
        if (c_tau > 0.0) {
            t_psi = 1.5 * par->n_p * cabs(fbk->psi_s) * i_a / c_tau;
            t_tau = I * fbk->psi_s / c_tau;
        }
    }

    /* Error signals (the integral action is not used: alpha_i = 0) */
    double e_psi = psi_s_ref - cabs(fbk->psi_s);
    double e_tau = tau_M_ref - fbk->tau_M;
    double complex e_u =
        self->alpha_psi * e_psi * t_psi + self->alpha_tau * e_tau * t_tau;

    /* Voltage reference */
    double complex v = par->R_s * fbk->i_s + I * fbk->w_s * fbk->psi_s;
    return v + e_u;
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
        /* Current limit */
        double i_m = psi_R / par->L_M;
        double tau_M_cl = 0.0;
        if (psi_R < par->L_M * self->i_s_max) {
            double i_q_max = sqrt(self->i_s_max * self->i_s_max - i_m * i_m);
            tau_M_cl = 1.5 * par->n_p * psi_R * i_q_max;
        }
        /* Limited torque reference */
        double tau =
            py_min(py_min(py_min(fabs(tau_M_ref), self->tau_M_max), tau_b), tau_M_cl);
        tau_M_ref = tau * sign(tau_M_ref);
    }
    *psi_s_ref = psi;
    *tau_M_ref_lim = tau_M_ref;
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
                             cfg->alpha_tau);
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
    pwm_set_overmodulation(&self->pwm, 1); /* pwm_mode = "MME" */
    im_vhz_ctrl_init(&self->vhz_ctrl, par, cfg);
    rate_limiter_init(&self->rate_limiter, slew_rate);
}

static void im_vhz_control_system_compute_output(IMVHzControlSystem *self,
                                                 const IMVHzMeasurements *meas,
                                                 double w_M_ref)
{
    IMVHzController *ctrl = &self->vhz_ctrl;
    IMObserverOutputs *fbk = &self->fbk;
    IMVHzReferences *ref = &self->ref;
    double T_s = ctrl->T_s;

    /* Feedback signals */
    double complex u_c_ab =
        pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 1);

    /* Rate-limited speed reference (RateLimiter), stored in the update function */
    w_M_ref = rate_limiter_compute_output(&self->rate_limiter, T_s, w_M_ref);

    /* The rotor speed of the observer is the speed reference */
    im_flux_observer_compute_output(&ctrl->observer, u_c_ab, meas->i_c_ab, w_M_ref, 0.0,
                                    ctrl->h, fbk);
    fbk->u_dc = meas->u_dc;

    /* References */
    ref->T_s = T_s;
    im_reference_gen_flux_and_torque(&ctrl->reference_gen, ctrl->tau_M_lpf, fbk->w_s,
                                     cabs(fbk->psi_R), fbk->u_dc, &ref->psi_s,
                                     &ref->tau_M);
    ref->u_s = im_flux_torque_ctrl_compute_output(&ctrl->flux_torque_ctrl, ref->psi_s,
                                                  ref->tau_M, fbk);

    /* Duty ratios for the PWM */
    double complex u_s_ab_ref = cexp(I * fbk->theta_c) * ref->u_s;
    ref->u_c_ab = pwm_compute_output(&self->pwm, T_s, u_s_ab_ref, fbk->u_dc, fbk->w_c,
                                     meas->i_c_ab, ref->d_abc);
    ref->w_M = w_M_ref;
}

static void im_vhz_control_system_update(IMVHzControlSystem *self)
{
    IMVHzController *ctrl = &self->vhz_ctrl;
    double T_s = self->ref.T_s;
    pwm_update(&self->pwm, self->ref.d_abc);
    rate_limiter_update(&self->rate_limiter, self->ref.w_M);
    ctrl->tau_M_lpf += T_s * ctrl->alpha_f * (self->fbk.tau_M - ctrl->tau_M_lpf);
    im_flux_observer_update(&ctrl->observer, T_s, &self->fbk);
}

static void im_vhz_control_system_set_resistances(IMVHzControlSystem *self, double R_s,
                                                  double R_R)
{
    IMVHzController *ctrl = &self->vhz_ctrl;
    ctrl->reference_gen.par.R_s = R_s;
    ctrl->reference_gen.par.R_R = R_R;
    ctrl->flux_torque_ctrl.par.R_s = R_s;
    ctrl->flux_torque_ctrl.par.R_R = R_R;
    ctrl->observer.par.R_s = R_s;
    ctrl->observer.par.R_R = R_R;
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
