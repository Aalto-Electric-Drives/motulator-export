/*
 * Sensorless flux-vector control of synchronous machine drives, ported from
 * motulator. See sm_flux_vector.h.
 */

#include "sm_flux_vector.h"

/* Configuration ------------------------------------------------------------ */

static FluxVectorControllerCfg flux_vector_controller_cfg(double i_s_max)
{
    FluxVectorControllerCfg cfg;
    cfg.i_s_max = i_s_max;
    cfg.alpha_tau = 2.0 * M_PI * 100.0;
    cfg.alpha_psi = NAN;
    cfg.alpha_i = NAN;
    cfg.alpha_o = NAN;
    cfg.k_o[0] = NAN;
    cfg.k_o[1] = NAN;
    cfg.psi_s_min = NAN;
    cfg.psi_s_max = INFINITY;
    cfg.k_u = 0.9;
    cfg.k_mtpv = 0.85;
    cfg.J = NAN;
    cfg.T_s = 125e-6;
    cfg.sensorless = 1;
    return cfg;
}

/* FluxObserver ------------------------------------------------------------- */

static void flux_observer_init(FluxObserver *self, SynchronousMachinePars par,
                               double k_theta, const double k_o[2])
{
    self->par = par;
    self->k_theta = k_theta;
    self->k_o[0] = k_o[0];
    self->k_o[1] = k_o[1];
    self->theta_m = 0.0;
    self->psi_s = par.psi_f;
    self->T_s_prev = 0.0;
}

static double complex flux_observer_aux_flux(const FluxObserver *self,
                                             double complex i_s,
                                             double complex psi_s)
{
    IncrIndMat L = incr_ind_mat(&self->par, i_s);
    return aux_flux(&L, i_s, psi_s);
}

static void flux_observer_compute_output(const FluxObserver *self,
                                         double complex u_s_ab,
                                         double complex i_s_ab, double w_M,
                                         double eps_ext, double h,
                                         ObserverOutputs *out)
{
    /* The weight h of the external position error signal eps_ext selects between
     * the sensorless (h = 0) and sensored (h = 1) modes */
    const SynchronousMachinePars *par = &self->par;
    out->psi_s = self->psi_s;
    out->psi_f = par->psi_f;
    out->h = h;

    /* Rotor speed */
    out->w_M = w_M;
    out->w_m = par->n_p * w_M;

    /* Coordinate system angle equals the estimated rotor angle */
    out->theta_c = out->theta_m = self->theta_m;

    /* Current vector in (estimated) rotor coordinates. The voltage averaged over
     * the previous and ongoing sampling periods is set by the control system
     * (DriveControlSystem.get_feedback). */
    out->i_s = cexp(-I * out->theta_c) * i_s_ab;
    out->u_s = 0.0;

    /* Flux estimation error */
    double complex psi_s_model = psi_s_dq(par, out->i_s);
    out->e_o = psi_s_model - out->psi_s;

    /* Auxiliary flux */
    out->psi_a = flux_observer_aux_flux(self, out->i_s, psi_s_model);

    /* Error signals for the rotor angle and PM-flux estimation */
    double complex ratio = (out->psi_a != 0.0) ? out->e_o / out->psi_a : 0.0;
    out->eps = h * eps_ext - (1.0 - h) * cimag(ratio) / par->n_p;
    out->eps_f = -(1.0 - h) * creal(ratio);

    /* Angular speed of the coordinate system */
    out->w_c = out->w_m + self->k_theta * par->n_p * out->eps;

    /* Voltage vector in the observer coordinates, or the open-circuit voltage of
     * the model if the voltage is unknown (NAN, blocked gates): the back-EMF of the
     * flux estimate at the rotor speed, averaged over the sampling period (that of
     * the previous period), and the resistive voltage drop */
    if (!isnan(creal(u_s_ab))) {
        out->u_s_zoh = cexp(-I * out->theta_c) * u_s_ab;
    } else if (self->T_s_prev > 0.0) {
        double complex emf =
            (cexp(I * out->w_m * self->T_s_prev) - 1.0) / self->T_s_prev * out->psi_s;
        out->u_s_zoh = emf + par->R_s * out->i_s;
    } else {
        out->u_s_zoh = I * out->w_m * out->psi_s + par->R_s * out->i_s;
    }

    /* Torque estimate */
    out->tau_M = 1.5 * par->n_p * cimag(out->i_s * conj(out->psi_s));
}

static void flux_observer_update(FluxObserver *self, double T_s,
                                 const ObserverOutputs *out)
{
    const SynchronousMachinePars *par = &self->par;

    double k_o1 = self->k_o[0] + self->k_o[1] * fabs(out->w_m);
    double complex k_o2;
    if (out->psi_a != 0.0) {
        k_o2 = (1.0 - out->h) * (k_o1 * out->psi_a / conj(out->psi_a));
    } else {
        k_o2 = (1.0 - out->h) * k_o1;
    }

    /* Correct the flux estimate at the sampling instant and predict it to the next
     * sampling instant. The stator voltage is constant in stator coordinates, while
     * the resistive voltage drop is constant in the observer coordinates. The PM-flux
     * estimation gain k_f is zero. */
    double complex v_err = k_o1 * out->e_o + k_o2 * conj(out->e_o);
    double complex psi_s = out->psi_s + T_s * v_err;
    double complex Phi, Gamma;
    hold_equivalent(T_s, out->w_c, &Phi, &Gamma);
    self->psi_s = Phi * (psi_s + T_s * out->u_s_zoh) - Gamma * par->R_s * out->i_s;
    self->theta_m = wrap(self->theta_m + T_s * out->w_c);
    self->T_s_prev = T_s;
}

/* SpeedFluxObserver -------------------------------------------------------- */

static void speed_flux_observer_init(SpeedFluxObserver *self,
                                     SynchronousMachinePars par, double alpha_o,
                                     double T_s, const double k_o[2], double J)
{
    /* Observer gains for critically damped dynamics */
    alpha_o = matched_pole(alpha_o, T_s);
    double k_theta, k_w, k_tau;
    if (J <= 0.0) {
        k_theta = 2.0 * alpha_o;
        k_w = pow(alpha_o, 2.0);
        k_tau = 0.0;
    } else {
        k_theta = 3.0 * alpha_o;
        k_w = 3.0 * pow(alpha_o, 2.0);
        k_tau = J * pow(alpha_o, 3.0);
    }
    speed_observer_init(&self->speed_observer, k_w, k_tau, J);
    flux_observer_init(&self->flux_observer, par, k_theta, k_o);
}

static void speed_flux_observer_compute_output(const SpeedFluxObserver *self,
                                               double complex u_s_ab,
                                               double complex i_s_ab, double eps_ext,
                                               double h, ObserverOutputs *out)
{
    double w_M = self->speed_observer.w_M;
    double tau_L = self->speed_observer.tau_L;
    flux_observer_compute_output(&self->flux_observer, u_s_ab, i_s_ab, w_M, eps_ext, h,
                                 out);
    out->tau_L = tau_L;
}

static void speed_flux_observer_update(SpeedFluxObserver *self, double T_s,
                                       const ObserverOutputs *out)
{
    speed_observer_update(&self->speed_observer, T_s, out->eps, out->tau_M);
    flux_observer_update(&self->flux_observer, T_s, out);
}

/* Estimates initialized from the current response delta_i_s_ab to a zero-voltage
 * period of length T_s, i_s_ab being the current at its end (SpeedFluxObserver.catch):
 * the back-EMF drives the current along the negative q axis of the rotor, which
 * gives the rotor angle and the speed magnitude, the direction of rotation being
 * taken from the speed estimate w_M, which is updated with the angle and flux
 * estimates. Returns 0 if the back-EMF is below the resistive voltage drop
 * (standstill) or the machine has no magnets, when the estimates are left
 * unchanged. */
static int flux_observer_catch(const SynchronousMachinePars *par,
                               double complex delta_i_s_ab, double complex i_s_ab,
                               double T_s, double *w_M, double *theta_m,
                               double complex *psi_s)
{
    IncrIndMat L = incr_ind_mat(par, 0.0);
    double complex e_s_ab = -L.L_qq * delta_i_s_ab / T_s; /* Back-EMF */
    if (cabs(e_s_ab) <= par->R_s * cabs(i_s_ab) || par->psi_f == 0.0) {
        return 0;
    }
    double sign = (*w_M >= 0.0) ? 1.0 : -1.0;
    double w_m = sign * cabs(e_s_ab) / par->psi_f;
    /* The response lags the rotor angle at the end of the period, since the
     * back-EMF rotates during the period */
    *theta_m = wrap(carg(e_s_ab) - sign * M_PI / 2.0
                    + w_m * T_s * L.L_qq / (2.0 * L.L_dd));
    *w_M = w_m / par->n_p;
    *psi_s = psi_s_dq(par, cexp(-I * *theta_m) * i_s_ab);
    return 1;
}

static void speed_flux_observer_catch(SpeedFluxObserver *self,
                                      double complex delta_i_s_ab,
                                      double complex i_s_ab, double T_s)
{
    FluxObserver *fo = &self->flux_observer;
    flux_observer_catch(&fo->par, delta_i_s_ab, i_s_ab, T_s,
                        &self->speed_observer.w_M, &fo->theta_m, &fo->psi_s);
}

/* ReferenceGenerator ------------------------------------------------------- */

static void reference_gen_init(ReferenceGenerator *self,
                               const SynchronousMachinePars *par, double i_s_max,
                               double psi_s_min, double psi_s_max, double k_u,
                               double k_mtpv, const ReferenceLUTs *luts)
{
    self->k_u = k_u;
    self->k_mtpv = k_mtpv;
    self->psi_s_min = isnan(psi_s_min) ? par->psi_f : psi_s_min;
    self->psi_s_max = psi_s_max;
    ReferenceLUTs computed;
    if (luts == NULL) {
        compute_reference_luts(par, i_s_max, &computed);
        luts = &computed;
    }
    self->psi_s_mtpa = luts->psi_s_mtpa;
    self->tau_M_cl = luts->tau_M_cl;
    self->tau_M_mtpv = luts->tau_M_mtpv;
}

static double reference_gen_max_flux(const ReferenceGenerator *self, double w_m,
                                     double u_dc)
{
    double u_s_max = self->k_u * u_dc / sqrt(3.0);
    return (w_m != 0.0) ? u_s_max / fabs(w_m) : INFINITY;
}

static void reference_gen_compute_flux_and_torque_refs(const ReferenceGenerator *self,
                                                       double tau_M_ref, double w_m,
                                                       double u_dc, double *psi_s_ref,
                                                       double *tau_M_ref_lim)
{
    /* MTPA flux */
    double psi_s_abs_ref = clip(lut_interp(&self->psi_s_mtpa, fabs(tau_M_ref)),
                                self->psi_s_min, self->psi_s_max);

    /* Maximum flux (field weakening) */
    psi_s_abs_ref = fmin(psi_s_abs_ref, reference_gen_max_flux(self, w_m, u_dc));

    /* Current limit */
    double tau_M_cl = lut_interp(&self->tau_M_cl, psi_s_abs_ref);
    tau_M_ref = fmin(tau_M_cl, fabs(tau_M_ref)) * sign(tau_M_ref);

    /* MTPV limit */
    double tau_M_mtpv = lut_interp(&self->tau_M_mtpv, psi_s_abs_ref);
    if (tau_M_mtpv > 0.0) {
        tau_M_ref = fmin(self->k_mtpv * tau_M_mtpv, fabs(tau_M_ref)) * sign(tau_M_ref);
    }

    *psi_s_ref = psi_s_abs_ref;
    *tau_M_ref_lim = tau_M_ref;
}

/* FluxTorqueController ----------------------------------------------------- */

static void flux_torque_ctrl_init(FluxTorqueController *self,
                                  SynchronousMachinePars par, double alpha_psi,
                                  double alpha_tau, double alpha_i)
{
    self->par = par;
    self->alpha_psi = alpha_psi;
    self->alpha_tau = alpha_tau;
    self->alpha_i = alpha_i;
    self->x_psi = alpha_i * par.psi_f;
    self->x_tau = 0.0;
    self->i_a = 0.0;
    self->u_ff = 0.0;
}

static double complex flux_torque_ctrl_aux_current(const FluxTorqueController *self,
                                                   double complex psi_s,
                                                   double complex i_s)
{
    IncrIndMat L = incr_ind_mat(&self->par, i_s);
    return aux_current(&L, psi_s, i_s);
}

static double complex flux_torque_ctrl_compute_output(FluxTorqueController *self,
                                                      double psi_s_ref,
                                                      double tau_M_ref,
                                                      const ObserverOutputs *fbk)
{
    const SynchronousMachinePars *par = &self->par;
    double psi_s_abs = cabs(fbk->psi_s);

    /* Auxiliary current and torque-production factor */
    double complex i_a = flux_torque_ctrl_aux_current(self, fbk->psi_s, fbk->i_s);
    double c_tau = 1.5 * par->n_p * creal(i_a * conj(fbk->psi_s));

    /* Directions */
    double complex t_psi = (c_tau > 0.0) ? 1.5 * par->n_p * psi_s_abs * i_a / c_tau : 1.0;
    double complex t_tau = (c_tau > 0.0) ? I * fbk->psi_s / c_tau : 0.0;

    /* If c_tau < 0, the torque cannot be controlled. In a PM machine with weak PMs,
     * this occurs at a high flux and a small load angle (psi_s.real > 0), where the
     * flux is reduced until the torque becomes controllable again. Beyond the MTPV
     * limit (psi_s.real < 0 for PM machines), reducing the flux would not help. */
    if (c_tau < 0.0 && par->psi_f > 0.0 && creal(fbk->psi_s) > 0.0) {
        psi_s_ref = 0.0;
    }

    /* Error signals */
    double e_psi = psi_s_ref - psi_s_abs;
    double e_tau = tau_M_ref - fbk->tau_M;
    double complex e_u = self->alpha_psi * e_psi * t_psi + self->alpha_tau * e_tau * t_tau;
    double complex u_i = self->x_psi * t_psi + self->x_tau * t_tau;
    double complex e_v = u_i - self->alpha_i * (psi_s_abs * t_psi + fbk->tau_M * t_tau);

    /* Voltage reference, consisting of the feedforward and integral action (u_ff)
     * and the proportional action (e_u) */
    double complex u_ff = par->R_s * fbk->i_s + I * fbk->w_m * fbk->psi_s + e_v;

    /* Workspace variables for the update function */
    self->i_a = i_a;
    self->u_ff = u_ff;

    return u_ff + e_u;
}

static void flux_torque_ctrl_update(FluxTorqueController *self, double T_s,
                                    const ObserverOutputs *fbk)
{
    const SynchronousMachinePars *par = &self->par;
    double psi_s_abs = cabs(fbk->psi_s);
    /* Error signal and gains */
    double complex u_err = fbk->u_s - self->u_ff;
    double k_psi = (psi_s_abs > 0.0) ? self->alpha_i / psi_s_abs : 0.0;
    double k_tau = 1.5 * par->n_p * self->alpha_i;
    /* Update the integral states */
    self->x_psi += T_s * k_psi * creal(fbk->psi_s * conj(u_err));
    self->x_tau += T_s * k_tau * creal(I * self->i_a * conj(u_err));
}

/* FluxVectorController ----------------------------------------------------- */

static void flux_vector_ctrl_init(FluxVectorController *self,
                                  SynchronousMachinePars par,
                                  const FluxVectorControllerCfg *cfg)
{
    /* Resolve the defaults, as in FluxVectorController.__init__ and
     * create_speed_flux_observer */
    double alpha_psi = isnan(cfg->alpha_psi) ? cfg->alpha_tau : cfg->alpha_psi;
    double alpha_i = isnan(cfg->alpha_i) ? cfg->alpha_tau : cfg->alpha_i;
    double J = isnan(cfg->J) ? 0.0 : cfg->J; /* Zero means None */
    double alpha_o = cfg->alpha_o;
    if (isnan(alpha_o)) {
        /* Default speed estimation poles (default_alpha_o). With J, the triple pole
         * at alpha_o/sqrt(3) keeps the speed gain about the same. */
        alpha_o = cfg->sensorless ? 2.0 * M_PI * 50.0 : 2.0 * M_PI * 400.0;
        alpha_o = (J > 0.0) ? alpha_o / sqrt(3.0) : alpha_o;
    }
    /* Default observer gain of create_speed_flux_observer (default_sensorless_k_o in
     * the sensorless mode) */
    double k_o[2] = {cfg->k_o[0], cfg->k_o[1]};
    if (isnan(k_o[0]) || isnan(k_o[1])) {
        if (cfg->sensorless) {
            IncrIndMat L_s0 = incr_ind_mat(&par, 0.0);
            k_o[0] = 0.25 * par.R_s * (1.0 / L_s0.L_dd + 1.0 / L_s0.L_qq);
            k_o[1] = 0.2;
        } else {
            k_o[0] = 2.0 * M_PI * 15.0;
            k_o[1] = 0.0;
        }
    }

    self->T_s = cfg->T_s;
    self->sensorless = cfg->sensorless;
    reference_gen_init(&self->reference_gen, &par, cfg->i_s_max, cfg->psi_s_min,
                       cfg->psi_s_max, cfg->k_u, cfg->k_mtpv, NULL);
    flux_torque_ctrl_init(&self->flux_torque_ctrl, par, alpha_psi, cfg->alpha_tau,
                          alpha_i);
    speed_flux_observer_init(&self->observer, par, alpha_o, cfg->T_s, k_o, J);
}

static void flux_vector_ctrl_compute_output(FluxVectorController *self,
                                            double tau_M_ref,
                                            const ObserverOutputs *fbk,
                                            References *ref)
{
    ref->T_s = self->T_s;
    reference_gen_compute_flux_and_torque_refs(&self->reference_gen, tau_M_ref,
                                               fbk->w_m, fbk->u_dc, &ref->psi_s,
                                               &ref->tau_M);
    ref->u_s = flux_torque_ctrl_compute_output(&self->flux_torque_ctrl, ref->psi_s,
                                               ref->tau_M, fbk);
}

static void flux_vector_ctrl_update(FluxVectorController *self, const References *ref,
                                    const ObserverOutputs *fbk)
{
    speed_flux_observer_update(&self->observer, ref->T_s, fbk);
    flux_torque_ctrl_update(&self->flux_torque_ctrl, ref->T_s, fbk);
}

/* VectorControlSystem ------------------------------------------------------ */

static void vector_control_system_init(VectorControlSystem *self,
                                       SynchronousMachinePars par,
                                       const FluxVectorControllerCfg *cfg,
                                       PIController speed_ctrl)
{
    pwm_init(&self->pwm, 1.5);
    self->k_comp = 1.5; /* k_comp of FluxVectorController */
    flux_vector_ctrl_init(&self->vector_ctrl, par, cfg);
    self->speed_ctrl = speed_ctrl;
    self->speed_ctrl.u_i = 0.0;
    self->speed_ctrl.v = 0.0;
    self->enabled = 1;
    self->catch_step = 0;
    self->i_c_ab_catch = 0.0;
    self->T_s_catch = 0.0;
}

static void vector_control_system_compute_output(VectorControlSystem *self,
                                                 const Measurements *meas,
                                                 double w_M_ref, int enabled)
{
    ObserverOutputs *fbk = &self->fbk;
    References *ref = &self->ref;
    self->enabled = enabled;

    /* Flying start of a sensorless drive after the release of the gates
     * (VectorControlSystem.get_feedback): the zero vector of the standby output is
     * applied in the first sampling period after the release, and the estimates are
     * initialized from the current response at the end of that period. The output
     * of the first step is computed as in standby, since it is applied before the
     * catch (the computational delay). */
    if (self->vector_ctrl.sensorless) {
        if (!enabled) {
            self->catch_step = 1;
        } else if (self->catch_step == 1) {
            self->catch_step = 2; /* The zero-voltage period starts */
            self->i_c_ab_catch = meas->i_c_ab;
        } else if (self->catch_step == 2) {
            self->catch_step = 0;
            speed_flux_observer_catch(&self->vector_ctrl.observer,
                                      meas->i_c_ab - self->i_c_ab_catch,
                                      meas->i_c_ab, self->T_s_catch);
        }
    }

    /* Feedback signals (DriveControlSystem.get_feedback). The observer uses the
     * voltage of the ongoing sampling period, while the control laws use the voltage
     * averaged over the previous and ongoing periods. In standby (with blocked gates
     * and at the first step after the release), the voltage is unknown (NAN), and
     * the observer runs on its open-circuit model, whose voltage is used for the
     * control laws too. */
    int standby = !enabled || self->catch_step != 0;
    double complex u_c_ab_zoh = NAN;
    double complex u_c_ab = NAN;
    if (!standby) {
        u_c_ab_zoh = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 0);
        u_c_ab = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 1);
    }
    const SpeedFluxObserver *observer = &self->vector_ctrl.observer;
    if (self->vector_ctrl.sensorless) {
        speed_flux_observer_compute_output(observer, u_c_ab_zoh, meas->i_c_ab, 0.0,
                                           0.0, fbk);
    } else {
        /* Position error from the measured rotor angle (compute_feedback) */
        double n_p = observer->flux_observer.par.n_p;
        double theta_m = observer->flux_observer.theta_m;
        double eps = wrap(n_p * meas->theta_M - theta_m) / n_p;
        speed_flux_observer_compute_output(observer, u_c_ab_zoh, meas->i_c_ab, eps,
                                           1.0, fbk);
    }
    fbk->u_s = standby ? fbk->u_s_zoh : cexp(-I * fbk->theta_c) * u_c_ab;
    fbk->u_dc = meas->u_dc;

    /* Speed controller and vector controller, or in standby, the vector controller
     * with zero torque reference and the zero vector of a sensorless drive */
    double tau_M_ref = 0.0;
    if (!standby) {
        tau_M_ref = pi_compute_output(&self->speed_ctrl, w_M_ref, fbk->w_M, 0.0);
    }
    flux_vector_ctrl_compute_output(&self->vector_ctrl, tau_M_ref, fbk, ref);

    /* Duty ratios for the PWM, the angle of the voltage reference advanced by
     * k_comp*T_s*w_c, or the zero vector of the flying start */
    double complex u_s_ab_ref = cexp(I * fbk->theta_c) * ref->u_s;
    u_s_ab_ref *= cexp(I * self->k_comp * ref->T_s * fbk->w_c);
    if (self->catch_step != 0) {
        u_s_ab_ref = 0.0;
        self->T_s_catch = ref->T_s;
    }
    ref->u_c_ab = pwm_compute_output(&self->pwm, ref->T_s, u_s_ab_ref, fbk->u_dc,
                                     fbk->w_c, meas->i_c_ab, ref->d_abc);
    ref->w_M = w_M_ref;
}

static void vector_control_system_update(VectorControlSystem *self)
{
    pwm_update(&self->pwm, self->ref.T_s, self->ref.d_abc);
    flux_vector_ctrl_update(&self->vector_ctrl, &self->ref, &self->fbk);
    if (self->enabled && self->catch_step == 0) {
        pi_update(&self->speed_ctrl, self->ref.T_s, self->ref.tau_M);
    }
}

/* Signal vectors of the modular control system ---------------------------- */

#define COMPLEX_AT(u, i) complex_from((u)[i], (u)[(i) + 1])

static void measurements_pack(const Measurements *meas, double y[MEAS_WIDTH])
{
    y[MEAS_i_c_ab] = creal(meas->i_c_ab);
    y[MEAS_i_c_ab + 1] = cimag(meas->i_c_ab);
    y[MEAS_u_dc] = meas->u_dc;
    y[MEAS_theta_M] = meas->theta_M;
}

static void measurements_unpack(const double u[MEAS_WIDTH], Measurements *meas)
{
    meas->i_c_ab = COMPLEX_AT(u, MEAS_i_c_ab);
    meas->u_dc = u[MEAS_u_dc];
    meas->theta_M = u[MEAS_theta_M];
}

static void observer_outputs_pack(const ObserverOutputs *fbk, double y[FBK_WIDTH])
{
    const double complex z[] = {fbk->i_s,   fbk->u_s,   fbk->u_s_zoh,
                                fbk->psi_s, fbk->e_o,   fbk->psi_a};
    const int i_z[] = {FBK_i_s, FBK_u_s, FBK_u_s_zoh, FBK_psi_s, FBK_e_o, FBK_psi_a};
    for (int k = 0; k < 6; k++) {
        y[i_z[k]] = creal(z[k]);
        y[i_z[k] + 1] = cimag(z[k]);
    }
    y[FBK_eps] = fbk->eps;
    y[FBK_eps_f] = fbk->eps_f;
    y[FBK_tau_M] = fbk->tau_M;
    y[FBK_w_c] = fbk->w_c;
    y[FBK_w_m] = fbk->w_m;
    y[FBK_w_M] = fbk->w_M;
    y[FBK_theta_c] = fbk->theta_c;
    y[FBK_theta_m] = fbk->theta_m;
    y[FBK_psi_f] = fbk->psi_f;
    y[FBK_h] = fbk->h;
}

static void observer_outputs_unpack(const double u[FBK_WIDTH], ObserverOutputs *fbk)
{
    fbk->u_dc = 0.0;
    fbk->i_s = COMPLEX_AT(u, FBK_i_s);
    fbk->u_s = COMPLEX_AT(u, FBK_u_s);
    fbk->u_s_zoh = COMPLEX_AT(u, FBK_u_s_zoh);
    fbk->psi_s = COMPLEX_AT(u, FBK_psi_s);
    fbk->e_o = COMPLEX_AT(u, FBK_e_o);
    fbk->eps = u[FBK_eps];
    fbk->eps_f = u[FBK_eps_f];
    fbk->psi_a = COMPLEX_AT(u, FBK_psi_a);
    fbk->tau_M = u[FBK_tau_M];
    fbk->tau_L = 0.0;
    fbk->w_c = u[FBK_w_c];
    fbk->w_m = u[FBK_w_m];
    fbk->w_M = u[FBK_w_M];
    fbk->theta_c = u[FBK_theta_c];
    fbk->theta_m = u[FBK_theta_m];
    fbk->psi_f = u[FBK_psi_f];
    fbk->h = u[FBK_h];
}

static void references_unpack(const double u[REF_WIDTH], References *ref)
{
    References zero = {0};
    *ref = zero;
    ref->psi_s = u[REF_psi_s];
    ref->tau_M = u[REF_tau_M];
    ref->u_s = COMPLEX_AT(u, REF_u_s);
}

#undef COMPLEX_AT
