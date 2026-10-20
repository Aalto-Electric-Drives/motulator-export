/*
 * Native simulation of grid converter systems with disturbance-observer-based
 * grid-forming control: the translation unit of the shared library of
 * motulator_export.native, with its API for Python (ctypes).
 *
 * The system (System) consists of the simulation (sim.h), the model of the grid
 * converter system (grid.h), and the control system: GridConverterControlSystem with
 * ObserverBasedGridFormingController (gfm_observer.h) in the power-control mode, with
 * the active power and AC voltage references. The API functions mirror the
 * constructors of motulator; those of the model are in sim.c and grid.c. The control
 * system is constructed from its parameters when the simulation starts (start),
 * after which sim_run of sim.c runs the simulation.
 */

#include "common.c"
#include "gfm_observer.c"
#include "sim.c"
#include "grid.c"
#include "native.c"

/* Signals logged at each step (the signal groups of ControlSystem.save), see
 * log_fields_width */
#define GFM_FIELDS                                                                     \
    "ref.T_s ref.p_g ref.q_g ref.v_c ref.u_c:c ref.i_c:c ref.u_dc:n ref.d_abc:3 "      \
    "ref.enabled:b fbk.i_c:c fbk.u_c:c fbk.u_c_zoh:c fbk.v_c:c fbk.u_g:c fbk.p_g "     \
    "fbk.q_g fbk.theta_c fbk.w_c fbk.u_dc"

/* Control system with the active power and AC voltage references */
typedef struct {
    ControlSystem base;
    GFMControlSystem ctrl;
    Signal p_g_ref;
    Signal v_c_ref;
    double t; /* Controller time */
} GFMSystemControl;

/* Save the signals of the step (GFM_FIELDS) */
static int gfm_save(GFMSystemControl *self)
{
    const GFMReferences *ref = &self->ctrl.ref;
    const GFMFeedbacks *fbk = &self->ctrl.fbk;
    double *p = log_append(&self->base.log);
    if (p == NULL) {
        return SIM_ERROR_MEMORY;
    }
    log_put(&p, self->t);
    log_put(&p, ref->T_s);
    log_put(&p, ref->p_g);
    log_put(&p, 0.0); /* Reactive power reference, not used */
    log_put(&p, ref->v_c);
    log_put_complex(&p, ref->u_c);
    log_put_complex(&p, ref->i_c);
    for (int k = 0; k < 3; k++) {
        log_put(&p, ref->d_abc[k]);
    }
    log_put(&p, 1.0); /* Gate enable */
    log_put_complex(&p, fbk->i_c);
    log_put_complex(&p, fbk->u_c);
    log_put_complex(&p, fbk->u_c_zoh);
    log_put_complex(&p, fbk->v_c);
    log_put_complex(&p, fbk->u_g);
    log_put(&p, fbk->p_g);
    log_put(&p, fbk->q_g);
    log_put(&p, fbk->theta_c);
    log_put(&p, fbk->w_c);
    log_put(&p, fbk->u_dc);
    return SIM_OK;
}

/* Control step (ControlSystem.run_control_loop with the measurements of
 * GridConverterControlSystem.get_measurement, the PCC voltage not used) */
static int gfm_run(ControlSystem *base, const Model *mdl, double t,
                   const double complex *x, double *T_s, double d_abc[3])
{
    GFMSystemControl *self = (GFMSystemControl *)base;
    const GridConverterSystem *grid = (const GridConverterSystem *)mdl;
    (void)t;
    double p_g_ref = signal_value(&self->p_g_ref, self->t);
    double v_c_ref = signal_value(&self->v_c_ref, self->t);
    if (!isfinite(p_g_ref) || !isfinite(v_c_ref)) {
        return SIM_ERROR_SIGNAL;
    }
    GFMMeasurements meas;
    meas.i_c_ab = grid_meas_current(grid, x);
    meas.u_dc = model_dc_voltage(mdl, x);
    meas.u_c_ab = 0.0;
    gfm_control_system_compute_output(&self->ctrl, &meas, p_g_ref, v_c_ref, 1);
    int status = gfm_save(self);
    if (status != SIM_OK) {
        return status;
    }
    gfm_control_system_update(&self->ctrl);
    *T_s = self->ctrl.ref.T_s;
    self->t = fmod(self->t + *T_s, 1e9); /* Avoid overflow, as ControlSystem.update */
    for (int k = 0; k < 3; k++) {
        d_abc[k] = self->ctrl.ref.d_abc[k];
    }
    return SIM_OK;
}

/* System ------------------------------------------------------------------- */

typedef struct {
    Simulation sim; /* First member, see the API of sim.c */
    GridConverterSystem mdl;
    GFMSystemControl ctrl;
    /* Parameters of the control system, from which it is constructed at the start */
    GFMControllerCfg cfg;
    PWM pwm; /* Settings of the PWM */
} System;

/* API for Python (motulator_export.native._grid) --------------------------- */

System *system_new(void)
{
    System *self = calloc(1, sizeof(System));
    if (self == NULL) {
        return NULL;
    }
    grid_init(&self->mdl);
    modulator_init(&self->mdl.base.modulator, 0, 0.0, 0.0, 1, 0);
    self->sim.mdl = &self->mdl.base;
    self->sim.ctrl = &self->ctrl.base;
    GFMSystemControl *ctrl = &self->ctrl;
    ctrl->base.run = gfm_run;
    ctrl->base.fields = GFM_FIELDS;
    log_init(&ctrl->base.log, log_fields_width(GFM_FIELDS));
    signal_constant(&ctrl->p_g_ref, 0.0);
    signal_constant(&ctrl->v_c_ref, 0.0);
    pwm_init(&self->pwm, 1.5);
    return self;
}

void system_free(System *self)
{
    if (self != NULL) {
        simulation_free(&self->sim);
        log_free(&self->ctrl.base.log);
        free(self);
    }
}

/* Configuration (see GFMControllerCfg of gfm_observer.h), NAN for the defaults */
void set_controller(System *self, double i_max, double L, double R, double R_a,
                    double k_v, double alpha_o, double alpha_c, double u_nom,
                    double w_nom, double i_d_max, double alpha_l, double T_s)
{
    GFMControllerCfg *cfg = &self->cfg;
    *cfg = gfm_controller_cfg(i_max, L);
    cfg->R = R;
    cfg->R_a = R_a;
    cfg->k_v = k_v;
    cfg->alpha_o = alpha_o;
    cfg->alpha_c = alpha_c;
    cfg->u_nom = u_nom;
    cfg->w_nom = w_nom;
    cfg->T_s = T_s;
    cfg->i_d_max = i_d_max;
    cfg->alpha_l = alpha_l;
}

/* PWM of the control system, see native_pwm_settings */
void set_pwm(System *self, int mme, double t_d, double u_th, double i_0,
             int feedforward, double d_min, int single_update)
{
    native_pwm_settings(&self->pwm, mme, t_d, u_th, i_0, feedforward, d_min,
                        single_update);
}

/* Active power reference, set by signal_steps or signal_function */
Signal *power_reference(System *self)
{
    return &self->ctrl.p_g_ref;
}

/* AC voltage reference, set by signal_steps or signal_function */
Signal *ac_voltage_reference(System *self)
{
    return &self->ctrl.v_c_ref;
}

/* Construct the control system and start the simulation at the time t0 with the
 * states x0 (real and imaginary parts) of the model, the first sampling instant
 * t_start, and the controller time t_ctrl */
void start(System *self, double t0, const double *x0, double t_start, double t_ctrl,
           double max_step, double rtol, double atol)
{
    GFMSystemControl *ctrl = &self->ctrl;
    gfm_control_system_init(&ctrl->ctrl, &self->cfg);
    native_pwm_copy_settings(&ctrl->ctrl.pwm, &self->pwm);
    ctrl->t = t_ctrl;

    double complex x[SIM_MAX_STATES];
    for (int i = 0; i < self->mdl.base.n; i++) {
        x[i] = complex_from(x0[2 * i], x0[2 * i + 1]);
    }
    SolverCfg cfg = {max_step, rtol, atol};
    simulation_init(&self->sim, &self->mdl.base, &ctrl->base, &cfg, t0, x, t_start);
}
