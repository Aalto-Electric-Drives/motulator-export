/*
 * Open-loop V/Hz control of induction machine drives, ported from motulator. See
 * im_open_loop_vhz.h.
 */

#include "im_open_loop_vhz.h"

/* IMOpenLoopVHzController -------------------------------------------------- */

static void im_open_loop_vhz_ctrl_init(IMOpenLoopVHzController *self, double n_p,
                                       double complex u_0, double complex k,
                                       double T_s)
{
    self->n_p = n_p;
    self->u_0 = u_0;
    self->k = k;
    self->T_s = T_s;
    self->theta_c = 0.0;
}

static void im_open_loop_vhz_ctrl_get_feedback(const IMOpenLoopVHzController *self,
                                               double complex u_s_ab,
                                               double complex i_s_ab, double w_M_ref,
                                               IMOpenLoopVHzFeedbacks *fbk)
{
    IMOpenLoopVHzFeedbacks zero = {0};
    *fbk = zero;
    fbk->w_M = w_M_ref;
    fbk->theta_c = self->theta_c;
    fbk->w_c = fbk->w_s = self->n_p * w_M_ref;
    fbk->i_s = cexp(-I * fbk->theta_c) * i_s_ab;
    if (!isnan(creal(u_s_ab))) {
        fbk->u_s_zoh = cexp(-I * fbk->theta_c) * u_s_ab;
    }
}

static double complex im_open_loop_vhz_ctrl_compute_output(
    const IMOpenLoopVHzController *self, const IMOpenLoopVHzFeedbacks *fbk)
{
    return self->u_0 + self->k * fbk->w_s;
}

static void im_open_loop_vhz_ctrl_update(IMOpenLoopVHzController *self, double T_s,
                                         const IMOpenLoopVHzFeedbacks *fbk)
{
    self->theta_c = wrap(self->theta_c + T_s * fbk->w_c);
}

/* IMOpenLoopVHzControlSystem ----------------------------------------------- */

static void im_open_loop_vhz_control_system_init(
    IMOpenLoopVHzControlSystem *self, const IMOpenLoopVHzController *vhz_ctrl,
    double slew_rate)
{
    pwm_init(&self->pwm, 1.5);
    pwm_set_overmodulation(&self->pwm, 1); /* Default of VHzControlSystem: MME */
    self->k_comp = 1.5;                    /* k_comp of OpenLoopVHzController */
    self->vhz_ctrl = *vhz_ctrl;
    rate_limiter_init(&self->rate_limiter, slew_rate);
}

static void im_open_loop_vhz_control_system_compute_output(
    IMOpenLoopVHzControlSystem *self, const IMOpenLoopVHzMeasurements *meas,
    double w_M_ref, int enabled)
{
    IMOpenLoopVHzController *ctrl = &self->vhz_ctrl;
    IMOpenLoopVHzFeedbacks *fbk = &self->fbk;
    IMOpenLoopVHzReferences *ref = &self->ref;
    double T_s = ctrl->T_s;

    /* Feedback signals (DriveControlSystem.get_feedback). The voltage of the
     * ongoing sampling period is u_s_zoh, and the voltage averaged over the previous
     * and ongoing periods is u_s. With blocked gates, the voltage is unknown (NAN),
     * and u_s is u_s_zoh (zero). */
    double complex u_c_ab_zoh = NAN;
    double complex u_c_ab = NAN;
    if (enabled) {
        u_c_ab_zoh = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 0);
        u_c_ab = pwm_realized_voltage(&self->pwm, meas->i_c_ab, meas->u_dc, 1);
    }

    /* Rate-limited speed reference (RateLimiter), stored in the update function */
    w_M_ref = rate_limiter_compute_output(&self->rate_limiter, T_s, w_M_ref);

    im_open_loop_vhz_ctrl_get_feedback(ctrl, u_c_ab_zoh, meas->i_c_ab, w_M_ref, fbk);
    fbk->u_s = enabled ? cexp(-I * fbk->theta_c) * u_c_ab : fbk->u_s_zoh;
    fbk->u_dc = meas->u_dc;

    /* References, the same with blocked gates */
    ref->T_s = T_s;
    ref->u_s = im_open_loop_vhz_ctrl_compute_output(ctrl, fbk);

    /* Duty ratios for the PWM, the angle of the voltage reference advanced by
     * k_comp*T_s*w_c, and the limited voltage reference without the advance */
    double complex advance = cexp(I * self->k_comp * T_s * fbk->w_c);
    double complex u_s_ab_ref = cexp(I * fbk->theta_c) * ref->u_s;
    u_s_ab_ref *= advance;
    double complex u_lim_ab;
    pwm_compute_output(&self->pwm, T_s, u_s_ab_ref, fbk->u_dc, fbk->w_c, meas->i_c_ab,
                       ref->d_abc, &u_lim_ab);
    ref->u_s_lim = cexp(-I * fbk->theta_c) * conj(advance) * u_lim_ab;
    ref->w_M = w_M_ref;
}

static void im_open_loop_vhz_control_system_update(IMOpenLoopVHzControlSystem *self)
{
    double T_s = self->ref.T_s;
    pwm_update(&self->pwm, T_s, self->ref.d_abc);
    rate_limiter_update(&self->rate_limiter, self->ref.w_M);
    im_open_loop_vhz_ctrl_update(&self->vhz_ctrl, T_s, &self->fbk);
}

/* Signal vectors of the blocks --------------------------------------------- */

#define COMPLEX_AT(u, i) complex_from((u)[i], (u)[(i) + 1])
#define SET_COMPLEX(y, i, z)                                                       \
    do {                                                                           \
        (y)[i] = creal(z);                                                         \
        (y)[(i) + 1] = cimag(z);                                                   \
    } while (0)

static void im_open_loop_vhz_measurements_pack(const IMOpenLoopVHzMeasurements *meas,
                                               double y[OLVHZ_MEAS_WIDTH])
{
    SET_COMPLEX(y, OLVHZ_MEAS_i_c_ab, meas->i_c_ab);
    y[OLVHZ_MEAS_u_dc] = meas->u_dc;
}

static void im_open_loop_vhz_measurements_unpack(const double u[OLVHZ_MEAS_WIDTH],
                                                 IMOpenLoopVHzMeasurements *meas)
{
    meas->i_c_ab = COMPLEX_AT(u, OLVHZ_MEAS_i_c_ab);
    meas->u_dc = u[OLVHZ_MEAS_u_dc];
}

static void im_open_loop_vhz_feedbacks_pack(const IMOpenLoopVHzFeedbacks *fbk,
                                            double y[OLVHZ_FBK_WIDTH])
{
    SET_COMPLEX(y, OLVHZ_FBK_i_s, fbk->i_s);
    SET_COMPLEX(y, OLVHZ_FBK_u_s, fbk->u_s);
    SET_COMPLEX(y, OLVHZ_FBK_u_s_zoh, fbk->u_s_zoh);
    y[OLVHZ_FBK_w_M] = fbk->w_M;
    y[OLVHZ_FBK_w_s] = fbk->w_s;
    y[OLVHZ_FBK_w_c] = fbk->w_c;
    y[OLVHZ_FBK_theta_c] = fbk->theta_c;
}

static void im_open_loop_vhz_feedbacks_unpack(const double u[OLVHZ_FBK_WIDTH],
                                              IMOpenLoopVHzFeedbacks *fbk)
{
    fbk->u_dc = 0.0;
    fbk->i_s = COMPLEX_AT(u, OLVHZ_FBK_i_s);
    fbk->u_s = COMPLEX_AT(u, OLVHZ_FBK_u_s);
    fbk->u_s_zoh = COMPLEX_AT(u, OLVHZ_FBK_u_s_zoh);
    fbk->w_M = u[OLVHZ_FBK_w_M];
    fbk->w_s = u[OLVHZ_FBK_w_s];
    fbk->w_c = u[OLVHZ_FBK_w_c];
    fbk->theta_c = u[OLVHZ_FBK_theta_c];
}

static void im_open_loop_vhz_references_unpack(const double u[OLVHZ_REF_WIDTH],
                                               IMOpenLoopVHzReferences *ref)
{
    IMOpenLoopVHzReferences zero = {0};
    *ref = zero;
    ref->u_s = COMPLEX_AT(u, OLVHZ_REF_u_s);
}

#undef COMPLEX_AT
#undef SET_COMPLEX
