/*
 * Open-loop V/Hz control of induction machine drives, ported from motulator.
 *
 * Python counterparts:
 *   motulator/drive/control/_im_open_loop_vhz.py  OpenLoopVHzFeedbacks,
 *                                                 OpenLoopVHzController
 *   motulator/drive/control/_base.py              DriveControlSystem,
 *                                                 VHzControlSystem
 *   motulator/common/control/_controllers.py      RateLimiter
 *
 * The V/Hz curve is affine, u_s_ref(w_s) = u_0 + k*w_s, given by the voltage u_0 at
 * zero frequency and the slope k. The PWM uses the MME overmodulation (the default
 * PWM of VHzControlSystem). As in the other control
 * systems, the output function has no side effects (apart from workspace variables),
 * and all states are updated in the update function.
 */

#ifndef MOTULATOR_IM_OPEN_LOOP_VHZ_H
#define MOTULATOR_IM_OPEN_LOOP_VHZ_H

#include "common.h"

/* Feedback signals (OpenLoopVHzFeedbacks) */
typedef struct {
    double u_dc;            /* DC-bus voltage */
    double complex i_s;     /* Stator current */
    double complex u_s;     /* Stator voltage averaged over the previous and ongoing
                             * sampling periods, set by the control system */
    double complex u_s_zoh; /* Stator voltage averaged over the ongoing period */
    double w_M;             /* Rate-limited speed reference (mechanical rad/s) */
    double w_s;             /* Stator angular frequency */
    double w_c;             /* Angular speed of the coordinate system */
    double theta_c;         /* Coordinate system angle */
} IMOpenLoopVHzFeedbacks;

/* Open-loop V/Hz controller (OpenLoopVHzController) */
typedef struct {
    double n_p;
    double complex u_0; /* V/Hz curve u_0 + k*w_s: voltage at zero frequency (V) */
    double complex k;   /* and slope (Vs) */
    double T_s;
    double theta_c; /* State: angle of the coordinate system */
} IMOpenLoopVHzController;

static void im_open_loop_vhz_ctrl_init(IMOpenLoopVHzController *self, double n_p,
                                       double complex u_0, double complex k,
                                       double T_s);
/* Feedback signals, the voltage u_s_ab NAN if unknown (blocked gates) */
static void im_open_loop_vhz_ctrl_get_feedback(const IMOpenLoopVHzController *self,
                                               double complex u_s_ab,
                                               double complex i_s_ab, double w_M_ref,
                                               IMOpenLoopVHzFeedbacks *fbk);
/* Stator voltage reference from the V/Hz curve */
static double complex im_open_loop_vhz_ctrl_compute_output(
    const IMOpenLoopVHzController *self, const IMOpenLoopVHzFeedbacks *fbk);
static void im_open_loop_vhz_ctrl_update(IMOpenLoopVHzController *self, double T_s,
                                         const IMOpenLoopVHzFeedbacks *fbk);

/* Reference signals of open-loop V/Hz control */
typedef struct {
    double T_s;
    double d_abc[3];
    double w_M; /* Rate-limited speed reference */
    double complex u_s;
    double complex u_s_lim; /* Limited voltage reference, excluding the
                             * compensation (PWM.limited_voltage) */
} IMOpenLoopVHzReferences;

/* Measured signals */
typedef struct {
    double complex i_c_ab; /* Converter current in stationary coordinates */
    double u_dc;
} IMOpenLoopVHzMeasurements;

/* V/Hz control system (VHzControlSystem) with the open-loop V/Hz controller. While
 * the gates are blocked, the converter voltage is unknown, and the controller
 * computes its output as with the gates enabled. */
typedef struct {
    PWM pwm;
    double k_comp; /* Angle-advance factor of the voltage reference (k_comp) */
    IMOpenLoopVHzController vhz_ctrl;
    RateLimiter rate_limiter; /* Of the speed reference (mechanical rad/s^2) */
    /* Workspace variables passed from the output to the update function */
    IMOpenLoopVHzFeedbacks fbk;
    IMOpenLoopVHzReferences ref;
} IMOpenLoopVHzControlSystem;

static void im_open_loop_vhz_control_system_init(
    IMOpenLoopVHzControlSystem *self, const IMOpenLoopVHzController *vhz_ctrl,
    double slew_rate);
/* Output of the control loop, with the gate enable at the sampling instant */
static void im_open_loop_vhz_control_system_compute_output(
    IMOpenLoopVHzControlSystem *self, const IMOpenLoopVHzMeasurements *meas,
    double w_M_ref, int enabled);
static void im_open_loop_vhz_control_system_update(IMOpenLoopVHzControlSystem *self);

/* Signal vectors of the blocks (see sm_flux_vector.h): the measurements (OLVHZ_MEAS),
 * the feedback signals (OLVHZ_FBK, without the DC-bus voltage of the measurements),
 * and the references (OLVHZ_REF), whose only field is the stator voltage reference
 * u_s */
#define OLVHZ_MEAS_i_c_ab 0
#define OLVHZ_MEAS_u_dc 2
#define OLVHZ_MEAS_WIDTH 3

#define OLVHZ_FBK_i_s 0
#define OLVHZ_FBK_u_s 2
#define OLVHZ_FBK_u_s_zoh 4
#define OLVHZ_FBK_w_M 6
#define OLVHZ_FBK_w_s 7
#define OLVHZ_FBK_w_c 8
#define OLVHZ_FBK_theta_c 9
#define OLVHZ_FBK_WIDTH 10

#define OLVHZ_REF_u_s 0
#define OLVHZ_REF_WIDTH 2

static void im_open_loop_vhz_measurements_pack(const IMOpenLoopVHzMeasurements *meas,
                                               double y[OLVHZ_MEAS_WIDTH]);
static void im_open_loop_vhz_measurements_unpack(const double u[OLVHZ_MEAS_WIDTH],
                                                 IMOpenLoopVHzMeasurements *meas);
static void im_open_loop_vhz_feedbacks_pack(const IMOpenLoopVHzFeedbacks *fbk,
                                            double y[OLVHZ_FBK_WIDTH]);
static void im_open_loop_vhz_feedbacks_unpack(const double u[OLVHZ_FBK_WIDTH],
                                              IMOpenLoopVHzFeedbacks *fbk);
/* The field u_s, the others are zero */
static void im_open_loop_vhz_references_unpack(const double u[OLVHZ_REF_WIDTH],
                                               IMOpenLoopVHzReferences *ref);

#endif
