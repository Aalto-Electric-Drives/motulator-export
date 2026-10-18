/*
 * Helpers of the native libraries (native_<name>.c), which include this file after
 * the C port of their control system, sim.c, and the model.
 */

/* Limited voltage reference (V) in stationary coordinates, excluding the compensation
 * of the duty-ratio error (PWM.limited_voltage), from the limited converter voltage
 * u_c_ab of the C port and the voltage reference u_ref (in the coordinates of the
 * angle theta_c), whose angle is advanced by k_comp*T_s*w_c before the modulation.
 * With the compensation, the minimum pulses are not supported. */
static double complex native_limited_voltage(const PWM *pwm, double complex u_c_ab,
                                             double complex u_ref, double theta_c,
                                             double k_comp, double T_s, double w_c,
                                             double u_dc)
{
    if (!(pwm_has_error_model(pwm) && pwm->feedforward) || u_dc <= 0.0) {
        return u_c_ab;
    }
    /* Duty ratios of the voltage reference, as in the control systems */
    double complex u_ref_ab = cexp(I * theta_c) * u_ref;
    u_ref_ab *= cexp(I * k_comp * T_s * w_c);
    double d_ref[3];
    pwm_duty_ratios(u_ref_ab, u_dc, pwm->mme, d_ref);
    return abc2complex(d_ref) * u_dc;
}

/* Settings of the PWM: the overmodulation (MME if mme is nonzero, else MPE), the
 * duty-ratio error model InverterNonlinearity(t_d, u_th, sign) with sign =
 * tanh(i/i_0), or np.sign if i_0 = 0, with its feedforward compensation, the minimum
 * duty ratio, and the single update */
static void native_pwm_settings(PWM *pwm, int mme, double t_d, double u_th,
                                double i_0, int feedforward, double d_min,
                                int single_update)
{
    pwm_set_overmodulation(pwm, mme);
    pwm_set_nonlinearity(pwm, t_d, u_th, i_0, feedforward);
    pwm_set_min_pulse(pwm, d_min);
    pwm_set_single_update(pwm, single_update);
}

/* Copy the settings of the PWM (native_pwm_settings) into the PWM of a constructed
 * control system */
static void native_pwm_copy_settings(PWM *pwm, const PWM *settings)
{
    native_pwm_settings(pwm, settings->mme, settings->t_d, settings->u_th,
                        settings->i_0, settings->feedforward, settings->d_min,
                        settings->single_update);
}
