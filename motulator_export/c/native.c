/*
 * Helpers of the native libraries (native_<name>.c), which include this file after
 * the C port of their control system, sim.c, and the model.
 */

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
