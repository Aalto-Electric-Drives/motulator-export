% Common control functions and classes, ported from motulator (MATLAB port of
% ../c/common.c). The functions are embedded in the MATLAB Function blocks by
% motulator_export.simulink, as local functions of the blocks that use them.
%
% Python counterparts:
%   motulator/common/utils/_utils.py         abc2complex, complex2abc, line2complex,
%                                            wrap, clip
%   motulator/common/control/_controllers.py PIController, ComplexPIController,
%                                            RateLimiter
%   motulator/common/control/_pwm.py         PWM (MPE and MME overmodulation),
%                                            InverterNonlinearity (with numpy.sign
%                                            or tanh)
%   motulator/common/utils/_dead_time.py     averaged_gate_signals
%   motulator/grid/control/_common.py        CurrentLimiter, PowerReferenceLimiter
%   motulator/drive/control/_common.py       SpeedController, SpeedObserver,
%                                            hold_equivalent, matched_pole
%
% Complex space vectors use peak-value scaling, as in motulator. A class is a struct
% with its functions, which return the modified struct (e.g., self = pi_update(self,
% T_s, u)). Complex variables and fields are complex from their initialization, as
% required by the code generation.

function u_ab = abc2complex(u)
%ABC2COMPLEX Space vector from the phase quantities u = [u_a u_b u_c].
u_ab = (2/3)*u(1) - (u(2) + u(3))/3 + 1i*(u(2) - u(3))/sqrt(3);
end

function u_abc = complex2abc(u)
%COMPLEX2ABC Phase quantities (a column vector) from the space vector.
u_abc = [real(u); 0.5*(-real(u) + sqrt(3)*imag(u)); 0.5*(-real(u) - sqrt(3)*imag(u))];
end

function u_ab = line2complex(u)
%LINE2COMPLEX Space vector from the line-to-line quantities u = [u_ab u_bc].
u_ab = (2*u(1) + u(2))/3 + 1i*u(2)/sqrt(3);
end

function z = complex_in(u)
%COMPLEX_IN Complex number from a signal [real part; imaginary part].
z = complex(u(1), u(2));
end

function y = complex_out(z)
%COMPLEX_OUT Signal [real part; imaginary part] of a complex number.
y = [real(z); imag(z)];
end

function theta = wrap(theta)
%WRAP Limit the angle into the range [-pi, pi), as numpy.mod does.
x = rem(theta + pi, 2*pi);
if x < 0
    x = x + 2*pi;
end
theta = x - pi;
end

function y = clip(value, min_value, max_value)
%CLIP Limit the value into the range [min_value, max_value].
y = max(min(value, max_value), min_value);
end

function y = lut_interp(lut, x)
%LUT_INTERP Lookup table with linear interpolation, equivalent to numpy.interp.
%   The table lut has the increasing sample points lut.x and the values lut.y.
n = numel(lut.x);
if x <= lut.x(1)
    y = lut.y(1);
    return
end
if x >= lut.x(n)
    y = lut.y(n);
    return
end
% Binary search for the interval x(lo) <= x < x(lo + 1)
lo = 1;
hi = n;
while hi - lo > 1
    mid = floor((lo + hi)/2);
    if lut.x(mid) <= x
        lo = mid;
    else
        hi = mid;
    end
end
slope = (lut.y(hi) - lut.y(lo))/(lut.x(hi) - lut.x(lo));
y = slope*(x - lut.x(lo)) + lut.y(lo);
end

function [x, aux] = brentq(f, xa, xb, data)
%BRENTQ Brent's root finding in a bracketing interval [xa, xb].
%   As scipy.optimize.brentq with the default tolerances (xtol = 2e-12, rtol =
%   4*eps, maxiter = 100), used via scipy.optimize.root_scalar in motulator. The
%   function is [y, aux] = f(x, data), and aux is that of the latest evaluation (a
%   side effect of the function in motulator). The function values at the bracket
%   ends must have opposite signs (or be zero), else x is NaN.
xtol = 2e-12;
rtol = 8.881784197001252e-16;
xpre = xa;
xcur = xb;
xblk = 0;
fblk = 0;
spre = 0;
scur = 0;
[fpre, aux] = f(xpre, data);
[fcur, aux] = f(xcur, data);
if fpre == 0
    x = xpre;
    return
end
if fcur == 0
    x = xcur;
    return
end
if (fpre < 0) == (fcur < 0)
    x = NaN; % No sign change
    return
end
for i = 1:100
    if fpre ~= 0 && fcur ~= 0 && (fpre < 0) ~= (fcur < 0)
        xblk = xpre;
        fblk = fpre;
        spre = xcur - xpre;
        scur = spre;
    end
    if abs(fblk) < abs(fcur)
        xpre = xcur;
        xcur = xblk;
        xblk = xpre;
        fpre = fcur;
        fcur = fblk;
        fblk = fpre;
    end
    delta = (xtol + rtol*abs(xcur))/2;
    sbis = (xblk - xcur)/2;
    if fcur == 0 || abs(sbis) < delta
        x = xcur;
        return
    end
    if abs(spre) > delta && abs(fcur) < abs(fpre)
        if xpre == xblk
            % Interpolate
            stry = -fcur*(xcur - xpre)/(fcur - fpre);
        else
            % Extrapolate
            dpre = (fpre - fcur)/(xpre - xcur);
            dblk = (fblk - fcur)/(xblk - xcur);
            stry = -fcur*(fblk*dblk - fpre*dpre)/(dblk*dpre*(fblk - fpre));
        end
        if 2*abs(stry) < min(abs(spre), 3*abs(sbis) - delta)
            % Good short step
            spre = scur;
            scur = stry;
        else
            % Bisect
            spre = sbis;
            scur = sbis;
        end
    else
        % Bisect
        spre = sbis;
        scur = sbis;
    end
    xpre = xcur;
    fpre = fcur;
    if abs(scur) > delta
        xcur = xcur + scur;
    elseif sbis > 0
        xcur = xcur + delta;
    else
        xcur = xcur - delta;
    end
    [fcur, aux] = f(xcur, data);
end
x = xcur;
end

function self = pi_init(k_p, k_t, alpha_i, u_max)
%PI_INIT 2DOF PI controller (PIController), alpha_i being the inverse of the
%   integration time.
self = struct('k_p', k_p, 'k_t', k_t, 'alpha_i', alpha_i, 'u_max', u_max, ...
    'u_i', 0, 'v', 0);
end

function [self, u] = pi_compute_output(self, y_ref, y, u_ff)
%PI_COMPUTE_OUTPUT Limited controller output.
% Estimate of a disturbance input
self.v = self.u_i - (self.k_p - self.k_t)*y + u_ff;
u = self.k_t*(y_ref - y) + self.v;
% Limit the controller output
u = clip(u, -self.u_max, self.u_max);
end

function self = pi_update(self, T_s, u)
%PI_UPDATE Update the integral state with the realized (limited) output u.
self.u_i = self.u_i + T_s*self.alpha_i*(u - self.v);
end

function self = complex_pi_init(k_p, k_i, k_t)
%COMPLEX_PI_INIT 2DOF synchronous-frame complex-vector PI controller
%   (ComplexPIController).
self = struct('k_p', k_p, 'k_t', k_t, 'alpha_i', k_i/k_t, 'u_i', complex(0), ...
    'v', complex(0));
end

function [self, u] = complex_pi_compute_output(self, i_ref, i, u_ff)
%COMPLEX_PI_COMPUTE_OUTPUT Controller output.
% Disturbance input estimate
self.v = self.u_i - (self.k_p - self.k_t)*i + u_ff;
u = self.k_t*(i_ref - i) + self.v;
end

function self = complex_pi_update(self, T_s, u, w_c)
%COMPLEX_PI_UPDATE Update the integral state with the realized output u.
self.u_i = self.u_i + T_s*(self.alpha_i + 1i*w_c)*(u - self.v);
end

function i = current_limiter(i_max, i)
%CURRENT_LIMITER Limit the magnitude of a current vector to i_max (CurrentLimiter).
if abs(i) > i_max
    i = i_max*i/abs(i);
end
end

function self = power_reference_limiter_init(i_d_max, alpha_l, u_nom)
%POWER_REFERENCE_LIMITER_INIT Limiter of the active-power reference
%   (PowerReferenceLimiter): the maximum active power is computed from the voltage
%   reference and the currents, prioritizing the reactive and the negative-sequence
%   currents, and low-pass filtered. The initial maximum active power is that at
%   the nominal grid voltage u_nom.
self = struct('i_d_max', i_d_max, 'alpha_l', alpha_l, 'p_max', 1.5*u_nom*i_d_max);
end

function p_g_ref = power_reference_limiter_compute_output(self, p_g_ref)
%POWER_REFERENCE_LIMITER_COMPUTE_OUTPUT Limited active-power reference (__call__).
p_lim = min(abs(p_g_ref), self.p_max);
if p_g_ref < 0
    p_g_ref = -p_lim;
else
    p_g_ref = p_lim;
end
end

function self = power_reference_limiter_update(self, T_s, u_c, i_c, i_cn)
%POWER_REFERENCE_LIMITER_UPDATE Update the maximum active power.
%   The inputs are the converter voltage reference u_c, the converter current i_c
%   (their positive-sequence components in unbalanced conditions), and the
%   negative-sequence converter current i_cn.
abs_u_c = max(abs(u_c), 1e-6);
i_q = imag(u_c*conj(i_c))/abs_u_c;
i_d_max = max(self.i_d_max - abs(i_cn), 0);
i_d_lim_sqr = max(i_d_max*i_d_max - i_q*i_q, 0);
p_max = 1.5*abs_u_c*sqrt(i_d_lim_sqr);
self.p_max = self.p_max + T_s*self.alpha_l*(p_max - self.p_max);
end

function ctrl = speed_controller(J, alpha_s, alpha_i, tau_M_max)
%SPEED_CONTROLLER 2DOF PI speed controller (SpeedController); alpha_i = NaN means
%   alpha_s.
if isnan(alpha_i)
    alpha_i = alpha_s;
end
k_p = (alpha_s + alpha_i)*J;
k_i = alpha_s*alpha_i*J;
k_t = alpha_s*J;
ctrl = pi_init(k_p, k_t, k_i/k_t, tau_M_max);
end

function self = speed_observer_init(k_w, k_tau, J)
%SPEED_OBSERVER_INIT Speed observer (SpeedObserver); the mechanical system model is
%   used if J > 0.
self = struct('k_w', k_w, 'k_tau', k_tau, 'J', J, 'w_M', 0, 'tau_L', 0);
end

function self = speed_observer_update(self, T_s, eps, tau_M)
%SPEED_OBSERVER_UPDATE Update the speed and load torque estimates.
if self.J <= 0
    d_w_M = self.k_w*eps;
    d_tau_L = 0;
else
    d_w_M = (tau_M - self.tau_L)/self.J + self.k_w*eps;
    d_tau_L = -self.k_tau*eps;
end
self.w_M = self.w_M + T_s*d_w_M;
self.tau_L = self.tau_L + T_s*d_tau_L;
end

function [Phi, Gamma] = hold_equivalent(T_s, w_c)
%HOLD_EQUIVALENT Hold-equivalent discretization of a vector integrator in rotating
%   coordinates (hold_equivalent): the coefficients discretize d(psi)/dt = u + v -
%   1j*w_c*psi, where psi is in coordinates rotating at w_c, u is constant in
%   stator coordinates, and v is constant in the rotating coordinates over the
%   sampling period. Then, psi(k+1) = Phi*(psi(k) + T_s*u(k)) + Gamma*v(k), with
%   Phi = exp(-1j*w_c*T_s) and Gamma = (1 - Phi)/(1j*w_c).
% Half-angle form, requiring a single sine-cosine pair. The sinc function is
% computed as numpy.sinc(x/pi).
x = 0.5*w_c*T_s;
exp_jx = exp(-1i*x);
if x ~= 0
    y = pi*(x/pi);
else
    y = pi*1e-20;
end
Phi = exp_jx*exp_jx;
Gamma = T_s*(sin(y)/y)*exp_jx;
end

function alpha = matched_pole(alpha, T_s)
%MATCHED_POLE Pole for the gain formulas of estimators updated by the forward Euler
%   method (matched_pole): (1 - exp(-alpha*T_s))/T_s, placing the discrete-time
%   poles at exp(-alpha*T_s).
alpha = -expm1(-alpha*T_s)/T_s;
end

function self = rate_limiter_init(rate_limit)
%RATE_LIMITER_INIT Rate limiter (RateLimiter), the falling rate limit being the
%   negative rising one.
self = struct('rate_limit', rate_limit, 'old_y', 0);
end

function y = rate_limiter_compute_output(self, T_s, u)
%RATE_LIMITER_COMPUTE_OUTPUT Limited signal; the state is updated with it by
%   rate_limiter_update.
rate = (u - self.old_y)/T_s;
if rate > self.rate_limit
    y = self.old_y + T_s*self.rate_limit;
elseif rate < -self.rate_limit
    y = self.old_y - T_s*self.rate_limit;
else
    y = u;
end
end

function self = rate_limiter_update(self, y)
%RATE_LIMITER_UPDATE Store the output.
self.old_y = y;
end

function d_err = inverter_nonlinearity(t_d, u_th, i_0, i_abc, d_abc, u_dc, T_half)
%INVERTER_NONLINEARITY Duty-ratio error due to the dead time t_d and the threshold
%   voltage u_th of the power devices (InverterNonlinearity), u_dc being the DC-bus
%   voltage and T_half the half carrier period. The current direction is sign =
%   tanh(i/i_0), or the signum function (numpy.sign) if i_0 = 0.
d = d_abc(:);
% Switching-cycle averaged on-times of the upper and lower switches
% (averaged_gate_signals), the dead time affecting only the switching legs
delta = (t_d/(2*T_half))*(d > 0 & d < 1);
q_hi = max(d - delta, 0);
q_lo = max(1 - d - delta, 0);
b = 1 - q_hi - q_lo;
% Current direction: tanh(i/i_0), or the signum function if i_0 = 0
if i_0 > 0
    s = tanh(i_abc(:)/i_0);
else
    s = sign(i_abc(:));
end
d_err = d - q_hi - 0.5*b.*(1 - s) + s*u_th/u_dc;
end

function self = pwm_init(k_pred)
%PWM_INIT Space-vector PWM (PWM), with the prediction factor k_pred of the currents
%   for the compensation of the duty-ratio error.
%   The overmodulation is MPE or MME (pwm_set_overmodulation, MPE by default).
%   Optionally, the duty-ratio error due to the dead time and the threshold voltage
%   is modeled (pwm_set_nonlinearity), which corresponds to d_err =
%   InverterNonlinearity(t_d, u_th, sign) in motulator with sign = tanh(i/i_0), or
%   numpy.sign if i_0 = 0. The duty ratios are updated twice per carrier period, or
%   once (pwm_set_single_update), which sets the half carrier period of the error
%   model. The angle of the voltage reference is not advanced here: the control
%   systems compensate for the delays. The state d_abc has the duty ratios of the
%   previous and the ongoing periods as its rows, T_half is the half carrier
%   period of the previous period, assumed to apply also to the ongoing one (before
%   the first duty ratios, the legs do not switch, and the infinite period gives no
%   dead-time error), and i_c_ab is the current of the feedforward compensation
%   (see pwm_feedforward_current).
self = struct('k_pred', k_pred, 'mme', 0, 't_d', 0, 'u_th', 0, 'i_0', 0, ...
    'feedforward', 1, 'd_min', 0, 'single_update', 0, 'd_abc', zeros(2, 3), ...
    'T_half', Inf, 'i_c_ab', complex(0));
end

function self = pwm_set_nonlinearity(self, t_d, u_th, i_0, feedforward)
%PWM_SET_NONLINEARITY Duty-ratio error model of the dead time t_d and the threshold
%   voltage u_th (no error model if both are zero) and its compensation.
self.t_d = t_d;
self.u_th = u_th;
self.i_0 = i_0;
self.feedforward = feedforward;
end

function self = pwm_set_overmodulation(self, mme)
%PWM_SET_OVERMODULATION Overmodulation: MME if mme is nonzero, else MPE.
self.mme = mme;
end

function self = pwm_set_min_pulse(self, d_min)
%PWM_SET_MIN_PULSE Minimum duty ratio d_min of a switching leg (d_min of PWM), see
%   pwm_limit_pulses.
self.d_min = d_min;
end

function self = pwm_set_single_update(self, single_update)
%PWM_SET_SINGLE_UPDATE Single update (single_update of PWM) if single_update is
%   nonzero.
self.single_update = single_update;
end

function y = pwm_has_error_model(self)
%PWM_HAS_ERROR_MODEL Error model of the PWM in use (d_err is not None).
y = self.t_d > 0 || self.u_th > 0;
end

function T_half = pwm_half_period(self, T_s)
%PWM_HALF_PERIOD Half carrier period (s) corresponding to the sampling period.
if self.single_update
    T_half = T_s/2;
else
    T_half = T_s;
end
end

function d_abc = pwm_limit_pulses(self, d_abc, d_ref, i_abc, use_i, u_dc, T_half)
%PWM_LIMIT_PULSES Limit the duty ratios of the switching legs to [d_min, 1 - d_min]
%   (limit_pulses): a duty ratio in (0, d_min) is replaced by 0 or d_min, and in
%   (1 - d_min, 1) by 1 - d_min or 1, whichever gives the realized duty ratio nearer
%   to the reference d_ref (the leg switches if equally near). The realized duty
%   ratios are corrected for the duty-ratio error at the currents i_abc, the DC-bus
%   voltage u_dc, and the half carrier period T_half if use_i is true.
candidates = zeros(1, 2); % The leg switching, the leg clamped
for k = 1:3
    if d_abc(k) > 0 && d_abc(k) < self.d_min
        candidates = [self.d_min, 0];
    elseif d_abc(k) > 1 - self.d_min && d_abc(k) < 1
        candidates = [1 - self.d_min, 1];
    else
        continue
    end
    errors = zeros(1, 2);
    for j = 1:2
        d_abc(k) = candidates(j);
        realized = candidates(j);
        if use_i
            d_err = inverter_nonlinearity(self.t_d, self.u_th, self.i_0, i_abc, ...
                d_abc, u_dc, T_half);
            realized = realized - d_err(k);
        end
        errors(j) = abs(realized - d_ref(k));
    end
    if errors(2) < errors(1)
        d_abc(k) = candidates(2);
    else
        d_abc(k) = candidates(1);
    end
end
end

function u_ab = pwm_realized_voltage(self, i_c_ab, u_dc, average)
%PWM_REALIZED_VOLTAGE Realized voltage from the duty ratios and the measured DC-bus
%   voltage, corrected for the duty-ratio error using the measured current and the
%   half carrier period of the previous period (get_realized_voltage): the average
%   of the previous and the ongoing sampling periods if average is true, else that
%   of the ongoing sampling period.
i_abc = complex2abc(i_c_ab);
if average
    k0 = 1;
else
    k0 = 2;
end
q_ab = complex(0);
for k = k0:2
    d = self.d_abc(k, :).';
    if pwm_has_error_model(self)
        d = d - inverter_nonlinearity(self.t_d, self.u_th, self.i_0, i_abc, d, ...
            u_dc, self.T_half);
    end
    q_ab = q_ab + abc2complex(d);
end
u_ab = u_dc*q_ab/(3 - k0);
end

function d_abc = pwm_duty_ratios(u_c_ref_ab, u_dc, mme)
%PWM_DUTY_RATIOS Duty ratios of space-vector PWM with the overmodulation.
u_abc = complex2abc(u_c_ref_ab);
% Zero-sequence voltage resulting in space-vector PWM
u_0 = 0.5*(max(u_abc) + min(u_abc));
u_abc = u_abc - u_0;
% MPE overmodulation (MME results from limiting the duty ratios)
m = (2/u_dc)*max(u_abc);
if ~mme && m > 1
    u_abc = u_abc/m;
end
% Duty ratios
d_abc = max(min(u_abc/u_dc + 0.5, 1), 0);
end

function [u_ab, d_abc] = pwm_compute_output(self, T_s, u_c_ref_ab, u_dc, w, i_c_ab)
%PWM_COMPUTE_OUTPUT Duty ratios and the limited voltage reference.
%   The measured current i_c_ab, predicted by the angle k_pred*T_s*w, is used in the
%   feedforward compensation of the duty-ratio error. With a discharged DC bus
%   (u_dc <= 0), the duty ratios are 0.5.
% A discharged DC bus, e.g., with blocked gates before the precharge
if u_dc <= 0
    d_abc = 0.5*ones(3, 1);
    u_ab = complex(0);
    return
end
% Duty ratios
d_abc = pwm_duty_ratios(u_c_ref_ab, u_dc, self.mme);
d_ref = d_abc;
% Compensate for the duty-ratio error using the predicted currents
compensate = pwm_has_error_model(self) && self.feedforward;
T_half = pwm_half_period(self, T_s);
i_c_abc = zeros(3, 1);
if compensate
    i_c_abc = complex2abc(exp(1i*self.k_pred*T_s*w)*i_c_ab);
    d_err = inverter_nonlinearity(self.t_d, self.u_th, self.i_0, i_c_abc, d_abc, ...
        u_dc, T_half);
    d_abc = clip(d_abc + d_err, 0, 1);
end
% Minimum pulses
if self.d_min > 0
    d_abc = pwm_limit_pulses(self, d_abc, d_ref, i_c_abc, compensate, u_dc, T_half);
end
% Limited voltage reference, including the compensation
u_ab = abc2complex(d_abc)*u_dc;
end

function self = pwm_update(self, T_s, d_abc)
%PWM_UPDATE Store the duty ratios of the next sampling period and the half carrier
%   period of the sampling period T_s.
self.d_abc(1, :) = self.d_abc(2, :);
self.d_abc(2, :) = reshape(d_abc, 1, 3);
self.T_half = pwm_half_period(self, T_s);
end

function i_c_ab = pwm_feedforward_current(self, enabled, i_c_ab)
%PWM_FEEDFORWARD_CURRENT Measured current for the feedforward compensation of
%   pwm_compute_output: i_c_ab, or while the gates are blocked (enabled is false),
%   the current of the latest sampling period with enabled gates, stored by
%   pwm_store_current (in motulator, get_realized_voltage stores the current, and it
%   is not called while the gates are blocked).
if ~enabled
    i_c_ab = self.i_c_ab;
end
end

function self = pwm_store_current(self, i_c_ab)
%PWM_STORE_CURRENT Store the current of the feedforward compensation
%   (pwm_feedforward_current).
self.i_c_ab = i_c_ab;
end
