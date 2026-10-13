% Grid-following control of grid converters, ported from motulator (MATLAB port of
% the classes of ../c/gfl_current_vector.c used by the blocks of the control
% system). The functions are embedded in the MATLAB Function blocks by
% motulator_export.simulink.
%
% Python counterparts:
%   motulator/grid/control/_common.py            PLL
%   motulator/grid/control/_gfl_current_vector.py  CurrentController,
%                                                CurrentVectorController

function self = pll_init(u_nom, w_nom, alpha_pll)
%PLL_INIT Phase-locked loop with the voltage-magnitude filtering (PLL).
self = struct('k_p', 2*alpha_pll, 'k_i', alpha_pll*alpha_pll, 'w_g', w_nom, ...
    'theta_c', 0, 'u_g', u_nom);
end

function out = pll_compute_output(self, i_c_ab, u_g_ab)
%PLL_COMPUTE_OUTPUT Feedback signals (PLLOutputs) from the converter current and the
%   PCC voltage in stationary coordinates (the field u_c is zero, and u_dc is not
%   included).
theta_c = self.theta_c;
w_g = self.w_g;
% Coordinate transformations
rot = exp(-1i*theta_c);
i_c = rot*i_c_ab;
u_g_meas = rot*u_g_ab;
% Filtered voltage magnitude
u_g = self.u_g;
% Error signal and the angular speed of the coordinate system
if self.u_g > 0
    eps = imag(u_g_meas)/self.u_g;
else
    eps = 0;
end
w_c = w_g + self.k_p*eps;
% Powers
s_g = 1.5*u_g*conj(i_c);
out = struct('i_c', i_c, 'u_c', complex(0), 'u_g', u_g, 'u_g_meas', u_g_meas, ...
    'theta_c', theta_c, 'w_g', w_g, 'w_c', w_c, 'p_g', real(s_g), ...
    'q_g', imag(s_g), 'eps', eps);
end

function self = pll_update(self, T_s, out)
%PLL_UPDATE Update the angle, the frequency, and the voltage-magnitude estimates.
self.theta_c = wrap(self.theta_c + T_s*out.w_c);
self.w_g = self.w_g + T_s*self.k_i*out.eps;
self.u_g = self.u_g + T_s*self.k_p*(real(out.u_g_meas) - self.u_g);
end

function ctrl = gfl_current_controller(L, alpha_c, alpha_i)
%GFL_CURRENT_CONTROLLER Current controller (CurrentController), a 2DOF PI
%   controller with the gains from the bandwidths and the inductance (alpha_i = NaN
%   means alpha_c).
if isnan(alpha_i)
    alpha_i = alpha_c;
end
k_t = alpha_c*L;
k_i = alpha_c*alpha_i*L;
k_p = (alpha_c + alpha_i)*L;
ctrl = complex_pi_init(k_p, k_i, k_t);
end

function i_c_ref = gfl_current_reference(p_g_ref, q_g_ref, u_g)
%GFL_CURRENT_REFERENCE Current reference from the power references and the filtered
%   PCC voltage magnitude u_g (CurrentVectorController.compute_output, before the
%   CurrentLimiter).
i_c_ref = (p_g_ref - 1i*q_g_ref)/(1.5*u_g);
end

function [p_g_ref, q_g_ref] = gfl_power_references(i_c_ref, u_g)
%GFL_POWER_REFERENCES Power references corresponding to the limited current
%   reference i_c_ref (CurrentVectorController.compute_output, after the
%   CurrentLimiter), which are also used for the anti-windup of the DC-bus voltage
%   controller.
s_g_ref = 1.5*u_g*conj(i_c_ref);
p_g_ref = real(s_g_ref);
q_g_ref = imag(s_g_ref);
end
