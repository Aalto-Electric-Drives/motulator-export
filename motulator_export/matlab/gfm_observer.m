% Disturbance-observer-based grid-forming control of grid converters, ported from
% motulator (MATLAB port of the classes of ../c/gfm_observer.c used by the blocks of
% the control system). The functions are embedded in the MATLAB Function blocks by
% motulator_export.simulink.
%
% Python counterpart:
%   motulator/grid/control/_gfm_observer.py  Observer,
%                                            ObserverBasedGridFormingController

function self = gfm_observer_init(u_nom, w_nom, L, alpha_o, T_s, R)
%GFM_OBSERVER_INIT Discrete-time disturbance observer (Observer), with the
%   coefficients of the hold-equivalent model and the observer gain.
phi = exp(-1i*w_nom*T_s);
gamma_g = (phi - 1)/(1i*w_nom*L);
self = struct('L', L, 'R', R, 'w_g', w_nom, 'phi', phi, 'gamma_c', T_s*phi/L, ...
    'gamma_g', gamma_g, 'k_o', (1 - exp(-alpha_o*T_s))/gamma_g, 'theta_c', 0, ...
    'u_g', complex(u_nom), 'i_c', complex(0));
end

function out = gfm_observer_compute_output(self, u_c_ab, i_c_ab)
%GFM_OBSERVER_COMPUTE_OUTPUT Feedback signals from the converter voltage of the
%   ongoing sampling period and the converter current in stationary coordinates
%   (the field u_c is zero, and u_dc is not included).
theta_c = self.theta_c;
% Measured values in synchronous coordinates
rot = exp(-1i*theta_c);
i_c = rot*i_c_ab;
u_c_zoh = rot*u_c_ab;
% Correct the predicted grid-voltage estimate
u_g = self.u_g + self.k_o*(i_c - self.i_c);
v_c = u_g + 1i*self.w_g*self.L*i_c;
% Active and reactive powers
s_g = 1.5*u_g*conj(i_c);
out = struct('i_c', i_c, 'u_c', complex(0), 'u_c_zoh', u_c_zoh, 'v_c', v_c, ...
    'u_g', u_g, 'p_g', real(s_g), 'q_g', imag(s_g), 'theta_c', theta_c, ...
    'w_c', self.w_g);
end

function self = gfm_observer_update(self, T_s, out)
%GFM_OBSERVER_UPDATE Predict the current, the grid voltage being constant in these
%   coordinates.
self.i_c = self.phi*out.i_c + self.gamma_c*out.u_c_zoh ...
    + self.gamma_g*(out.u_g + self.R*out.i_c);
self.u_g = out.u_g;
self.theta_c = wrap(self.theta_c + T_s*self.w_g);
end

function ref = gfm_current_reference(R_a, k_v, k_c, p_g_ref, v_c_ref, fbk)
%GFM_CURRENT_REFERENCE References p_g and v_c and the current reference i_c of the
%   transparent current limitation (ObserverBasedGridFormingController.
%   compute_output before the CurrentLimiter), R_a being the active resistance, k_v
%   the voltage control gain, and k_c the current control gain.
% Complex gains and the feedback correction for the grid-forming mode
exp_j_theta = exp(1i*angle(fbk.v_c));
k_p = exp_j_theta*R_a/(1.5*v_c_ref);
k_vc = exp_j_theta*(1 - 1i*k_v);
e_c = k_p*(p_g_ref - fbk.p_g) + k_vc*(v_c_ref - abs(fbk.v_c));
% Current reference of the transparent current limitation
ref = struct('p_g', p_g_ref, 'v_c', v_c_ref, 'i_c', fbk.i_c + e_c/k_c);
end

function u_c_ref = gfm_voltage_reference(k_c, R, i_c_ref, fbk)
%GFM_VOLTAGE_REFERENCE Converter voltage reference from the limited current
%   reference (ObserverBasedGridFormingController.compute_output after the
%   CurrentLimiter), R being the total series resistance estimate.
e_c = k_c*(i_c_ref - fbk.i_c);
u_c_ref = e_c + fbk.v_c + R*fbk.i_c;
end
