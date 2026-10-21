% Sensorless flux-vector control of synchronous machine drives, ported from
% motulator (MATLAB port of the classes of ../c/sm_flux_vector.c used by the blocks
% of the control system). The functions are copied into the MATLAB functions of the
% blocks (mfun_*.m) by motulator_export.simulink.
%
% Python counterparts:
%   motulator/drive/control/_sm_observers.py      FluxObserver
%   motulator/drive/control/_sm_reference_gen.py  ReferenceGenerator (LUT-based)
%   motulator/drive/control/_sm_flux_vector.py    FluxTorqueController
%
% As in motulator, the output functions are free of side effects (apart from
% workspace variables), and the states are updated in the update functions.

function self = flux_observer_init(par, k_theta, k_o)
%FLUX_OBSERVER_INIT Flux observer in estimated rotor coordinates (FluxObserver),
%   with the observer gain k_o(w_m) = k_o(1) + k_o(2)*abs(w_m).
self = struct('par', par, 'k_theta', k_theta, 'k_o', [k_o(1), k_o(2)], ...
    'theta_m', 0, 'psi_s', complex(par.psi_f), 'T_s_prev', 0);
end

function out = flux_observer_compute_output(self, u_s_ab, i_s_ab, w_M, eps_ext, h)
%FLUX_OBSERVER_COMPUTE_OUTPUT Feedback signals (ObserverOutputs) without u_dc and
%   tau_L. The weight h of the external position error signal eps_ext selects
%   between the sensorless (h = 0) and sensored (h = 1) modes. The voltage u_s_ab is
%   NaN if unknown (blocked gates).
par = self.par;
% Coordinate system angle equals the estimated rotor angle
theta_c = self.theta_m;
% Current vector in (estimated) rotor coordinates. The voltage averaged over the
% previous and ongoing sampling periods is set by the control system
% (DriveControlSystem.get_feedback).
i_s = exp(-1i*theta_c)*i_s_ab;
% Flux estimation error
psi_s_model = psi_s_dq(par, i_s);
e_o = psi_s_model - self.psi_s;
% Auxiliary flux
L = incr_ind_mat(par, i_s);
psi_a = aux_flux(L, i_s, psi_s_model);
% Error signals for the rotor angle and PM-flux estimation
if psi_a ~= 0
    ratio = e_o/psi_a;
else
    ratio = complex(0);
end
eps = h*eps_ext - (1 - h)*imag(ratio)/par.n_p;
eps_f = -(1 - h)*real(ratio);
% Rotor speed and the angular speed of the coordinate system
w_m = par.n_p*w_M;
w_c = w_m + self.k_theta*par.n_p*eps;
% Voltage vector in the observer coordinates, or the open-circuit voltage of the
% model if the voltage is unknown (NaN, blocked gates): the back-EMF of the flux
% estimate at the rotor speed, averaged over the sampling period (that of the
% previous period), and the resistive voltage drop
if ~isnan(real(u_s_ab))
    u_s_zoh = exp(-1i*theta_c)*u_s_ab;
elseif self.T_s_prev > 0
    emf = (exp(1i*w_m*self.T_s_prev) - 1)/self.T_s_prev*self.psi_s;
    u_s_zoh = emf + par.R_s*i_s;
else
    u_s_zoh = 1i*w_m*self.psi_s + par.R_s*i_s;
end
% Torque estimate
tau_M = 1.5*par.n_p*imag(i_s*conj(self.psi_s));
out = struct('i_s', i_s, 'u_s', complex(0), 'u_s_zoh', u_s_zoh, ...
    'psi_s', self.psi_s, 'e_o', e_o, 'eps', eps, 'eps_f', eps_f, 'psi_a', psi_a, ...
    'tau_M', tau_M, 'w_c', w_c, 'w_m', w_m, 'w_M', w_M, 'theta_c', theta_c, ...
    'theta_m', self.theta_m, 'psi_f', par.psi_f, 'h', h);
end

function self = flux_observer_update(self, T_s, out)
%FLUX_OBSERVER_UPDATE Update the flux and rotor angle estimates.
par = self.par;
k_o1 = self.k_o(1) + self.k_o(2)*abs(out.w_m);
if out.psi_a ~= 0
    k_o2 = (1 - out.h)*(k_o1*out.psi_a/conj(out.psi_a));
else
    k_o2 = complex((1 - out.h)*k_o1);
end
% Correct the flux estimate at the sampling instant and predict it to the next
% sampling instant. The stator voltage is constant in stator coordinates, while the
% resistive voltage drop is constant in the observer coordinates. The PM-flux
% estimation gain k_f is zero.
v_err = k_o1*out.e_o + k_o2*conj(out.e_o);
psi_s = out.psi_s + T_s*v_err;
[Phi, Gamma] = hold_equivalent(T_s, out.w_c);
self.psi_s = Phi*(psi_s + T_s*out.u_s_zoh) - Gamma*par.R_s*out.i_s;
self.theta_m = wrap(self.theta_m + T_s*out.w_c);
self.T_s_prev = T_s;
end

function [caught, w_M, theta_m, psi_s] = flux_observer_catch(par, delta_i_s_ab, ...
    i_s_ab, T_s, w_M)
%FLUX_OBSERVER_CATCH Estimates initialized from the current response delta_i_s_ab to
%   a zero-voltage period of length T_s, i_s_ab being the current at its end
%   (SpeedFluxObserver.catch): the back-EMF drives the current along the negative q
%   axis of the rotor, which gives the rotor angle and the speed magnitude, the
%   direction of rotation being taken from the speed estimate w_M, which is updated
%   with the angle and flux estimates. If the back-EMF estimate does not exceed the
%   resistive voltage drop of the current (standstill, or a current that has not
%   decayed) or the machine has no magnets, caught is false and the estimates are
%   not valid.
L = incr_ind_mat(par, complex(0));
e_s_ab = -L.L_qq*delta_i_s_ab/T_s; % Back-EMF
% The resistive decay of a residual current (the d-axis current through L_d) must
% not be taken for the back-EMF
i_max = max(abs(i_s_ab), abs(i_s_ab - delta_i_s_ab));
caught = abs(e_s_ab) > par.R_s*i_max*max(L.L_qq/L.L_dd, 1) && par.psi_f ~= 0;
theta_m = 0;
psi_s = complex(0);
if ~caught
    return
end
if w_M >= 0
    sign_w = 1;
else
    sign_w = -1;
end
w_m = sign_w*abs(e_s_ab)/par.psi_f;
% The response lags the rotor angle at the end of the period, since the back-EMF
% rotates during the period
theta_m = wrap(angle(e_s_ab) - sign_w*pi/2 + w_m*T_s*L.L_qq/(2*L.L_dd));
w_M = w_m/par.n_p;
psi_s = complex(psi_s_dq(par, exp(-1i*theta_m)*i_s_ab));
end

function self = reference_gen_init(par, psi_s_min, psi_s_max, k_u, k_mtpv, luts)
%REFERENCE_GEN_INIT Flux and torque reference generator based on precomputed lookup
%   tables (ReferenceGenerator); psi_s_min = NaN means psi_f. The tables luts are
%   the 16-by-6 matrix of compute_reference_luts (sm_control_loci.m), computed
%   outside the block by sm_reference_luts.m in the mask initialization.
if isnan(psi_s_min)
    psi_s_min = par.psi_f;
end
self = struct('k_u', k_u, 'k_mtpv', k_mtpv, 'psi_s_min', psi_s_min, ...
    'psi_s_max', psi_s_max, ...
    'psi_s_mtpa', struct('x', luts(:, 1), 'y', luts(:, 2)), ...
    'tau_M_cl', struct('x', luts(:, 3), 'y', luts(:, 4)), ...
    'tau_M_mtpv', struct('x', luts(:, 5), 'y', luts(:, 6)));
end

function psi = reference_gen_max_flux(self, w_m, u_dc)
%REFERENCE_GEN_MAX_FLUX Maximum flux (field weakening).
u_s_max = self.k_u*u_dc/sqrt(3);
if w_m ~= 0
    psi = u_s_max/abs(w_m);
else
    psi = Inf;
end
end

function [psi_s_ref, tau_M_ref] = reference_gen_compute_flux_and_torque_refs( ...
    self, tau_M_ref, w_m, u_dc)
%REFERENCE_GEN_COMPUTE_FLUX_AND_TORQUE_REFS Flux reference and limited torque
%   reference.
% MTPA flux
psi_s_abs_ref = clip(lut_interp(self.psi_s_mtpa, abs(tau_M_ref)), ...
    self.psi_s_min, self.psi_s_max);
% Maximum flux (field weakening)
psi_s_abs_ref = min(psi_s_abs_ref, reference_gen_max_flux(self, w_m, u_dc));
% Current limit
tau_M_cl = lut_interp(self.tau_M_cl, psi_s_abs_ref);
tau_M_ref = min(tau_M_cl, abs(tau_M_ref))*sign(tau_M_ref);
% MTPV limit
tau_M_mtpv = lut_interp(self.tau_M_mtpv, psi_s_abs_ref);
if tau_M_mtpv > 0
    tau_M_ref = min(self.k_mtpv*tau_M_mtpv, abs(tau_M_ref))*sign(tau_M_ref);
end
psi_s_ref = psi_s_abs_ref;
end

function self = flux_torque_ctrl_init(par, alpha_psi, alpha_tau, alpha_i)
%FLUX_TORQUE_CTRL_INIT Flux and torque controller (FluxTorqueController).
self = struct('par', par, 'alpha_psi', alpha_psi, 'alpha_tau', alpha_tau, ...
    'alpha_i', alpha_i, 'x_psi', alpha_i*par.psi_f, 'x_tau', 0, ...
    'i_a', complex(0), 'u_ff', complex(0));
end

function [self, u_s_ref] = flux_torque_ctrl_compute_output(self, psi_s_ref, ...
    tau_M_ref, fbk)
%FLUX_TORQUE_CTRL_COMPUTE_OUTPUT Stator voltage reference from the flux and torque
%   references.
par = self.par;
psi_s_abs = abs(fbk.psi_s);
% Auxiliary current and torque-production factor
L = incr_ind_mat(par, fbk.i_s);
i_a = aux_current(L, fbk.psi_s, fbk.i_s);
c_tau = 1.5*par.n_p*real(i_a*conj(fbk.psi_s));
% Directions
if c_tau > 0
    t_psi = 1.5*par.n_p*psi_s_abs*i_a/c_tau;
    t_tau = 1i*fbk.psi_s/c_tau;
else
    t_psi = complex(1);
    t_tau = complex(0);
end
% If c_tau < 0, the torque cannot be controlled. In a PM machine with weak PMs, this
% occurs at a high flux and a small load angle (psi_s.real > 0), where the flux is
% reduced until the torque becomes controllable again. Beyond the MTPV limit
% (psi_s.real < 0 for PM machines), reducing the flux would not help.
if c_tau < 0 && par.psi_f > 0 && real(fbk.psi_s) > 0
    psi_s_ref = 0;
end
% Error signals
e_psi = psi_s_ref - psi_s_abs;
e_tau = tau_M_ref - fbk.tau_M;
e_u = self.alpha_psi*e_psi*t_psi + self.alpha_tau*e_tau*t_tau;
u_i = self.x_psi*t_psi + self.x_tau*t_tau;
e_v = u_i - self.alpha_i*(psi_s_abs*t_psi + fbk.tau_M*t_tau);
% Voltage reference, consisting of the feedforward and integral action (u_ff) and
% the proportional action (e_u)
u_ff = par.R_s*fbk.i_s + 1i*fbk.w_m*fbk.psi_s + e_v;
% Workspace variables for the update function
self.i_a = i_a;
self.u_ff = u_ff;
u_s_ref = u_ff + e_u;
end

function self = flux_torque_ctrl_update(self, T_s, fbk)
%FLUX_TORQUE_CTRL_UPDATE Update the integral states.
par = self.par;
psi_s_abs = abs(fbk.psi_s);
% Error signal and gains
u_err = fbk.u_s - self.u_ff;
if psi_s_abs > 0
    k_psi = self.alpha_i/psi_s_abs;
else
    k_psi = 0;
end
k_tau = 1.5*par.n_p*self.alpha_i;
% Update the integral states
self.x_psi = self.x_psi + T_s*k_psi*real(fbk.psi_s*conj(u_err));
self.x_tau = self.x_tau + T_s*k_tau*real(1i*self.i_a*conj(u_err));
end
