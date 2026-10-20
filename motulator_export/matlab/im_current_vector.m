% Current-vector control of induction machine drives, ported from motulator (MATLAB
% port of the classes of ../c/im_current_vector.c used by the blocks of the control
% systems). The functions are copied into the MATLAB functions of the blocks
% (mfun_*.m) by motulator_export.simulink.
%
% Python counterparts:
%   motulator/drive/utils/_parameters.py        InductionMachineInvGammaPars
%   motulator/drive/control/_im_observers.py    FluxObserver
%   motulator/drive/control/_im_current_vector.py  CurrentReferenceGenerator,
%                                               CurrentController
%
% The machine model par is a struct with the fields n_p, R_s, R_R, L_sgm, and L_M
% (InductionMachineInvGammaPars).

function alpha = im_alpha(par)
%IM_ALPHA Inverse rotor time constant (alpha of InductionMachineInvGammaPars).
alpha = par.R_R/par.L_M;
end

function w_rb = im_w_rb(par)
%IM_W_RB Breakdown slip angular frequency (w_rb of InductionMachineInvGammaPars).
if par.L_sgm > 0
    w_rb = par.R_R*(1/par.L_sgm + 1/par.L_M);
else
    w_rb = 0;
end
end

function self = im_flux_observer_init(par, sensorless)
%IM_FLUX_OBSERVER_INIT Reduced-order flux observer in synchronous coordinates
%   (FluxObserver). The flag sensorless selects the default observer gain k_o(w_m),
%   and unit_gain = true gives k_o = 1 (create_vhz_observer with L_M = inf).
self = struct('par', par, 'sensorless', sensorless, 'unit_gain', false, ...
    'psi_s', complex(0), 'theta_c', 0, 'T_s_old', 0, 'psi_R_pred', complex(0));
end

function k_o = im_observer_gain(self, w_m)
%IM_OBSERVER_GAIN Default observer gain of create_speed_flux_observer
%   (default_sensorless_k_o in the sensorless mode).
alpha = im_alpha(self.par);
if self.unit_gain
    k_o = complex(1);
elseif self.sensorless
    k_o = (0.5*alpha + 0.2*abs(w_m))/(alpha - 1i*w_m);
else
    k_o = 1 + 0.2*abs(w_m)/(alpha - 1i*w_m);
end
end

function out = im_flux_observer_compute_output(self, u_s_ab, i_s_ab, w_M, eps_ext, h)
%IM_FLUX_OBSERVER_COMPUTE_OUTPUT Feedback signals (ObserverOutputs) without u_dc and
%   tau_L. The weight h of the external speed error signal eps_ext selects between
%   the sensorless (h = 0) and sensored (h = 1) modes.
par = self.par;
psi_s = self.psi_s;
theta_c = self.theta_c;
% Current and voltage vectors in synchronous coordinates. The voltage averaged over
% the previous and ongoing sampling periods is set by the control system
% (DriveControlSystem.get_feedback).
i_s = exp(-1i*theta_c)*i_s_ab;
u_s_zoh = exp(-1i*theta_c)*u_s_ab;
% Mechanical and electrical angular speeds of the rotor
w_m = par.n_p*w_M;
% Rotor flux estimate
psi_R = psi_s - par.L_sgm*i_s;
% Slip angular frequency
prod = psi_s*conj(psi_R);
if real(prod) > 0
    w_r = im_w_rb(par)*imag(prod)/real(prod);
else
    w_r = 0;
end
% Angular speed of the coordinate system equals synchronous angular frequency
w_s = w_m + w_r;
% Estimation error, i.e., the difference between the rotor flux predictions of the
% current model and the voltage model
if self.T_s_old > 0
    e_o = (self.psi_R_pred - psi_R)/self.T_s_old;
else
    e_o = complex(0);
end
% Torque estimate
if par.L_sgm > 0
    tau_M = 1.5*par.n_p*imag(i_s*conj(psi_s));
else
    tau_M = 0;
end
% Rotor speed error signal for the speed observer. The model-based part is faded
% out as the external error signal takes over.
if abs(psi_R) > 0
    eps_o = -imag(e_o/psi_R)/par.n_p;
else
    eps_o = 0;
end
eps = h*eps_ext + (1 - h)*eps_o;
out = struct('i_s', i_s, 'u_s', complex(0), 'u_s_zoh', u_s_zoh, 'psi_s', psi_s, ...
    'psi_R', psi_R, 'tau_M', tau_M, 'w_c', w_s, 'w_s', w_s, 'w_r', w_r, ...
    'w_m', w_m, 'w_M', w_M, 'theta_c', theta_c, 'e_o', e_o, 'eps', eps, 'h', h);
end

function self = im_flux_observer_update(self, T_s, out)
%IM_FLUX_OBSERVER_UPDATE Update the flux estimate and the coordinate angle.
par = self.par;
% Observer gains. The conjugate-error gain is faded out together with the
% model-based error signal.
k_o1 = im_observer_gain(self, out.w_m);
if abs(out.psi_R) > 0
    proj = out.psi_R/conj(out.psi_R);
else
    proj = complex(0);
end
k_o2 = (1 - out.h)*k_o1*proj;
% Correct the flux estimate at the sampling instant. The estimation error has
% accumulated over the previous sampling period, whose length scales it.
v_err = k_o1*out.e_o + k_o2*conj(out.e_o);
psi_s = out.psi_s + self.T_s_old*v_err;
psi_R = psi_s - par.L_sgm*out.i_s;
% Predict the voltage model and the current model to the next sampling instant. The
% stator voltage is constant in stator coordinates, while the other terms are
% constant in the observer coordinates.
[Phi, Gamma] = hold_equivalent(T_s, out.w_c);
self.psi_s = Phi*(psi_s + T_s*out.u_s_zoh) - Gamma*par.R_s*out.i_s;
[Phi, Gamma] = hold_equivalent(T_s, out.w_c - out.w_m);
self.psi_R_pred = Phi*psi_R + Gamma*(par.R_R*out.i_s - im_alpha(par)*psi_R);
self.theta_c = wrap(self.theta_c + T_s*out.w_c);
% Sampling period for the next sampling period
self.T_s_old = T_s;
end

function self = im_reference_gen_init(par, psi_s_nom, i_s_max, w_s_nom, k_u, k_fw)
%IM_REFERENCE_GEN_INIT Current reference generator with field weakening
%   (CurrentReferenceGenerator); k_fw = NaN means 2*R_R/(w_s_nom*L_sgm^2).
i_sd_nom = psi_s_nom/(par.L_M + par.L_sgm);
if isnan(k_fw)
    k_fw = 2*par.R_R/(w_s_nom*par.L_sgm*par.L_sgm);
end
self = struct('par', par, 'i_s_max', i_s_max, 'k_u', k_u, 'i_sd_nom', i_sd_nom, ...
    'k_fw', k_fw, 'i_sd_ref', i_sd_nom);
end

function [i_s_ref, tau_M_ref_lim] = im_reference_gen_compute_output(self, ...
    tau_M_ref, psi_R)
%IM_REFERENCE_GEN_COMPUTE_OUTPUT Current reference in rotor flux coordinates and
%   the limited torque reference, given the rotor flux magnitude psi_R.
par = self.par;
i_sd_ref = self.i_sd_ref;
% q-axis current reference
if psi_R > 0
    i_sq_ref = tau_M_ref/(1.5*par.n_p*psi_R);
else
    i_sq_ref = 0;
end
% q-axis current limit: priority given to the d component, and the breakdown torque
% limit
i_sq_max1 = sqrt(self.i_s_max*self.i_s_max - i_sd_ref*i_sd_ref);
i_sq_max2 = psi_R/par.L_sgm + i_sd_ref;
i_sq_max = min(i_sq_max1, i_sq_max2);
i_sq_ref = clip(i_sq_ref, -i_sq_max, i_sq_max);
i_s_ref = complex(i_sd_ref, i_sq_ref);
% Limited torque (for the speed controller)
tau_M_ref_lim = 1.5*par.n_p*psi_R*i_sq_ref;
end

function self = im_reference_gen_update(self, T_s, u_s_ref, u_dc)
%IM_REFERENCE_GEN_UPDATE Field weakening based on the unlimited reference voltage.
u_s_max = self.k_u*u_dc/sqrt(3);
self.i_sd_ref = self.i_sd_ref + T_s*self.k_fw*(u_s_max - abs(u_s_ref));
self.i_sd_ref = clip(self.i_sd_ref, -self.i_s_max, self.i_sd_nom);
end

function ctrl = im_current_controller(par, alpha_c, alpha_i)
%IM_CURRENT_CONTROLLER Current controller (CurrentController), a 2DOF PI controller
%   with the gains from the bandwidths and the leakage inductance.
k_t = alpha_c*par.L_sgm;
k_i = alpha_c*alpha_i*par.L_sgm;
k_p = (alpha_c + alpha_i)*par.L_sgm;
ctrl = complex_pi_init(k_p, k_i, k_t);
end
