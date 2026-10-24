% Flux-vector control and observer-based V/Hz control of induction machine drives,
% ported from motulator (MATLAB port of the classes of ../c/im_flux_vector.c used by
% the blocks of the control systems). The functions are copied into the MATLAB
% functions of the blocks (mfun_*.m) by motulator_export.simulink.
%
% Python counterparts:
%   motulator/drive/control/_im_flux_vector.py  ReferenceGenerator,
%                                               FluxTorqueController
%
% Pure open-loop V/Hz control is the special case L_M = inf of the machine model
% (with R_s = R_R = L_sgm = 0), see im_current_vector.m for the observer.

function y = py_min(a, b)
%PY_MIN Minimum as the built-in min of Python: the first argument unless the second
%   one is smaller, so that a NaN argument is skipped unless it is the first one.
if b < a
    y = b;
else
    y = a;
end
end

function y = py_max(a, b)
%PY_MAX Maximum as the built-in max of Python, see py_min.
if b > a
    y = b;
else
    y = a;
end
end

function self = im_flux_torque_ctrl_init(par, alpha_psi, alpha_tau, alpha_i)
%IM_FLUX_TORQUE_CTRL_INIT Flux and torque controller (FluxTorqueController) with the
%   integral action of the bandwidth alpha_i (zero in observer-based V/Hz control).
self = struct('par', par, 'alpha_psi', alpha_psi, 'alpha_tau', alpha_tau, ...
    'alpha_i', alpha_i, 'x_psi', 0, 'x_tau', 0, 'i_a', complex(0), ...
    'u_ff', complex(0));
end

function [self, u_s_ref] = im_flux_torque_ctrl_compute_output(self, psi_s_ref, ...
    tau_M_ref, fbk)
%IM_FLUX_TORQUE_CTRL_COMPUTE_OUTPUT Stator voltage reference from the flux and
%   torque references.
par = self.par;
psi_s_abs = abs(fbk.psi_s);
% Auxiliary current (L_sgm = 0 in open-loop V/Hz control, where only the flux is
% controlled)
i_a = complex(0);
if par.L_sgm > 0
    i_a = fbk.psi_R/par.L_sgm;
end
% Torque-production factor and directions
c_tau = 1.5*par.n_p*real(i_a*conj(fbk.psi_s));
t_psi = complex(1);
t_tau = complex(0);
if c_tau > 0
    t_psi = 1.5*par.n_p*psi_s_abs*i_a/c_tau;
    t_tau = 1i*fbk.psi_s/c_tau;
end
% Error signals
e_psi = psi_s_ref - psi_s_abs;
e_tau = tau_M_ref - fbk.tau_M;
e_u = self.alpha_psi*e_psi*t_psi + self.alpha_tau*e_tau*t_tau;
u_i = self.x_psi*t_psi + self.x_tau*t_tau;
e_v = u_i - self.alpha_i*(psi_s_abs*t_psi + fbk.tau_M*t_tau);
% Voltage reference, consisting of the feedforward and integral action (u_ff) and
% the proportional action (e_u)
u_ff = par.R_s*fbk.i_s + 1i*fbk.w_s*fbk.psi_s + e_v;
% Workspace variables for the update
self.i_a = i_a;
self.u_ff = u_ff;
u_s_ref = u_ff + e_u;
end

function self = im_flux_torque_ctrl_update(self, T_s, fbk)
%IM_FLUX_TORQUE_CTRL_UPDATE Update the integral states with the realized voltage
%   fbk.u_s.
psi_s_abs = abs(fbk.psi_s);
% Error signal and gains
u_err = fbk.u_s - self.u_ff;
k_psi = 0;
if psi_s_abs > 0
    k_psi = self.alpha_i/psi_s_abs;
end
k_tau = 1.5*self.par.n_p*self.alpha_i;
% Update the integral states
self.x_psi = self.x_psi + T_s*k_psi*real(fbk.psi_s*conj(u_err));
self.x_tau = self.x_tau + T_s*k_tau*real(1i*self.i_a*conj(u_err));
end

function self = im_flux_reference_gen_init(par, psi_s_nom, i_s_max, tau_M_max, k_u, ...
    k_b)
%IM_FLUX_REFERENCE_GEN_INIT Reference generator with field weakening and torque
%   limits (ReferenceGenerator).
self = struct('par', par, 'psi_s_nom', psi_s_nom, 'i_s_max', i_s_max, ...
    'tau_M_max', tau_M_max, 'k_u', k_u, 'k_b', k_b);
end

function [psi_s_ref, tau_M_ref] = im_reference_gen_flux_and_torque(self, tau_M_ref, ...
    w_s, psi_R, u_dc)
%IM_REFERENCE_GEN_FLUX_AND_TORQUE Stator flux reference and limited torque
%   reference.
par = self.par;
% Field weakening
u_s_max = self.k_u*u_dc/sqrt(3);
if w_s ~= 0
    psi_s_max = u_s_max/abs(w_s);
else
    psi_s_max = Inf;
end
% Current limit
psi_s_cl = par.L_sgm*self.i_s_max + psi_R;
% Stator flux reference
psi = py_min(py_min(psi_s_max, self.psi_s_nom), psi_s_cl);
% Torque reference limiting
if par.L_sgm > 0 % L_sgm = 0 in open-loop V/Hz control
    % Breakdown torque limit
    k = self.k_b*par.L_M/(par.L_M + par.L_sgm);
    tau_b = 0.75*par.n_p*k*(psi*psi)/par.L_sgm;
    % Current limit: the stator flux on the circle psi stays within the circle
    % L_sgm*i_s_max around the rotor flux, which gives the maximum q-component of
    % the stator flux in rotor flux coordinates
    tau_M_cl = 0;
    if psi_R > 0
        psi_cl = par.L_sgm*self.i_s_max;
        psi_sd = (psi*psi + psi_R*psi_R - psi_cl*psi_cl)/(2*psi_R);
        psi_sd = py_max(psi_sd, 0);
        psi_sq = sqrt(py_max(psi*psi - psi_sd*psi_sd, 0));
        tau_M_cl = 1.5*par.n_p*psi_R*psi_sq/par.L_sgm;
    end
    % Limited torque reference
    tau = py_min(py_min(py_min(abs(tau_M_ref), self.tau_M_max), tau_b), tau_M_cl);
    tau_M_ref = tau*sign(tau_M_ref);
end
psi_s_ref = psi;
end
