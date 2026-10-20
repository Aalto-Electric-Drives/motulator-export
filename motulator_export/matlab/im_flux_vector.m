% Observer-based V/Hz control of induction machine drives, ported from motulator
% (MATLAB port of the classes of ../c/im_flux_vector.c used by the blocks of the
% control system). The functions are copied into the MATLAB functions of the
% blocks (mfun_*.m) by motulator_export.simulink.
%
% Python counterparts:
%   motulator/drive/control/_im_vhz.py         ReferenceGenerator,
%                                              FluxTorqueController
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

function self = im_flux_torque_ctrl_init(par, alpha_psi, alpha_tau)
%IM_FLUX_TORQUE_CTRL_INIT Flux and torque controller (FluxTorqueController) without
%   the integral action.
self = struct('par', par, 'alpha_psi', alpha_psi, 'alpha_tau', alpha_tau);
end

function u_s_ref = im_flux_torque_ctrl_compute_output(self, psi_s_ref, tau_M_ref, fbk)
%IM_FLUX_TORQUE_CTRL_COMPUTE_OUTPUT Stator voltage reference from the flux and
%   torque references.
par = self.par;
% Auxiliary current, torque-production factor, and directions
t_psi = complex(1);
t_tau = complex(0);
if par.L_sgm > 0 % L_sgm = 0 in open-loop V/Hz control
    i_a = fbk.psi_R/par.L_sgm;
    c_tau = 1.5*par.n_p*real(i_a*conj(fbk.psi_s));
    if c_tau > 0
        t_psi = 1.5*par.n_p*abs(fbk.psi_s)*i_a/c_tau;
        t_tau = 1i*fbk.psi_s/c_tau;
    end
end
% Error signals (the integral action is not used: alpha_i = 0)
e_psi = psi_s_ref - abs(fbk.psi_s);
e_tau = tau_M_ref - fbk.tau_M;
e_u = self.alpha_psi*e_psi*t_psi + self.alpha_tau*e_tau*t_tau;
% Voltage reference
v = par.R_s*fbk.i_s + 1i*fbk.w_s*fbk.psi_s;
u_s_ref = v + e_u;
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
    % Current limit
    i_m = psi_R/par.L_M;
    tau_M_cl = 0;
    if psi_R < par.L_M*self.i_s_max
        i_q_max = sqrt(self.i_s_max*self.i_s_max - i_m*i_m);
        tau_M_cl = 1.5*par.n_p*psi_R*i_q_max;
    end
    % Limited torque reference
    tau = py_min(py_min(py_min(abs(tau_M_ref), self.tau_M_max), tau_b), tau_M_cl);
    tau_M_ref = tau*sign(tau_M_ref);
end
psi_s_ref = psi;
end
