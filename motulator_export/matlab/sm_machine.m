% Synchronous machine model with a GradNet current map with spatial harmonics,
% ported from motulator (MATLAB port of ../c/sm_machine.c, without the PM-flux
% linkage, which the model takes from motulator). The functions are copied into
% the MATLAB function of the machine (mfun_gradnet_machine.m) by
% motulator_export.simulink.
%
% Python counterparts:
%   motulator/drive/utils/_parameters.py  SpatialSaturatedSynchronousMachinePars
%   motulator/drive/model/_machines.py    SynchronousMachine

function [i_s_dq, tau_M] = machine_magnetic_map(par, psi_s_dq, theta_m)
%MACHINE_MAGNETIC_MAP Stator current (A) in rotor coordinates and the
%   electromagnetic torque (Nm), as functions of the stator flux linkage (Vs) in
%   rotor coordinates and the electrical rotor angle (rad).
[i_s_dq, tau_m] = gradnet_current_map_harmonics(par.current_map, par.k, psi_s_dq, ...
    theta_m);
tau_M = par.n_p*tau_m;
end

function d_psi_s_dq = machine_flux_rhs(par, u_s_ab, psi_s_dq, theta_M, w_M)
%MACHINE_FLUX_RHS State derivative of the stator flux linkage (Vs) in rotor
%   coordinates, given the stator voltage (V) in stator coordinates and the
%   mechanical rotor angle and speed.
theta_m = par.n_p*theta_M;
i_s_dq = machine_magnetic_map(par, psi_s_dq, theta_m);
u_s_dq = u_s_ab*exp(-1i*theta_m);
d_psi_s_dq = u_s_dq - par.R_s*i_s_dq - 1i*par.n_p*w_M*psi_s_dq;
end
