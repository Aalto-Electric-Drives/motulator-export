% Synchronous machine model with a GradNet current map with spatial harmonics,
% ported from motulator (MATLAB port of ../c/sm_machine.c). The functions are copied
% into the MATLAB function of the machine (mfun_gradnet_machine.m) by
% motulator_export.simulink.
%
% Python counterparts:
%   motulator/drive/utils/_parameters.py  SpatialSaturatedSynchronousMachinePars
%   motulator/drive/model/_machines.py    SynchronousMachine

function [i_d, i_s] = pm_flux_i_d(psi_d, par)
%PM_FLUX_I_D d-axis current at the d-axis flux linkage psi_d and zero angle.
[i_s, ~] = gradnet_current_map_harmonics(par.current_map, par.k, complex(psi_d), 0);
i_d = real(i_s);
end

function psi_f = pm_flux(par)
%PM_FLUX PM-flux linkage from i_d(psi_f) = 0 (_solve_pm_flux).
%   The root is bracketed by doubling the upper bound and then found using Brent's
%   method. Zero if there are no permanent magnets, NaN if the root cannot be
%   bracketed.
i_lo = pm_flux_i_d(0, par);
if i_lo >= 0
    psi_f = 0; % No permanent magnets
    return
end
psi_lo = 0;
psi_hi = 1e-3;
i_hi = pm_flux_i_d(psi_hi, par);
while i_hi < 0 && psi_hi < 1e3
    psi_lo = psi_hi;
    psi_hi = 2*psi_hi;
    i_hi = pm_flux_i_d(psi_hi, par);
end
if isfinite(i_lo) && i_hi >= 0
    psi_f = brentq(@pm_flux_i_d, psi_lo, psi_hi, par);
else
    psi_f = NaN;
end
end

function par = spatial_saturated_synchronous_machine_pars(n_p, R_s, current_map, k)
%SPATIAL_SATURATED_SYNCHRONOUS_MACHINE_PARS Machine model with the current map of
%   the spatial harmonic order k (SpatialSaturatedSynchronousMachinePars), with the
%   PM-flux linkage computed from the current map.
par = struct('n_p', n_p, 'R_s', R_s, 'current_map', current_map, 'k', k, ...
    'psi_f', 0);
par.psi_f = pm_flux(par);
if par.psi_f < 1e-3
    par.psi_f = 0; % No permanent magnets
end
end

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
