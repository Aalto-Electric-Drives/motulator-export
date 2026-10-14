% Optimal control loci of synchronous machines, ported from motulator (MATLAB port of
% ../c/sm_control_loci.c). The functions are embedded in the MATLAB Function blocks
% by motulator_export.simulink.
%
% Python counterpart:
%   motulator/drive/utils/_sm_control_loci.py  ControlLoci
%
% Both SynchronousMachinePars and SaturatedSynchronousMachinePars (with a flux map)
% are supported, see sm_parameters.m. The loci are computed at 16 points and stored
% as lookup tables, as in ReferenceGenerator of motulator. The tables are computed
% by the function sm_reference_luts.m next to the Simulink model (written by
% motulator_export.simulink), which the mask initialization of the block
% ReferenceGenerator calls, so that the block does not compute them at its start.

function [y, i_s] = mtpa_cond(gamma, d)
%MTPA_COND Condition of the MTPA current angle gamma, d.magnitude being the current
%   magnitude.
i_s = d.magnitude*exp(1i*gamma);
psi_s = psi_s_dq(d.par, i_s);
L = incr_ind_mat(d.par, i_s);
psi_a = aux_flux(L, i_s, psi_s);
y = real(psi_a*conj(i_s));
end

function [y, i_s] = mtpv_flux_cond(delta, d)
%MTPV_FLUX_COND Condition of the MTPV flux angle delta, d.magnitude being the flux
%   magnitude. The current is returned as the side effect of motulator.
psi_s = d.magnitude*exp(1i*delta);
i_s = iterate_i_s_dq(d.par, psi_s);
L = incr_ind_mat(d.par, i_s);
i_a = aux_current(L, psi_s, i_s);
y = real(i_a*conj(psi_s));
end

function [y, i_s] = mtpv_current_cond(gamma, d)
%MTPV_CURRENT_COND Condition of the MTPV current angle gamma, d.magnitude being the
%   current magnitude.
i_s = d.magnitude*exp(1i*gamma);
psi_s = psi_s_dq(d.par, i_s);
L = incr_ind_mat(d.par, i_s);
i_a = aux_current(L, psi_s, i_s);
y = real(i_a*conj(psi_s));
end

function r = angle_range(par)
%ANGLE_RANGE Range of the current or flux angle of the loci.
if par.psi_f == 0
    r = [0, 0.5*pi];
else
    r = [0.5*pi, pi];
end
end

function gamma = compute_mtpa_current_angle(par, i_s_abs)
%COMPUTE_MTPA_CURRENT_ANGLE MTPA current angle at the current magnitude.
d = struct('par', par, 'magnitude', i_s_abs);
r = angle_range(par);
if mtpa_cond(r(1), d)*mtpa_cond(r(2), d) > 0
    gamma = 0; % No root in the range
    return
end
gamma = brentq(@mtpa_cond, r(1), r(2), d);
end

function [delta, i_s] = compute_mtpv_flux_angle(par, psi_s_abs)
%COMPUTE_MTPV_FLUX_ANGLE MTPV flux angle and the current at the flux magnitude.
d = struct('par', par, 'magnitude', psi_s_abs);
r = angle_range(par);
if mtpv_flux_cond(r(1), d)*mtpv_flux_cond(r(2), d) > 0
    i_s = complex(NaN);
    delta = NaN; % No root in the range
    return
end
% The current at the latest evaluation, as in motulator
[delta, i_s] = brentq(@mtpv_flux_cond, r(1), r(2), d);
end

function i_s = compute_mtpv_current(par, i_s_abs)
%COMPUTE_MTPV_CURRENT MTPV current at the current magnitude, NaN if no MTPV.
d = struct('par', par, 'magnitude', i_s_abs);
r = angle_range(par);
if mtpv_current_cond(r(1), d)*mtpv_current_cond(r(2), d) >= 0
    i_s = complex(NaN); % No MTPV for this current
    return
end
gamma = brentq(@mtpv_current_cond, r(1), r(2), d);
i_s = i_s_abs*exp(1i*gamma);
end

function tau = locus_torque(par, i_s, psi_s)
%LOCUS_TORQUE Electromagnetic torque.
tau = 1.5*par.n_p*imag(i_s*conj(psi_s));
end

function luts = compute_reference_luts(par, i_s_max)
%COMPUTE_REFERENCE_LUTS Lookup tables of the ReferenceGenerator as the columns of a
%   16-by-6 matrix (the sample points and the values of each table): the MTPA flux
%   magnitude vs. the torque, the current-limit torque vs. the flux magnitude, and
%   the MTPV torque vs. the flux magnitude.
num = 16;
luts = zeros(num, 6);
% MTPA locus
i_s_abs = linspace(0, i_s_max, num);
i_s = complex(0);
psi_s = complex(0);
for k = 1:num
    gamma = compute_mtpa_current_angle(par, i_s_abs(k));
    i_s = i_s_abs(k)*exp(1i*gamma);
    psi_s = psi_s_dq(par, i_s);
    luts(k, 1:2) = [locus_torque(par, i_s, psi_s), abs(psi_s)];
end
i_s_mtpa_max = i_s;
% MTPV limit
psi_s_abs = linspace(0, abs(psi_s), num);
for k = 1:num
    [delta, i_s] = compute_mtpv_flux_angle(par, psi_s_abs(k));
    psi_s = psi_s_abs(k)*exp(1i*delta);
    luts(k, 5:6) = [abs(psi_s), locus_torque(par, i_s, psi_s)];
end
% Current limit, from the MTPV current angle (or, if no MTPV, from pi for PM
% machines and pi/2 for machines without PMs) to the MTPA current angle
gamma1 = angle(compute_mtpv_current(par, i_s_max));
gamma2 = angle(i_s_mtpa_max);
if isnan(gamma1)
    if par.psi_f == 0
        gamma1 = 0.5*pi;
    else
        gamma1 = pi;
    end
end
gamma = linspace(gamma1, gamma2, num);
for k = 1:num
    i_s = i_s_max*exp(1i*gamma(k));
    psi_s = psi_s_dq(par, i_s);
    luts(k, 3:4) = [abs(psi_s), locus_torque(par, i_s, psi_s)];
end
end
