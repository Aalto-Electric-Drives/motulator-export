% Synchronous machine model parameters, ported from motulator (MATLAB port of
% ../c/sm_parameters.c). The functions are embedded in the MATLAB Function blocks by
% motulator_export.simulink.
%
% Python counterpart:
%   motulator/drive/utils/_parameters.py  SynchronousMachinePars,
%                                         SaturatedSynchronousMachinePars
%
% The same struct represents both classes. If the field saturated is true, the flux
% linkage is given by a GradNet flux map (psi_s_dq_fcn in motulator), and the
% incremental inductances are given by its Jacobian, as in motulator. Otherwise, the
% constant inductances L_d and L_q are used. The struct has the field flux_map in
% both cases, so that the code is the same (without a flux map, its parameters are
% NaN).

function par = synchronous_machine_pars(n_p, R_s, L_d, L_q, psi_f, flux_map)
%SYNCHRONOUS_MACHINE_PARS Constant inductances (SynchronousMachinePars), with an
%   unused flux map.
par = struct('n_p', n_p, 'R_s', R_s, 'L_d', L_d, 'L_q', L_q, 'psi_f', psi_f, ...
    'saturated', false, 'flux_map', flux_map, 'L_d0', 0, 'L_q0', 0);
end

function par = saturated_synchronous_machine_pars(n_p, R_s, flux_map)
%SATURATED_SYNCHRONOUS_MACHINE_PARS GradNet flux map
%   (SaturatedSynchronousMachinePars).
par = struct('n_p', n_p, 'R_s', R_s, 'L_d', 0, 'L_q', 0, 'psi_f', 0, ...
    'saturated', true, 'flux_map', flux_map, 'L_d0', 0, 'L_q0', 0);
% As in SaturatedSynchronousMachinePars.__post_init__
par.psi_f = real(psi_s_dq(par, complex(0)));
L0 = incr_ind_mat(par, complex(0));
par.L_d0 = L0.L_dd;
par.L_q0 = L0.L_qq;
if par.psi_f < 1e-3 % EPS of motulator/drive/utils/_parameters.py
    par.psi_f = 0; % No permanent magnets
end
end

function psi_s = psi_s_dq(par, i_s)
%PSI_S_DQ Stator flux linkage as a function of the stator current.
if par.saturated
    psi_s = gradnet_map(par.flux_map, i_s);
else
    psi_s = par.L_d*real(i_s) + par.psi_f + 1i*par.L_q*imag(i_s);
end
end

function L = incr_ind_mat(par, i_s)
%INCR_IND_MAT Incremental inductance matrix [L_dd, L_dq; L_dq, L_qq] as a struct.
if ~par.saturated
    L = struct('L_dd', par.L_d, 'L_dq', 0, 'L_qq', par.L_q);
    return
end
jac = gradnet_map_jacobian(par.flux_map, i_s);
L = struct('L_dd', jac(1, 1), 'L_dq', jac(1, 2), 'L_qq', jac(2, 2));
end

function i_s = iterate_i_s_dq(par, psi_s)
%ITERATE_I_S_DQ Stator current as a function of the stator flux linkage.
if ~par.saturated
    i_s = (real(psi_s) - par.psi_f)/par.L_d + 1i*imag(psi_s)/par.L_q;
    return
end
% Newton's method with the incremental inductance matrix as the Jacobian (the root
% is the same as with scipy.optimize.root in motulator)
i_s = (real(psi_s) - par.psi_f)/par.L_d0 + 1i*imag(psi_s)/par.L_q0;
for k = 1:50
    err = psi_s_dq(par, i_s) - psi_s;
    if abs(err) < 1e-12*(1 + abs(psi_s))
        break
    end
    L = incr_ind_mat(par, i_s);
    det_L = L.L_dd*L.L_qq - L.L_dq*L.L_dq;
    d_i_d = (L.L_qq*real(err) - L.L_dq*imag(err))/det_L;
    d_i_q = (L.L_dd*imag(err) - L.L_dq*real(err))/det_L;
    i_s = i_s - (d_i_d + 1i*d_i_q);
end
end

function psi_a = aux_flux(L, i_s, psi_s)
%AUX_FLUX Auxiliary flux linkage.
psi_a = psi_s - L.L_qq*real(i_s) - 1i*L.L_dd*imag(i_s) + 1i*L.L_dq*conj(i_s);
end

function i_a = aux_current(L, psi_s, i_s)
%AUX_CURRENT Auxiliary current.
det_L = L.L_dd*L.L_qq - L.L_dq*L.L_dq;
i_a = (L.L_dd*real(psi_s) + 1i*L.L_qq*imag(psi_s) + 1i*L.L_dq*conj(psi_s))/det_L ...
    - i_s;
end
