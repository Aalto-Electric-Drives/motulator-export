function add_synchronous_machine(blk, pos)
%ADD_SYNCHRONOUS_MACHINE Block of SynchronousMachine with SynchronousMachinePars.
%   The stator flux linkage in rotor coordinates is the state, starting from the
%   PM-flux linkage, as in motulator. The mask has the machine model par.
import motulator.internal.*
script = {
    'function [d_psi_s_dq, i_s_abc, tau_M] = fcn(u_s_ab, psi_s_dq, w_M, theta_M, par)'
    '% Synchronous machine model in rotor coordinates (SynchronousMachine)'
    'n_p = par(1); R_s = par(2); L_d = par(3); L_q = par(4); psi_f = par(5);'
    'c = cos(n_p*theta_M);'
    's = sin(n_p*theta_M);'
    '% Stator current from the flux linkage'
    'i_d = (psi_s_dq(1) - psi_f)/L_d;'
    'i_q = psi_s_dq(2)/L_q;'
    '% Voltage equation: d_psi_s_dq = u_s_dq - R_s*i_s_dq - 1j*w_m*psi_s_dq'
    'w_m = n_p*w_M;'
    'u_d = c*u_s_ab(1) + s*u_s_ab(2);'
    'u_q = -s*u_s_ab(1) + c*u_s_ab(2);'
    'd_psi_s_dq = [u_d - R_s*i_d + w_m*psi_s_dq(2); u_q - R_s*i_q - w_m*psi_s_dq(1)];'
    '% Phase currents (no zero sequence) and electromagnetic torque'
    'i_alpha = c*i_d - s*i_q;'
    'i_beta = s*i_d + c*i_q;'
    'i_s_abc = [i_alpha; -0.5*i_alpha + sqrt(3)/2*i_beta'
    '    -0.5*i_alpha - sqrt(3)/2*i_beta];'
    'tau_M = 1.5*n_p*(psi_s_dq(1)*i_q - psi_s_dq(2)*i_d);'
    };
add_machine(blk, pos, script, 'psi_s_dq', '[par.psi_f; 0]', ...
    {'w_M', 'theta_M'}, '[par.n_p par.R_s par.L_d par.L_q par.psi_f]');
add_mask(blk, 'SynchronousMachine (motulator)', ...
    ['Synchronous machine with constant inductances (SynchronousMachine with ' ...
    'SynchronousMachinePars), modeled in rotor coordinates with the stator flux ' ...
    'linkage as the state, starting from the PM-flux linkage, as in motulator. The ' ...
    'inputs are the stator voltage u_s_ab (V) in stationary coordinates and the ' ...
    'rotor speed w_M (rad/s) and angle theta_M (rad), and the outputs the phase ' ...
    'currents i_s_abc (A) and the torque tau_M (Nm). The machine model par is a ' ...
    'struct with the fields n_p, R_s, L_d, L_q, and psi_f.'], ...
    {'par', 'par: Machine model (SynchronousMachinePars) as a struct', ...
    'machine.par'}, '', icon(blk, 'SynchronousMachine'));
end
