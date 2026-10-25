function add_induction_machine(blk, pos)
%ADD_INDUCTION_MACHINE Block of InductionMachine with constant parameters.
%   The inverse-Gamma model in stator coordinates, with the stator and rotor flux
%   linkages as the states, is equivalent to the Gamma model of InductionMachine
%   in motulator. The mask has the machine model par (inverse-Gamma parameters).
import motulator.internal.*
script = {
    'function [d_psi, i_s_abc, tau_M] = fcn(u_s_ab, psi, w_M, par)'
    '% Induction machine model in stator coordinates (inverse-Gamma model)'
    'n_p = par(1); R_s = par(2); R_R = par(3); L_sgm = par(4); L_M = par(5);'
    'psi_s = psi(1:2);'
    'psi_R = psi(3:4);'
    '% Stator and rotor currents from the flux linkages'
    'i_s = (psi_s - psi_R)/L_sgm;'
    'i_R = psi_R/L_M - i_s;'
    '% Voltage equations: d_psi_R = -R_R*i_R + 1j*w_m*psi_R'
    'w_m = n_p*w_M;'
    'd_psi = [u_s_ab - R_s*i_s; -R_R*i_R + w_m*[-psi_R(2); psi_R(1)]];'
    '% Phase currents (no zero sequence) and electromagnetic torque'
    'i_s_abc = [i_s(1); -0.5*i_s(1) + sqrt(3)/2*i_s(2)'
    '    -0.5*i_s(1) - sqrt(3)/2*i_s(2)];'
    'tau_M = 1.5*n_p*(psi_s(1)*i_s(2) - psi_s(2)*i_s(1));'
    };
add_machine(blk, pos, script, 'psi', 'zeros(4, 1)', {'w_M'}, ...
    '[par.n_p par.R_s par.R_R par.L_sgm par.L_M]');
add_mask(blk, 'InductionMachine (motulator)', ...
    ['Induction machine with constant parameters (InductionMachine), modeled with ' ...
    'the inverse-Gamma model in stator coordinates with the stator and rotor flux ' ...
    'linkages as the states, which is equivalent to the Gamma model of motulator. ' ...
    'The inputs are the stator voltage u_s_ab (V) in stationary coordinates and ' ...
    'the rotor speed w_M (rad/s), and the outputs the phase currents i_s_abc (A) ' ...
    'and the torque tau_M (Nm). The machine model par is a struct with the fields ' ...
    'n_p, R_s, R_R, L_sgm, and L_M of InductionMachineInvGammaPars.'], ...
    {'par', 'par: Machine model (InductionMachineInvGammaPars) as a struct', ...
    'machine.par'}, '', icon(blk, 'InductionMachine'));
end
