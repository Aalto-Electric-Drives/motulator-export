function add_l_filter(blk, pos)
%ADD_L_FILTER Block of LFilter with the grid inductance.
%   The converter current in stationary coordinates is the state. The PCC voltage
%   between the filter and the grid inductance is measured as the line-to-line
%   voltages u_ab and u_bc. The mask has the arguments of LFilter.
import motulator.internal.*
script = {
    'function [d_i_c, i_c_abc, u_g_line] = fcn(u_c_ab, i_c, e_g_ab, par)'
    '% L filter and grid impedance in stationary coordinates (LFilter)'
    'L_t = par(1) + par(3); R_t = par(2);'
    'd_i_c = (u_c_ab - e_g_ab - R_t*i_c)/L_t;'
    '% Phase currents (no zero sequence) and line-to-line PCC voltages'
    'i_c_abc = [1 0; -0.5 sqrt(3)/2; -0.5 -sqrt(3)/2]*i_c;'
    'u_g_ab = (par(3)*(u_c_ab - par(2)*i_c) + par(1)*e_g_ab)/L_t;'
    'u_g_line = [1.5*u_g_ab(1) - sqrt(3)/2*u_g_ab(2); sqrt(3)*u_g_ab(2)];'
    };
add_ac_filter(blk, pos, script, '[0; 0]', '[L_f R_f L_g]', ...
    {'i_c_abc', 'u_g_line'});
add_mask(blk, 'LFilter (motulator)', ...
    ['L filter with the grid inductance, without the grid resistance (LFilter), ' ...
    'with the converter current in stationary coordinates as the state. The ' ...
    'inputs are the converter voltage u_c_ab (V) and the grid voltage e_g_ab (V), ' ...
    'and the outputs the converter current i_c_abc (A) and the line-to-line PCC ' ...
    'voltages u_g_line (V) between the filter and the grid inductance.'], {
    'L_f', 'L_f: Filter inductance (H)', 'ac_filter.L_f'
    'R_f', 'R_f: Filter resistance (Ohm)', 'ac_filter.R_f'
    'L_g', 'L_g: Grid inductance (H)', 'ac_filter.L_g'
    }, '', icon(blk, 'LFilter'));
end
