function add_lcl_filter(blk, pos)
%ADD_LCL_FILTER Block of LCLFilter without resistances.
%   The converter current, the capacitor voltage, and the grid current in
%   stationary coordinates are the states. Without the grid impedance, the PCC
%   voltage (after the grid-side inductor) is the grid voltage, measured as the
%   line-to-line voltages u_ab and u_bc. The mask has the arguments of LCLFilter.
import motulator.internal.*
script = {
    'function [d_x, i_c_abc, i_g_abc, u_g_line] = fcn(u_c_ab, x, e_g_ab, par)'
    '% LCL filter in stationary coordinates (LCLFilter)'
    'L_fc = par(1); L_fg = par(2); C_f = par(3);'
    'i_c = x(1:2); u_f = x(3:4); i_g = x(5:6);'
    'd_x = [(u_c_ab - u_f)/L_fc; (i_c - i_g)/C_f; (u_f - e_g_ab)/L_fg];'
    '% Phase currents (no zero sequence) and line-to-line PCC voltages'
    'T = [1 0; -0.5 sqrt(3)/2; -0.5 -sqrt(3)/2];'
    'i_c_abc = T*i_c;'
    'i_g_abc = T*i_g;'
    'u_g_line = [1.5*e_g_ab(1) - sqrt(3)/2*e_g_ab(2); sqrt(3)*e_g_ab(2)];'
    };
add_ac_filter(blk, pos, script, ...
    '[0; 0; real(u_f0_ab); imag(u_f0_ab); 0; 0]', '[L_fc L_fg C_f]', ...
    {'i_c_abc', 'i_g_abc', 'u_g_line'});
add_mask(blk, 'LCLFilter (motulator)', ...
    ['LCL filter without resistances and the grid impedance (LCLFilter), with ' ...
    'the converter current, the capacitor voltage (starting from u_f0_ab), and ' ...
    'the grid current in stationary coordinates as the states. The inputs are ' ...
    'the converter voltage u_c_ab (V) and the grid voltage e_g_ab (V), and the ' ...
    'outputs the converter and grid currents i_c_abc and i_g_abc (A) and the ' ...
    'line-to-line PCC voltages u_g_line (V), which equal those of the grid.'], {
    'L_fc', 'L_fc: Converter-side inductance (H)', 'ac_filter.L_fc'
    'L_fg', 'L_fg: Grid-side inductance (H)', 'ac_filter.L_fg'
    'C_f', 'C_f: Filter capacitance (F)', 'ac_filter.C_f'
    'u_f0_ab', 'u_f0_ab: Initial capacitor voltage (V), a complex space vector', ...
    'ac_filter.u_f0_ab'
    }, '', icon(blk, 'LCLFilter'));
end
