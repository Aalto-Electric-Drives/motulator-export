function scope = build_grid(sys, mdl, c, lib, info)
%BUILD_GRID Blocks and lines of the model of a grid converter system.
%   BUILD_GRID(SYS, MDL, C, LIB, INFO) adds to the model SYS the blocks of the
%   library LIB: the control system C (see control_system), whose block INFO is
%   described by the data of the library (see block_info), the computational delay
%   of one sampling period, the carrier comparison, the converter, the AC filter,
%   and the grid of the grid converter system MDL, with the references. The masks
%   of the blocks refer to the parameters of the model. The root-level output
%   ports 'mdl' and 'ctrl' give the converter currents [i_c_a i_c_b i_c_c] (and the
%   grid currents [i_g_a i_g_b i_g_c] with the LCL filter) and the monitored
%   signals of the control system, which are also shown in a scope, whose groups
%   SCOPE {name, indices in [mdl; ctrl]} are returned.
%
%   The blocks are aligned with the ports they connect to, so that the lines are
%   straight, and the feedback lines are drawn through given corners.
import motulator.internal.*
cs = c.name;
in = @(name) find(strcmp(info.inputs, name));
lcl = isa(mdl.ac_filter, 'motulator.grid.model.LCLFilter');

% Main path in a row: the control system, the computational delay of one sampling
% period, the PWM, the converter, and the AC filter, with the grid below the line
% from the converter to the filter
n_in = numel(info.inputs);
add_block(control_block(c, lib), [sys '/' cs], ...
    'Position', [160 100 300 100 + 50*n_in]);
add([sys '/Delay'], 'built-in/UnitDelay', [340 0 375 30], 'SampleTime', c.T_s_expr);
add_block([lib '/CarrierComparison'], [sys '/PWM'], 'Position', [410 0 490 60]);
set_param([sys '/PWM'], 'T_s', c.T_s_expr);
add_block([lib '/VoltageSourceConverter'], [sys '/Converter'], ...
    'Position', [530 0 610 60]);
if lcl
    add_block([lib '/LCLFilter'], [sys '/AC filter'], 'Position', [720 0 820 120]);
else
    add_block([lib '/LFilter'], [sys '/AC filter'], 'Position', [720 0 820 80]);
end
add_block([lib '/ThreePhaseSource'], [sys '/Grid'], 'Position', [630 0 680 30]);
y = port_y(sys, cs, 'Outport', 1);
for blk = {'Delay', 'PWM', 'Converter', 'AC filter'}
    align(sys, blk{1}, 'Inport', 1, y);
end
align(sys, 'Grid', 'Outport', 1, port_y(sys, 'AC filter', 'Inport', 2));

% The gate enable (the converter does not block its gates), the references sampled
% at the sampling instants as in motulator, and the measured DC-bus voltage
sources = [{'enable', 1}; c.refs];
for k = 1:size(sources, 1)
    [name, src] = sources{k, :};
    add_source([sys '/' name], src, [90 0 120 30], c.T_s);
    align(sys, name, 'Outport', 1, port_y(sys, cs, 'Inport', in(name)));
    connect(sys, [name '/1'], sprintf('%s/%d', cs, in(name)));
end
add([sys '/u_dc'], 'built-in/Constant', [80 0 130 20], 'Value', 'converter.u_dc');
align(sys, 'u_dc', 'Outport', 1, port_y(sys, cs, 'Inport', in('u_dc')));
connect(sys, 'u_dc/1', sprintf('%s/%d', cs, in('u_dc')));

% Converter (and grid) currents for the output port 'mdl'
n_mdl = 1 + lcl;
add([sys '/Mux mdl'], 'built-in/Mux', [950 0 955 40*n_mdl], 'Inputs', num2str(n_mdl));
align(sys, 'Mux mdl', 'Inport', 1, port_y(sys, 'AC filter', 'Outport', 1));

% Signal flow. The measured converter currents and PCC voltages are fed back above
% the row.
connect(sys, [cs '/1'], 'Delay/1');
connect(sys, 'Delay/1', 'PWM/1');
connect(sys, 'PWM/1', 'Converter/1');
connect(sys, 'Converter/1', 'AC filter/1');
connect(sys, 'Grid/1', 'AC filter/2');
connect(sys, 'AC filter/1', 'Mux mdl/1');
route(sys, 'AC filter/1', sprintf('%s/%d', cs, in('i_c_abc')), 'x', 840, 'y', 60, ...
    'x', 60);
u_g = sprintf('%s/%d', cs, in('u_g_line'));
if lcl
    connect(sys, 'AC filter/2', 'Mux mdl/2');
    route(sys, 'AC filter/3', u_g, 'x', 870, 'y', 40, 'x', 40);
else
    route(sys, 'AC filter/2', u_g, 'x', 870, 'y', 40, 'x', 40);
end

% Output ports and the scope, with the monitored signals in a lane below the blocks:
% the powers, the converter currents, and the other monitored signals
groups = info.outputs;
n_mdl = 3*n_mdl;
signals = @(names) n_mdl + cellfun(@(n) find(strcmp(info.signals, n)), names);
scope = [{'Power', signals(groups{1, 2}); 'Converter current', [1 2 3]}; ...
    cell(size(groups, 1) - 1, 2)];
for k = 2:size(groups, 1)
    scope(k + 1, :) = {strtok(groups{k, 1}, ' '), signals(groups{k, 2})};
end
pos = get_param([sys '/' cs], 'Position');
add_outputs(sys, cs, scope, 1010, pos(4) + 60);
end
