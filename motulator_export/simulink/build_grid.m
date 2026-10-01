function build_grid(s)
%BUILD_GRID Build a Simulink model of a grid converter system.
%   BUILD_GRID(S) compiles the S-function of the control system, builds the model
%   S.name, and saves it in the folder S.folder. The struct S is written by
%   motulator_export.simulink.grid, with the fields
%
%     name      Model name
%     folder    Folder of the model, the S-function, and its compiled version
%     init      MATLAB code of the model workspace (converter, ac_filter, ac_source)
%     ac_filter 'lcl' (LCL filter) or 'l' (L filter and grid inductance)
%     control   Control system, see blocks.add_control_system
%     refs      References {name, 'step' or 'constant', steps or value}, the first
%               inputs of the control system (see blocks.add_step)
%     scope     Scope signals {name, indices in [mdl; ctrl]}
%     T_s       Sampling period (s)
%     t_stop    Stop time (s)
%
%   The system model is built from basic Simulink blocks: the carrier comparison,
%   the ideal converter with a stiff DC bus, the AC filter, and the grid voltage.
%   The root-level output ports 'mdl' and 'ctrl' give the converter currents
%   [i_c_a i_c_b i_c_c] (and the grid currents [i_g_a i_g_b i_g_c] with the LCL
%   filter) and the monitored signals of the control system.
%
%   The blocks are aligned with the ports they connect to, so that the lines are
%   straight, and the feedback lines are drawn through given corners.

blocks.compile(s.control.sfunction, s.folder);
slx = blocks.new_model(s);
sys = s.name;
cs = s.control.name;
lcl = strcmp(s.ac_filter, 'lcl');

% Main path in a row: the control system, the computational delay of one sampling
% period, the PWM, the converter, and the AC filter, with the grid below the line
% from the converter to the filter
n_in = numel(s.control.inputs);
blocks.add_control_system([sys '/' cs], s.control, [160 100 300 100 + 50*n_in]);
add([sys '/Delay'], 'built-in/UnitDelay', [340 0 375 30], ...
    'SampleTime', blocks.num(s.T_s));
blocks.add_pwm([sys '/PWM'], s.T_s, [410 0 490 60]);
blocks.add_converter([sys '/Converter'], [530 0 610 60]);
if lcl
    add_lcl_filter([sys '/AC filter'], [720 0 820 120]);
else
    add_l_filter([sys '/AC filter'], [720 0 820 80]);
end
add_grid([sys '/Grid'], [630 0 680 30]);
y = blocks.port_y(sys, cs, 'Outport', 1);
for blk = {'Delay', 'PWM', 'Converter', 'AC filter'}
    blocks.align(sys, blk{1}, 'Inport', 1, y);
end
blocks.align(sys, 'Grid', 'Outport', 1, blocks.port_y(sys, 'AC filter', 'Inport', 2));

% References and the DC-bus voltage
for k = 1:size(s.refs, 1)
    name = s.refs{k, 1};
    if strcmp(s.refs{k, 2}, 'step')
        blocks.add_step([sys '/' name], s.refs{k, 3}, [90 0 120 30]);
    else
        add([sys '/' name], 'built-in/Constant', [80 0 130 20], ...
            'Value', blocks.num(s.refs{k, 3}));
    end
    blocks.align(sys, name, 'Outport', 1, blocks.port_y(sys, cs, 'Inport', k));
    blocks.connect(sys, [name '/1'], sprintf('%s/%d', cs, k));
end
add([sys '/u_dc'], 'built-in/Constant', [80 0 130 20], 'Value', 'converter.u_dc');
blocks.align(sys, 'u_dc', 'Outport', 1, blocks.port_y(sys, cs, 'Inport', n_in));
blocks.connect(sys, 'u_dc/1', sprintf('%s/%d', cs, n_in));

% Converter (and grid) currents for the output port 'mdl'
n_mdl = 1 + lcl;
add([sys '/Mux mdl'], 'built-in/Mux', [950 0 955 40*n_mdl], ...
    'Inputs', num2str(n_mdl));
blocks.align(sys, 'Mux mdl', 'Inport', 1, ...
    blocks.port_y(sys, 'AC filter', 'Outport', 1));

% Signal flow. The measured converter currents and PCC voltages are fed back above
% the row.
n_refs = size(s.refs, 1);
i_c = sprintf('%s/%d', cs, n_refs + 1);
blocks.connect(sys, [cs '/1'], 'Delay/1');
blocks.connect(sys, 'Delay/1', 'PWM/1');
blocks.connect(sys, 'PWM/1', 'Converter/1');
blocks.connect(sys, 'Converter/1', 'AC filter/1');
blocks.connect(sys, 'Grid/1', 'AC filter/2');
blocks.connect(sys, 'AC filter/1', 'Mux mdl/1');
blocks.route(sys, 'AC filter/1', i_c, 'x', 840, 'y', 60, 'x', 60);
if lcl
    blocks.connect(sys, 'AC filter/2', 'Mux mdl/2');
    blocks.route(sys, 'AC filter/3', sprintf('%s/%d', cs, n_refs + 2), ...
        'x', 870, 'y', 40, 'x', 40);
end

% Output ports and the scope, with the monitored signals in a lane below the blocks
pos = get_param([sys '/' cs], 'Position');
y_ctrl = pos(4) + 60;
blocks.add_outputs(sys, cs, s.scope, 1010, y_ctrl);

save_system(sys, slx);
fprintf('Saved %s\n', slx);
end

% -------------------------------------------------------------------------------
function add_lcl_filter(blk, pos)
% LCL filter without resistances (LCLFilter), with the converter current, the
% capacitor voltage, and the grid current in stationary coordinates as the states.
% Without the grid impedance, the PCC voltage (after the grid-side inductor) is the
% grid voltage, measured as the line-to-line voltages u_ab and u_bc.
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
add_ac_filter(blk, pos, script, '[0; 0; ac_filter.u_f0_ab(:); 0; 0]', ...
    '[ac_filter.L_fc ac_filter.L_fg ac_filter.C_f]', ...
    {'i_c_abc', 'i_g_abc', 'u_g_line'});
end

function add_l_filter(blk, pos)
% L filter and the grid inductance (LFilter), with the converter current in
% stationary coordinates as the state
script = {
    'function [d_i_c, i_c_abc] = fcn(u_c_ab, i_c, e_g_ab, par)'
    '% L filter and grid impedance in stationary coordinates (LFilter)'
    'L_t = par(1) + par(3); R_t = par(2);'
    'd_i_c = (u_c_ab - e_g_ab - R_t*i_c)/L_t;'
    '% Phase currents (no zero sequence)'
    'i_c_abc = [1 0; -0.5 sqrt(3)/2; -0.5 -sqrt(3)/2]*i_c;'
    };
add_ac_filter(blk, pos, script, '[0; 0]', ...
    '[ac_filter.L_f ac_filter.R_f ac_filter.L_g]', {'i_c_abc'});
end

function add_ac_filter(blk, pos, script, ic, par, outputs)
% AC filter subsystem: a MATLAB Function block computing the derivatives of the
% states (the state of an integrator) and the outputs from the converter voltage,
% the states, the grid voltage, and the parameters
add(blk, 'built-in/Subsystem', pos);
fcn = [blk '/Filter model'];
height = 44*max(4, numel(outputs) + 1);
add(fcn, 'simulink/User-Defined Functions/MATLAB Function', [230 40 350 40 + height]);
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', fcn);
chart.Script = strjoin(script', newline);
% The state is integrated to the left of the function, with the feedback line above
add([blk '/u_c_ab'], 'built-in/Inport', [140 0 170 14]);
add([blk '/x'], 'built-in/Integrator', [140 0 170 30], 'InitialCondition', ic);
add([blk '/e_g_ab'], 'built-in/Inport', [40 0 70 14]);
add([blk '/par'], 'built-in/Constant', [40 0 180 20], 'Value', par);
fcn_inputs = {'u_c_ab', 'x', 'e_g_ab', 'par'};
for k = 1:numel(fcn_inputs)
    y = blocks.port_y(blk, 'Filter model', 'Inport', k);
    blocks.align(blk, fcn_inputs{k}, 'Outport', 1, y);
    blocks.connect(blk, [fcn_inputs{k} '/1'], sprintf('Filter model/%d', k));
end
blocks.route(blk, 'Filter model/1', 'x/1', 'x', 370, 'y', 15, 'x', 110);
% The vectors as 1-D vectors for the S-function input
for k = 1:numel(outputs)
    vec = ['1-D ' outputs{k}];
    add([blk '/' vec], 'built-in/Reshape', [400 0 430 30], ...
        'OutputDimensionality', '1-D array');
    add([blk '/' outputs{k}], 'built-in/Outport', [480 0 510 14]);
    y = blocks.port_y(blk, 'Filter model', 'Outport', k + 1);
    blocks.align(blk, vec, 'Inport', 1, y);
    blocks.align(blk, outputs{k}, 'Inport', 1, y);
    blocks.connect(blk, sprintf('Filter model/%d', k + 1), [vec '/1']);
    blocks.connect(blk, [vec '/1'], [outputs{k} '/1']);
end
end

function add_grid(blk, pos)
% Grid voltage e_g_ab = e_g*exp(1j*w_g*t) of ThreePhaseSource, with constant
% magnitude and frequency
add(blk, 'built-in/Subsystem', pos);
add([blk '/e_g_alpha'], 'built-in/Sin', [40 30 70 60], 'SineType', 'Time based', ...
    'Amplitude', 'ac_source.e_g', 'Frequency', 'ac_source.w_g', 'Phase', 'pi/2', ...
    'SampleTime', '0');
add([blk '/e_g_beta'], 'built-in/Sin', [40 90 70 120], 'SineType', 'Time based', ...
    'Amplitude', 'ac_source.e_g', 'Frequency', 'ac_source.w_g', 'Phase', '0', ...
    'SampleTime', '0');
add([blk '/Mux'], 'built-in/Mux', [130 0 135 80], 'Inputs', '2');
blocks.align(blk, 'Mux', 'Inport', 1, blocks.port_y(blk, 'e_g_alpha', 'Outport', 1));
add([blk '/e_g_ab'], 'built-in/Outport', [190 0 220 14]);
blocks.align(blk, 'e_g_ab', 'Inport', 1, blocks.port_y(blk, 'Mux', 'Outport', 1));
blocks.connect(blk, 'e_g_alpha/1', 'Mux/1');
blocks.route(blk, 'e_g_beta/1', 'Mux/2', 'x', 100);
blocks.connect(blk, 'Mux/1', 'e_g_ab/1');
end

function add(blk, type, pos, varargin)
blocks.add(blk, type, pos, varargin{:});
end
