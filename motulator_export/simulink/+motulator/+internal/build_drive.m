function scope = build_drive(sys, mdl, c, lib, info)
%BUILD_DRIVE Blocks and lines of the model of a machine drive.
%   BUILD_DRIVE(SYS, MDL, C, LIB, INFO) adds to the model SYS the blocks of the
%   library LIB: the control system C (see control_system), whose block INFO is
%   described by the data of the library (see block_info), the computational delay
%   of one sampling period, the carrier comparison, the converter, the machine,
%   and the mechanics of the drive MDL, with the references and the load torque
%   (or the external rotor speed of ExternalRotorSpeed).
%   The masks of the blocks refer to the parameters of the model. The root-level
%   output ports 'mdl' and 'ctrl' give the machine signals [i_a i_b i_c w_M
%   theta_M tau_M] and the monitored signals of the control system, which are also
%   shown in a scope, whose groups SCOPE {name, indices in [mdl; ctrl]} are
%   returned.
%
%   The blocks are aligned with the ports they connect to, so that the lines are
%   straight, and the feedback lines are drawn through given corners.
import motulator.internal.*
cs = c.name;
in = @(name) find(strcmp(info.inputs, name));

% Main path in a row: the control system, the computational delay of one sampling
% period, the PWM, the converter, the machine, and the mechanics
add_block(control_block(c, lib), [sys '/' cs], 'Position', [160 100 300 300]);
add([sys '/Delay'], 'built-in/UnitDelay', [340 0 375 30], 'SampleTime', c.T_s_expr);
add_block([lib '/CarrierComparison'], [sys '/PWM'], 'Position', [410 0 490 60]);
set_param([sys '/PWM'], 'T_s', c.T_s_expr);
add_block([lib '/VoltageSourceConverter'], [sys '/Converter'], ...
    'Position', [530 0 610 60]);
if isa(mdl.machine, 'motulator.drive.model.InductionMachine')
    pos = [680 0 780 80];
else
    pos = [680 0 780 120];
end
add_block([lib '/' mdl.machine.block()], [sys '/Machine'], 'Position', pos);
external = isa(mdl.mechanics, 'motulator.drive.model.ExternalRotorSpeed');
if external
    mechanics = 'ExternalRotorSpeed';
else
    mechanics = 'MechanicalSystem';
end
add_block([lib '/' mechanics], [sys '/Mechanics'], 'Position', [890 0 990 80]);
y = port_y(sys, cs, 'Outport', 1);
for blk = {'Delay', 'PWM', 'Converter', 'Machine'}
    align(sys, blk{1}, 'Inport', 1, y);
end
align(sys, 'Mechanics', 'Inport', 1, port_y(sys, 'Machine', 'Outport', 2));

% Sources: the gate enable (the converter does not block its gates), the speed or
% torque reference sampled at the sampling instants as in motulator, the measured
% DC-bus voltage, and the load torque (or the external rotor speed)
sources = [{'enable', 1}; c.refs];
for k = 1:size(sources, 1)
    [name, src] = sources{k, :};
    add_source([sys '/' name], src, [90 0 120 30], c.T_s);
    align(sys, name, 'Outport', 1, port_y(sys, cs, 'Inport', in(name)));
    connect(sys, [name '/1'], sprintf('%s/%d', cs, in(name)));
end
add([sys '/u_dc'], 'built-in/Constant', [80 0 130 20], 'Value', 'converter.u_dc');
align(sys, 'u_dc', 'Outport', 1, port_y(sys, cs, 'Inport', in('u_dc')));
if external
    add_source([sys '/w_M'], mdl.mechanics.w_M, [835 0 865 30]);
    align(sys, 'w_M', 'Outport', 1, port_y(sys, 'Mechanics', 'Inport', 1));
else
    add_source([sys '/tau_L'], mdl.mechanics.tau_L, [835 0 865 30]);
    align(sys, 'tau_L', 'Outport', 1, port_y(sys, 'Mechanics', 'Inport', 2));
end

% Machine signals [i_a i_b i_c w_M theta_M tau_M] for the output port 'mdl'
add([sys '/Mux mdl'], 'built-in/Mux', [1060 0 1065 160], 'Inputs', '4');
align(sys, 'Mux mdl', 'Inport', 2, port_y(sys, 'Mechanics', 'Outport', 1));

% Signal flow. The current is fed back above the row, and the speed and the angle
% below it, in lanes below the blocks.
pos = get_param([sys '/Mux mdl'], 'Position');
y_top = 60;
n_in = numel(info.inputs);
y_w_M = max(pos(4), port_y(sys, cs, 'Inport', n_in)) + 40;
y_theta_M = y_w_M + 30;
y_ctrl = y_theta_M + 30;
connect(sys, 'u_dc/1', sprintf('%s/%d', cs, in('u_dc')));
connect(sys, [cs '/1'], 'Delay/1');
connect(sys, 'Delay/1', 'PWM/1');
connect(sys, 'PWM/1', 'Converter/1');
connect(sys, 'Converter/1', 'Machine/1');
if external
    connect(sys, 'w_M/1', 'Mechanics/1');
else
    connect(sys, 'Machine/2', 'Mechanics/1');
    connect(sys, 'tau_L/1', 'Mechanics/2');
end
connect(sys, 'Mechanics/1', 'Mux mdl/2');
connect(sys, 'Mechanics/2', 'Mux mdl/3');
route(sys, 'Machine/1', 'Mux mdl/1', 'x', 1045);
route(sys, 'Machine/2', 'Mux mdl/4', 'x', 815);
route(sys, 'Machine/1', sprintf('%s/%d', cs, in('i_s_abc')), 'x', 800, 'y', y_top, ...
    'x', 60);
route(sys, 'Mechanics/1', 'Machine/2', 'x', 1010, 'y', y_w_M, 'x', 660);
if isa(mdl.machine, 'motulator.drive.model.SynchronousMachine')
    % The rotor angle to the machine, and to the control system (not in V/Hz
    % control)
    route(sys, 'Mechanics/2', 'Machine/3', 'x', 1025, 'y', y_theta_M, 'x', 640);
    if ~isempty(in('theta_M'))
        route(sys, 'Mechanics/2', sprintf('%s/%d', cs, in('theta_M')), ...
            'x', 1025, 'y', y_theta_M, 'x', 140);
    end
elseif ~isempty(in('w_M'))
    % The rotor speed to the control system (not in V/Hz control)
    route(sys, 'Mechanics/1', sprintf('%s/%d', cs, in('w_M')), ...
        'x', 1010, 'y', y_w_M, 'x', 140);
end

% Output ports and the scope: the speed and the torque of the control system and
% of the machine, the phase currents, and the flux linkages of the control system
groups = info.outputs;
n_mdl = 6;
signals = @(names) n_mdl + cellfun(@(n) find(strcmp(info.signals, n)), names);
flux = find(strncmp(info.signals, 'psi', 3)) + n_mdl;
scope = {
    'Speed', [signals(groups{1, 2}), 4]
    'Torque', [signals(groups{2, 2}), 6]
    'Current', [1 2 3]
    'Flux', flux
    };
add_outputs(sys, cs, scope, 1120, y_ctrl);
end
