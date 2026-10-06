function build_drive(s)
%BUILD_DRIVE Build a Simulink model of a machine drive.
%   BUILD_DRIVE(S) compiles the S-function of the control system, builds the model
%   S.name, and saves it in the folder S.folder. The struct S is written by
%   motulator_export.simulink, with the fields
%
%     name      Model name
%     folder    Folder of the model, the S-function, and its compiled version
%     init      MATLAB code of the model workspace (machine, mechanics, converter)
%     machine   'sm' (synchronous machine), 'gn' (synchronous machine with a GradNet
%               current map), or 'im' (induction machine)
%     machine_params  Parameters of the S-function of the machine ('gn' only)
%     control   Control system, see blocks.add_control_system
%     enable    Input 'enable' of the control system: a constant, or steps as
%               w_M_ref
%     w_M_ref   Speed reference steps [time, before, after], see blocks.add_step
%     tau_L     Load torque steps
%     scope     Scope signals {name, indices in [mdl; ctrl]}
%     T_s       Sampling period (s)
%     t_stop    Stop time (s)
%
%   The control system is the S-function in a masked subsystem, whose parameters
%   follow the motulator API. The system model is built from basic Simulink blocks:
%   the carrier comparison, the ideal converter with a stiff DC bus, the machine,
%   and the mechanics. The root-level output ports 'mdl' and 'ctrl' give the
%   machine signals [i_a i_b i_c w_M theta_M tau_M] and the monitored signals of the
%   control system for the comparison with motulator.
%
%   The blocks are aligned with the ports they connect to, so that the lines are
%   straight, and the feedback lines are drawn through given corners.

blocks.compile(s.control.sfunction, s.folder);
slx = blocks.new_model(s);
sys = s.name;
cs = s.control.name;

% Main path in a row: the control system, the computational delay of one sampling
% period, the PWM, the converter, the machine, and the mechanics
blocks.add_control_system([sys '/' cs], s.control, [160 100 300 300]);
add([sys '/Delay'], 'built-in/UnitDelay', [340 0 375 30], ...
    'SampleTime', blocks.num(s.T_s));
blocks.add_pwm([sys '/PWM'], s.T_s, [410 0 490 60]);
blocks.add_converter([sys '/Converter'], [530 0 610 60]);
switch s.machine
    case 'sm'
        add_synchronous_machine([sys '/Machine'], [680 0 780 120]);
    case 'gn'
        add_gradnet_machine([sys '/Machine'], [680 0 780 120], s);
    case 'im'
        add_induction_machine([sys '/Machine'], [680 0 780 80]);
end
blocks.add_mechanics([sys '/Mechanics'], [890 0 990 80]);
y = blocks.port_y(sys, cs, 'Outport', 1);
for blk = {'Delay', 'PWM', 'Converter', 'Machine'}
    blocks.align(sys, blk{1}, 'Inport', 1, y);
end
y = blocks.port_y(sys, 'Machine', 'Outport', 2);
blocks.align(sys, 'Mechanics', 'Inport', 1, y);

% Sources
if isscalar(s.enable)
    add([sys '/enable'], 'built-in/Constant', [80 0 130 20], ...
        'Value', blocks.num(s.enable));
else
    blocks.add_step([sys '/enable'], s.enable, [90 0 120 30]);
end
blocks.align(sys, 'enable', 'Outport', 1, blocks.port_y(sys, cs, 'Inport', 1));
blocks.add_step([sys '/w_M_ref'], s.w_M_ref, [90 0 120 30]);
blocks.align(sys, 'w_M_ref', 'Outport', 1, blocks.port_y(sys, cs, 'Inport', 2));
add([sys '/u_dc'], 'built-in/Constant', [80 0 130 20], 'Value', 'converter.u_dc');
blocks.align(sys, 'u_dc', 'Outport', 1, blocks.port_y(sys, cs, 'Inport', 4));
blocks.add_step([sys '/tau_L'], s.tau_L, [835 0 865 30]);
blocks.align(sys, 'tau_L', 'Outport', 1, blocks.port_y(sys, 'Mechanics', 'Inport', 2));

% Machine signals [i_a i_b i_c w_M theta_M tau_M] for the output port 'mdl'
add([sys '/Mux mdl'], 'built-in/Mux', [1060 0 1065 160], 'Inputs', '4');
y = blocks.port_y(sys, 'Mechanics', 'Outport', 1);
blocks.align(sys, 'Mux mdl', 'Inport', 2, y);

% Signal flow. The current is fed back above the row, and the speed and the angle
% below it, in lanes below the blocks.
pos = get_param([sys '/Mux mdl'], 'Position');
y_top = 60;
y_w_M = max(pos(4), blocks.port_y(sys, cs, 'Inport', 5)) + 40;
y_theta_M = y_w_M + 30;
y_ctrl = y_theta_M + 30;
blocks.connect(sys, 'enable/1', [cs '/1']);
blocks.connect(sys, 'w_M_ref/1', [cs '/2']);
blocks.connect(sys, 'u_dc/1', [cs '/4']);
blocks.connect(sys, [cs '/1'], 'Delay/1');
blocks.connect(sys, 'Delay/1', 'PWM/1');
blocks.connect(sys, 'PWM/1', 'Converter/1');
blocks.connect(sys, 'Converter/1', 'Machine/1');
blocks.connect(sys, 'Machine/2', 'Mechanics/1');
blocks.connect(sys, 'tau_L/1', 'Mechanics/2');
blocks.connect(sys, 'Mechanics/1', 'Mux mdl/2');
blocks.connect(sys, 'Mechanics/2', 'Mux mdl/3');
blocks.route(sys, 'Machine/1', 'Mux mdl/1', 'x', 1045);
blocks.route(sys, 'Machine/2', 'Mux mdl/4', 'x', 815);
blocks.route(sys, 'Machine/1', [cs '/3'], 'x', 800, 'y', y_top, 'x', 60);
blocks.route(sys, 'Mechanics/1', 'Machine/2', 'x', 1010, 'y', y_w_M, 'x', 660);
switch s.machine
    case {'sm', 'gn'}
        % The rotor angle to the machine and to the control system
        blocks.route(sys, 'Mechanics/2', 'Machine/3', ...
            'x', 1025, 'y', y_theta_M, 'x', 640);
        blocks.route(sys, 'Mechanics/2', [cs '/5'], ...
            'x', 1025, 'y', y_theta_M, 'x', 140);
    case 'im'
        % The rotor speed to the control system
        blocks.route(sys, 'Mechanics/1', [cs '/5'], 'x', 1010, 'y', y_w_M, 'x', 140);
end

% Output ports and the scope
blocks.add_outputs(sys, cs, s.scope, 1120, y_ctrl);

save_system(sys, slx);
fprintf('Saved %s\n', slx);
end

% -------------------------------------------------------------------------------
function add_synchronous_machine(blk, pos)
% Synchronous machine with constant inductances (SynchronousMachinePars), with the
% stator flux linkage in rotor coordinates as the state, as in motulator
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
    'i_s_abc = [i_alpha; -0.5*i_alpha + sqrt(3)/2*i_beta; -0.5*i_alpha - sqrt(3)/2*i_beta];'
    'tau_M = 1.5*n_p*(psi_s_dq(1)*i_q - psi_s_dq(2)*i_d);'
    };
add_machine(blk, pos, script, 'psi_s_dq', '[machine.psi_f; 0]', ...
    {'w_M', 'theta_M'}, ...
    '[machine.n_p machine.R_s machine.L_d machine.L_q machine.psi_f]');
end

function add_gradnet_machine(blk, pos, s)
% Synchronous machine with a GradNet current map with spatial harmonics
% (SpatialSaturatedSynchronousMachinePars), modeled by the S-function
% sfun_gradnet_machine with the code of the PLECS model. Its inputs are the negated
% line-to-line voltages -u_ac and -u_bc, the rotor angle, and the rotor speed, and
% its outputs the phase currents a and b, the phase currents, the torque, the
% speed, and the angle. Its parameters are in the model workspace (machine).
blocks.compile('sfun_gradnet_machine', s.folder);
add(blk, 'built-in/Subsystem', pos);
fcn = [blk '/Magnetic model'];
add(fcn, 'built-in/S-Function', [230 40 370 240], ...
    'FunctionName', 'sfun_gradnet_machine', ...
    'Parameters', strjoin(s.machine_params, ', '));
% The stator voltage as the negated line-to-line voltages
add([blk '/u_s_ab'], 'built-in/Inport', [40 0 70 14]);
add([blk '/-u_line'], 'built-in/Gain', [110 0 180 30], ...
    'Gain', '-[1.5 sqrt(3)/2; 0 sqrt(3)]', 'Multiplication', 'Matrix(K*u)');
% The angle (input 3) above the speed (input 2), in the order of the S-function
add([blk '/theta_M'], 'built-in/Inport', [40 0 70 14], 'Port', '2');
add([blk '/w_M'], 'built-in/Inport', [40 0 70 14], 'Port', '2');
y = blocks.port_y(blk, 'Magnetic model', 'Inport', 1);
blocks.align(blk, '-u_line', 'Outport', 1, y);
blocks.align(blk, 'u_s_ab', 'Outport', 1, y);
blocks.align(blk, 'theta_M', 'Outport', 1, ...
    blocks.port_y(blk, 'Magnetic model', 'Inport', 2));
blocks.align(blk, 'w_M', 'Outport', 1, ...
    blocks.port_y(blk, 'Magnetic model', 'Inport', 3));
blocks.connect(blk, 'u_s_ab/1', '-u_line/1');
blocks.connect(blk, '-u_line/1', 'Magnetic model/1');
blocks.connect(blk, 'theta_M/1', 'Magnetic model/2');
blocks.connect(blk, 'w_M/1', 'Magnetic model/3');
% Outputs: the phase currents and the torque, the others are terminated
outputs = {'i_a, i_b', 'i_s_abc', 'tau_M', 'w_M out', 'theta_M out'};
for k = 1:numel(outputs)
    if any(k == [2 3])
        add([blk '/' outputs{k}], 'built-in/Outport', [450 0 480 14]);
    else
        add([blk '/' outputs{k}], 'built-in/Terminator', [450 0 470 20]);
    end
    blocks.align(blk, outputs{k}, 'Inport', 1, ...
        blocks.port_y(blk, 'Magnetic model', 'Outport', k));
    blocks.connect(blk, sprintf('Magnetic model/%d', k), [outputs{k} '/1']);
end
set_param([blk '/i_s_abc'], 'Port', '1');
set_param([blk '/tau_M'], 'Port', '2');
end

function add_induction_machine(blk, pos)
% Induction machine with constant parameters, the inverse-Gamma model with the
% stator and rotor flux linkages in stator coordinates as the states. The model is
% equivalent to the Gamma model of InductionMachine in motulator.
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
    'i_s_abc = [i_s(1); -0.5*i_s(1) + sqrt(3)/2*i_s(2); -0.5*i_s(1) - sqrt(3)/2*i_s(2)];'
    'tau_M = 1.5*n_p*(psi_s(1)*i_s(2) - psi_s(2)*i_s(1));'
    };
add_machine(blk, pos, script, 'psi', 'zeros(4, 1)', {'w_M'}, ...
    '[machine.n_p machine.R_s machine.R_R machine.L_sgm machine.L_M]');
end

function add_machine(blk, pos, script, state, ic, inputs, par)
% Machine subsystem: a MATLAB Function block computing the derivatives of the
% flux linkages (the state of an integrator), the phase currents, and the torque
% from the stator voltage, the state, the inputs (the rotor speed and angle), and
% the parameters
add(blk, 'built-in/Subsystem', pos);
fcn = [blk '/Machine model'];
height = 44*(numel(inputs) + 3);
add(fcn, 'simulink/User-Defined Functions/MATLAB Function', [230 40 350 40 + height]);
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', fcn);
chart.Script = strjoin(script', newline);
% The state is integrated to the left of the function, with the feedback line above
add([blk '/u_s_ab'], 'built-in/Inport', [140 0 170 14]);
add([blk '/' state], 'built-in/Integrator', [140 0 170 30], 'InitialCondition', ic);
for k = 1:numel(inputs)
    add([blk '/' inputs{k}], 'built-in/Inport', [40 0 70 14]);
end
add([blk '/par'], 'built-in/Constant', [40 0 180 20], 'Value', par);
% The 3x1 currents as a 1-D vector for the S-function input
add([blk '/1-D'], 'built-in/Reshape', [400 0 430 30], ...
    'OutputDimensionality', '1-D array');
add([blk '/i_s_abc'], 'built-in/Outport', [480 0 510 14]);
add([blk '/tau_M'], 'built-in/Outport', [480 0 510 14]);
fcn_inputs = [{'u_s_ab', state}, inputs, {'par'}];
for k = 1:numel(fcn_inputs)
    y = blocks.port_y(blk, 'Machine model', 'Inport', k);
    blocks.align(blk, fcn_inputs{k}, 'Outport', 1, y);
    blocks.connect(blk, [fcn_inputs{k} '/1'], sprintf('Machine model/%d', k));
end
y = blocks.port_y(blk, 'Machine model', 'Outport', 2);
blocks.align(blk, '1-D', 'Inport', 1, y);
blocks.align(blk, 'i_s_abc', 'Inport', 1, y);
y = blocks.port_y(blk, 'Machine model', 'Outport', 3);
blocks.align(blk, 'tau_M', 'Inport', 1, y);
blocks.route(blk, 'Machine model/1', [state '/1'], 'x', 370, 'y', 15, 'x', 110);
blocks.connect(blk, 'Machine model/2', '1-D/1');
blocks.connect(blk, '1-D/1', 'i_s_abc/1');
blocks.connect(blk, 'Machine model/3', 'tau_M/1');
end

function add(blk, type, pos, varargin)
blocks.add(blk, type, pos, varargin{:});
end
