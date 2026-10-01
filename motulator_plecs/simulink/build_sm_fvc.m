function build_sm_fvc(s)
%BUILD_SM_FVC Build a Simulink model of a synchronous machine drive.
%   BUILD_SM_FVC(S) compiles the S-function sfun_sm_fvc, builds the model S.name
%   with flux-vector control, and saves it in the folder S.folder. The struct S is
%   written by motulator_plecs.simulink.sm.write_model, with the fields
%
%     name     Model name
%     folder   Folder of the model and the compiled S-function
%     init     MATLAB code of the model workspace (machine, mechanics, converter)
%     mask     Mask parameters of the control system {variable, prompt, tab, value}
%     w_M_ref  Speed reference step [time, before, after]
%     tau_L    Load torque step [time, before, after]
%     T_s      Sampling period (s)
%     t_stop   Stop time (s)
%
%   The control system is the S-function (the C port of motulator) in a masked
%   subsystem, whose parameters follow the motulator API. The system model is built
%   from basic Simulink blocks: the carrier comparison, the ideal converter with a
%   stiff DC bus, the machine in rotor coordinates, and the mechanics. The root-level
%   output ports 'mdl' and 'ctrl' give the machine and controller signals for the
%   comparison with motulator.
%
%   The blocks are aligned with the ports they connect to, so that the lines are
%   straight, and the feedback lines are drawn through given corners.

name = s.name;
here = fileparts(mfilename('fullpath'));

% Compile the S-function into the model folder. The C port uses the C99 complex
% type, which needs gcc, clang, or MinGW-w64 (not MSVC).
if bdIsLoaded(name)
    close_system(name, 0);
end
clear sfun_sm_fvc
mex('-outdir', s.folder, ['-I' fullfile(here, '..', 'c')], ...
    'CFLAGS=$CFLAGS -std=gnu99', fullfile(here, 'sfun_sm_fvc.c'));
addpath(s.folder);

% Model, its workspace (the parameters of the system model), and the solver as in
% the PLECS models: Dormand-Prince with the maximum step T_s. The previous model is
% deleted, since it would shadow the new one.
slx = fullfile(s.folder, [name '.slx']);
if isfile(slx)
    delete(slx);
end
new_system(name);
ws = get_param(name, 'ModelWorkspace');
ws.DataSource = 'MATLAB Code';
ws.MATLABCode = s.init;
ws.reload;
set_param(name, ...
    'Solver', 'ode45', ...
    'MaxStep', num(s.T_s), ...
    'RelTol', '1e-6', ...
    'AbsTol', 'auto', ...
    'StopTime', num(s.t_stop), ...
    'SaveTime', 'on', ...
    'SaveOutput', 'on', ...
    'SaveFormat', 'Array', ...
    'LimitDataPoints', 'off', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'InitFcn', 'addpath(fileparts(get_param(bdroot, ''FileName'')))');

% Main path in a row: the control system, the computational delay of one sampling
% period, the PWM, the converter, the machine, and the mechanics
sys = name;
add_control_system([sys '/Flux-vector control'], s.mask, [160 100 300 300]);
add([sys '/Delay'], 'built-in/UnitDelay', [340 0 375 30], 'SampleTime', num(s.T_s));
add_pwm([sys '/PWM'], s.T_s, [410 0 490 60]);
add_converter([sys '/Converter'], [530 0 610 60]);
add_machine([sys '/Machine'], [680 0 780 120]);
add_mechanics([sys '/Mechanics'], [890 0 990 80]);
y = port_y(sys, 'Flux-vector control', 'Outport', 1);
for blk = {'Delay', 'PWM', 'Converter', 'Machine'}
    align(sys, blk{1}, 'Inport', 1, y);
end
align(sys, 'Mechanics', 'Inport', 1, port_y(sys, 'Machine', 'Outport', 2));

% Sources
add([sys '/w_M_ref'], 'built-in/Step', [90 0 120 30], 'Time', num(s.w_M_ref(1)), ...
    'Before', num(s.w_M_ref(2)), 'After', num(s.w_M_ref(3)), 'SampleTime', '0');
align(sys, 'w_M_ref', 'Outport', 1, port_y(sys, 'Flux-vector control', 'Inport', 1));
add([sys '/u_dc'], 'built-in/Constant', [80 0 130 20], 'Value', 'converter.u_dc');
align(sys, 'u_dc', 'Outport', 1, port_y(sys, 'Flux-vector control', 'Inport', 3));
add([sys '/tau_L'], 'built-in/Step', [835 0 865 30], 'Time', num(s.tau_L(1)), ...
    'Before', num(s.tau_L(2)), 'After', num(s.tau_L(3)), 'SampleTime', '0');
align(sys, 'tau_L', 'Outport', 1, port_y(sys, 'Mechanics', 'Inport', 2));

% Machine signals [i_a i_b i_c w_M theta_M tau_M] to the output port 'mdl'
add([sys '/Mux mdl'], 'built-in/Mux', [1060 0 1065 160], 'Inputs', '4');
align(sys, 'Mux mdl', 'Inport', 2, port_y(sys, 'Mechanics', 'Outport', 1));
add([sys '/mdl'], 'built-in/Outport', [1360 0 1390 14]);
align(sys, 'mdl', 'Inport', 1, port_y(sys, 'Mux mdl', 'Outport', 1));

% Signal flow. The current is fed back above the row and the speed and the angle
% below it, in lanes below the blocks.
pos = get_param([sys '/Mux mdl'], 'Position');
y_top = 60;
y_w_M = max(pos(4), port_y(sys, 'Flux-vector control', 'Inport', 4)) + 40;
y_theta_M = y_w_M + 30;
y_ctrl = y_theta_M + 30;
connect(sys, 'w_M_ref/1', 'Flux-vector control/1');
connect(sys, 'u_dc/1', 'Flux-vector control/3');
connect(sys, 'Flux-vector control/1', 'Delay/1');
connect(sys, 'Delay/1', 'PWM/1');
connect(sys, 'PWM/1', 'Converter/1');
connect(sys, 'Converter/1', 'Machine/1');
connect(sys, 'Machine/2', 'Mechanics/1');
connect(sys, 'tau_L/1', 'Mechanics/2');
connect(sys, 'Mechanics/1', 'Mux mdl/2');
connect(sys, 'Mechanics/2', 'Mux mdl/3');
connect(sys, 'Mux mdl/1', 'mdl/1');
route(sys, 'Machine/1', 'Mux mdl/1', 'x', 1045);
route(sys, 'Machine/2', 'Mux mdl/4', 'x', 815);
route(sys, 'Machine/1', 'Flux-vector control/2', 'x', 800, 'y', y_top, 'x', 60);
route(sys, 'Mechanics/1', 'Machine/2', 'x', 1010, 'y', y_w_M, 'x', 660);
route(sys, 'Mechanics/2', 'Machine/3', 'x', 1025, 'y', y_theta_M, 'x', 640);
route(sys, 'Mechanics/2', 'Flux-vector control/4', 'x', 1025, 'y', y_theta_M, ...
    'x', 140);

% Scope: the speed and the torque (reference, estimate, and actual), the currents,
% and the flux linkage (reference and estimate), selected from [mdl; ctrl]
add([sys '/Mux scope'], 'built-in/Mux', [1120 0 1125 80], 'Inputs', '2');
align(sys, 'Mux scope', 'Inport', 2, y_ctrl);
add([sys '/Scope'], 'built-in/Scope', [1270 0 1300 160], 'NumInputPorts', '4');
align(sys, 'Scope', 'Inport', 2, port_y(sys, 'Mux scope', 'Outport', 1));
selectors = {'Speed', '[7 8 4]'; 'Torque', '[9 10 6]'; 'Current', '[1 2 3]'
    'Flux', '[11 12]'};
for k = 1:size(selectors, 1)
    add_selector([sys '/' selectors{k, 1}], selectors{k, 2}, [1180 0 1230 20]);
    align(sys, selectors{k, 1}, 'Outport', 1, port_y(sys, 'Scope', 'Inport', k));
    route(sys, 'Mux scope/1', [selectors{k, 1} '/1'], 'x', 1150);
    connect(sys, [selectors{k, 1} '/1'], sprintf('Scope/%d', k));
end
route(sys, 'Mux mdl/1', 'Mux scope/1', 'x', 1090);
route(sys, 'Flux-vector control/2', 'Mux scope/2', 'x', 320, 'y', y_ctrl);

% Controller signals to the output port 'ctrl'
pos = get_param([sys '/Scope'], 'Position');
add([sys '/ctrl'], 'built-in/Outport', [1360 0 1390 14]);
align(sys, 'ctrl', 'Inport', 1, pos(4) + 40);
route(sys, 'Flux-vector control/2', 'ctrl/1', 'x', 320, 'y', y_ctrl, 'x', 1100);

save_system(name, slx);
fprintf('Saved %s\n', slx);
end

% -------------------------------------------------------------------------------
function add_control_system(blk, mask_params, pos)
% Masked subsystem with the S-function. The mask parameters are passed to the
% S-function in their order, which must be the order of the S-function parameters.
add(blk, 'built-in/Subsystem', pos);

% Contents: the measurements to the S-function, the duty ratios and the monitored
% signals [w_M_ref w_M tau_M_ref tau_M psi_s_ref psi_s theta_m i_d i_q] out. The
% ports of the S-function are created only if its parameters can be evaluated, so
% it is added with the values before the mask is created.
sfun = [blk '/S-Function'];
add(sfun, 'built-in/S-Function', [160 50 300 210], ...
    'FunctionName', 'sfun_sm_fvc', ...
    'Parameters', strjoin(mask_params(:, 4)', ', '));
inputs = {'w_M_ref', 'i_s_abc', 'u_dc', 'theta_M'};
for k = 1:numel(inputs)
    add([blk '/' inputs{k}], 'built-in/Inport', [80 0 110 14]);
    align(blk, inputs{k}, 'Outport', 1, port_y(blk, 'S-Function', 'Inport', k));
    connect(blk, [inputs{k} '/1'], sprintf('S-Function/%d', k));
end
outputs = {'d_abc', 'signals'};
for k = 1:numel(outputs)
    add([blk '/' outputs{k}], 'built-in/Outport', [350 0 380 14]);
    align(blk, outputs{k}, 'Inport', 1, port_y(blk, 'S-Function', 'Outport', k));
    connect(blk, sprintf('S-Function/%d', k), [outputs{k} '/1']);
end

% Mask
mask = Simulink.Mask.create(blk);
mask.Type = 'Flux-vector control (motulator)';
mask.Description = ['Speed control of a synchronous machine drive with ' ...
    'flux-vector control. The parameters correspond to the motulator API: ' ...
    'SynchronousMachinePars, FluxVectorControllerCfg, and SpeedController. ' ...
    'Empty parameters ([]) correspond to None, i.e., the defaults of motulator.'];
tabs = mask.addDialogControl('tabcontainer', 'Tabs');
[tab_names, ~, tab_index] = unique(mask_params(:, 3), 'stable');
for k = 1:numel(tab_names)
    tab = tabs.addDialogControl('tab', sprintf('Tab%d', k));
    tab.Prompt = tab_names{k};
end
for k = 1:size(mask_params, 1)
    mask.addParameter('Type', 'edit', 'Name', mask_params{k, 1}, ...
        'Prompt', mask_params{k, 2}, 'Value', mask_params{k, 4}, ...
        'Tunable', 'off', 'Container', sprintf('Tab%d', tab_index(k)));
end
set_param(sfun, 'Parameters', strjoin(mask_params(:, 1)', ', '));
end

function add_pwm(blk, T_s, pos)
% Carrier comparison: the symmetrical triangular carrier with the period 2*T_s,
% starting from its maximum, is compared with the duty ratios, which are updated
% at its extrema. This corresponds to CarrierComparison in motulator (without the
% counter quantization). The switching instants are located by the zero-crossing
% detection of the Relational Operator block.
add(blk, 'built-in/Subsystem', pos);
add([blk '/Compare'], 'built-in/RelationalOperator', [130 40 160 120], ...
    'Operator', '>', 'ZeroCross', 'on');
add([blk '/d_abc'], 'built-in/Inport', [40 0 70 14]);
add([blk '/Carrier'], 'simulink/Sources/Repeating Sequence', [40 0 80 30], ...
    'rep_seq_t', sprintf('[0 %s %s]', num(T_s), num(2*T_s)), ...
    'rep_seq_y', '[1 0 1]');
add([blk '/To double'], 'built-in/DataTypeConversion', [200 0 250 30], ...
    'OutDataTypeStr', 'double');
add([blk '/q_abc'], 'built-in/Outport', [290 0 320 14]);
align(blk, 'd_abc', 'Outport', 1, port_y(blk, 'Compare', 'Inport', 1));
align(blk, 'Carrier', 'Outport', 1, port_y(blk, 'Compare', 'Inport', 2));
align(blk, 'To double', 'Inport', 1, port_y(blk, 'Compare', 'Outport', 1));
align(blk, 'q_abc', 'Inport', 1, port_y(blk, 'Compare', 'Outport', 1));
connect(blk, 'd_abc/1', 'Compare/1');
connect(blk, 'Carrier/1', 'Compare/2');
connect(blk, 'Compare/1', 'To double/1');
connect(blk, 'To double/1', 'q_abc/1');
end

function add_converter(blk, pos)
% Ideal two-level converter with a stiff DC bus: u_s_ab = u_dc*q_ab, where q_ab is
% the space vector of the switching states q_abc (no zero sequence, as in motulator)
add(blk, 'built-in/Subsystem', pos);
add([blk '/Product'], 'built-in/Product', [230 50 260 130], 'Inputs', '**');
add([blk '/q_abc'], 'built-in/Inport', [40 0 70 14]);
add([blk '/abc2ab'], 'built-in/Gain', [110 0 180 30], ...
    'Gain', '[2/3 -1/3 -1/3; 0 1/sqrt(3) -1/sqrt(3)]', ...
    'Multiplication', 'Matrix(K*u)');
add([blk '/u_dc'], 'built-in/Constant', [110 0 180 20], 'Value', 'converter.u_dc');
add([blk '/u_s_ab'], 'built-in/Outport', [300 0 330 14]);
align(blk, 'abc2ab', 'Outport', 1, port_y(blk, 'Product', 'Inport', 1));
align(blk, 'q_abc', 'Outport', 1, port_y(blk, 'Product', 'Inport', 1));
align(blk, 'u_dc', 'Outport', 1, port_y(blk, 'Product', 'Inport', 2));
align(blk, 'u_s_ab', 'Inport', 1, port_y(blk, 'Product', 'Outport', 1));
connect(blk, 'q_abc/1', 'abc2ab/1');
connect(blk, 'abc2ab/1', 'Product/1');
connect(blk, 'u_dc/1', 'Product/2');
connect(blk, 'Product/1', 'u_s_ab/1');
end

function add_machine(blk, pos)
% Synchronous machine with constant inductances (SynchronousMachinePars), with the
% stator flux linkage in rotor coordinates as the state, as in motulator
add(blk, 'built-in/Subsystem', pos);
fcn = [blk '/Machine model'];
add(fcn, 'simulink/User-Defined Functions/MATLAB Function', [230 40 350 260]);
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', fcn);
chart.Script = strjoin({ ...
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
    }', newline);
% The flux linkage is integrated to the left of the function, with the feedback
% line above it
add([blk '/u_s_ab'], 'built-in/Inport', [140 0 170 14]);
add([blk '/psi_s_dq'], 'built-in/Integrator', [140 0 170 30], ...
    'InitialCondition', '[machine.psi_f; 0]');
add([blk '/w_M'], 'built-in/Inport', [40 0 70 14]);
add([blk '/theta_M'], 'built-in/Inport', [40 0 70 14]);
add([blk '/par'], 'built-in/Constant', [40 0 180 20], 'Value', ...
    '[machine.n_p machine.R_s machine.L_d machine.L_q machine.psi_f]');
% The 3x1 currents as a 1-D vector for the S-function input
add([blk '/1-D'], 'built-in/Reshape', [400 0 430 30], ...
    'OutputDimensionality', '1-D array');
add([blk '/i_s_abc'], 'built-in/Outport', [480 0 510 14]);
add([blk '/tau_M'], 'built-in/Outport', [480 0 510 14]);
inputs = {'u_s_ab', 'psi_s_dq', 'w_M', 'theta_M', 'par'};
for k = 1:numel(inputs)
    align(blk, inputs{k}, 'Outport', 1, port_y(blk, 'Machine model', 'Inport', k));
    connect(blk, [inputs{k} '/1'], sprintf('Machine model/%d', k));
end
align(blk, '1-D', 'Inport', 1, port_y(blk, 'Machine model', 'Outport', 2));
align(blk, 'i_s_abc', 'Inport', 1, port_y(blk, 'Machine model', 'Outport', 2));
align(blk, 'tau_M', 'Inport', 1, port_y(blk, 'Machine model', 'Outport', 3));
route(blk, 'Machine model/1', 'psi_s_dq/1', 'x', 370, 'y', 15, 'x', 110);
connect(blk, 'Machine model/2', '1-D/1');
connect(blk, '1-D/1', 'i_s_abc/1');
connect(blk, 'Machine model/3', 'tau_M/1');
end

function add_mechanics(blk, pos)
% Mechanical system without friction (MechanicalSystem)
add(blk, 'built-in/Subsystem', pos);
add([blk '/Sum'], 'built-in/Sum', [110 30 130 100], 'Inputs', '+-', ...
    'IconShape', 'rectangular');
add([blk '/tau_M'], 'built-in/Inport', [40 0 70 14]);
add([blk '/tau_L'], 'built-in/Inport', [40 0 70 14]);
add([blk '/Inverse inertia'], 'built-in/Gain', [160 0 210 30], ...
    'Gain', '1/mechanics.J');
add([blk '/Integrator w_M'], 'built-in/Integrator', [240 0 270 30], ...
    'InitialCondition', '0');
add([blk '/Integrator theta_M'], 'built-in/Integrator', [320 0 350 30], ...
    'InitialCondition', '0');
add([blk '/w_M'], 'built-in/Outport', [400 0 430 14]);
add([blk '/theta_M'], 'built-in/Outport', [400 0 430 14]);
y = port_y(blk, 'Sum', 'Outport', 1);
align(blk, 'tau_M', 'Outport', 1, port_y(blk, 'Sum', 'Inport', 1));
align(blk, 'tau_L', 'Outport', 1, port_y(blk, 'Sum', 'Inport', 2));
for b = {'Inverse inertia', 'Integrator w_M', 'w_M'}
    align(blk, b{1}, 'Inport', 1, y);
end
align(blk, 'Integrator theta_M', 'Inport', 1, y + 60);
align(blk, 'theta_M', 'Inport', 1, y + 60);
connect(blk, 'tau_M/1', 'Sum/1');
connect(blk, 'tau_L/1', 'Sum/2');
connect(blk, 'Sum/1', 'Inverse inertia/1');
connect(blk, 'Inverse inertia/1', 'Integrator w_M/1');
connect(blk, 'Integrator w_M/1', 'w_M/1');
route(blk, 'Integrator w_M/1', 'Integrator theta_M/1', 'x', 295);
connect(blk, 'Integrator theta_M/1', 'theta_M/1');
end

function add_selector(blk, indices, pos)
% Selector of the scope signals from [mdl; ctrl] (6 + 9 elements)
add(blk, 'built-in/Selector', pos, 'NumberOfDimensions', '1', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', indices, ...
    'InputPortWidth', '15');
end

function add(blk, type, pos, varargin)
add_block(type, blk, 'Position', pos, varargin{:});
end

function connect(sys, src, dst)
% Straight line between aligned ports ('block/port')
add_line(sys, src, dst);
end

function route(sys, src, dst, varargin)
% Line from the output port src to the input port dst ('block/port') through the
% corners given by the pairs 'x', value (horizontal segment to x) and 'y', value
% (vertical segment to y). The line enters dst horizontally.
p = port_pos(sys, src, 'Outport');
q = port_pos(sys, dst, 'Inport');
points = p;
for k = 1:2:numel(varargin)
    if varargin{k} == 'x'
        p = [varargin{k + 1} p(2)];
    else
        p = [p(1) varargin{k + 1}];
    end
    points(end + 1, :) = p; %#ok<AGROW>
end
if p(2) ~= q(2)
    points(end + 1, :) = [p(1) q(2)];
end
add_line(sys, [points; q]);
end

function p = port_pos(sys, port, kind)
% Position of the port 'block/port' of the kind 'Inport' or 'Outport'
k = find(port == '/', 1, 'last');
handles = get_param([sys '/' port(1:k - 1)], 'PortHandles');
p = get_param(handles.(kind)(str2double(port(k + 1:end))), 'Position');
end

function y = port_y(sys, blk, kind, n)
% Vertical position of the port n of the kind 'Inport' or 'Outport'
p = port_pos(sys, sprintf('%s/%d', blk, n), kind);
y = p(2);
end

function align(sys, blk, kind, n, y)
% Move the block vertically so that its port n of the kind 'Inport' or 'Outport'
% is at y
pos = get_param([sys '/' blk], 'Position');
dy = y - port_y(sys, blk, kind, n);
set_param([sys '/' blk], 'Position', pos + [0 dy 0 dy]);
end

function str = num(x)
% Number with full precision
str = sprintf('%.17g', x);
end
