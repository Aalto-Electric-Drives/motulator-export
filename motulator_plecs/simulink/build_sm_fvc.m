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
% the PLECS models: Dormand-Prince with the maximum step T_s
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

add_control_system([name '/Flux-vector control'], s.mask, [120 40 260 160]);
add_pwm([name '/PWM'], s.T_s, [380 50 460 110]);
add_converter([name '/Converter'], [520 50 600 110]);
add_machine([name '/Machine'], [660 50 760 130]);
add_mechanics([name '/Mechanics'], [820 50 920 110]);

% Sources
add([name '/w_M_ref'], 'built-in/Step', [40 35 70 65], 'Time', num(s.w_M_ref(1)), ...
    'Before', num(s.w_M_ref(2)), 'After', num(s.w_M_ref(3)));
add([name '/u_dc'], 'built-in/Constant', [400 150 460 170], ...
    'Value', 'converter.u_dc');
add([name '/tau_L'], 'built-in/Step', [760 150 790 180], 'Time', num(s.tau_L(1)), ...
    'Before', num(s.tau_L(2)), 'After', num(s.tau_L(3)));

% Computational delay of one sampling period
add([name '/Delay'], 'built-in/UnitDelay', [300 65 335 95], 'SampleTime', num(s.T_s));

% Signal flow
sys = name;
connect(sys, 'w_M_ref/1', 'Flux-vector control/1');
connect(sys, 'Machine/1', 'Flux-vector control/2');
connect(sys, 'u_dc/1', 'Flux-vector control/3');
connect(sys, 'Mechanics/2', 'Flux-vector control/4');
connect(sys, 'Flux-vector control/1', 'Delay/1');
connect(sys, 'Delay/1', 'PWM/1');
connect(sys, 'PWM/1', 'Converter/1');
connect(sys, 'u_dc/1', 'Converter/2');
connect(sys, 'Converter/1', 'Machine/1');
connect(sys, 'Mechanics/1', 'Machine/2');
connect(sys, 'Mechanics/2', 'Machine/3');
connect(sys, 'Machine/2', 'Mechanics/1');
connect(sys, 'tau_L/1', 'Mechanics/2');

% Output ports: the machine signals [i_a i_b i_c w_M theta_M tau_M] and the
% controller signals
add([name '/Mux mdl'], 'built-in/Mux', [980 220 985 300], 'Inputs', '4');
connect(sys, 'Machine/1', 'Mux mdl/1');
connect(sys, 'Mechanics/1', 'Mux mdl/2');
connect(sys, 'Mechanics/2', 'Mux mdl/3');
connect(sys, 'Machine/2', 'Mux mdl/4');
add([name '/mdl'], 'built-in/Outport', [1020 255 1050 270]);
connect(sys, 'Mux mdl/1', 'mdl/1');
add([name '/ctrl'], 'built-in/Outport', [1020 335 1050 350]);
connect(sys, 'Flux-vector control/2', 'ctrl/1');

% Scope: the speed and the torque (reference, estimate, and actual), the currents,
% and the flux linkage (reference and estimate)
add_selector([name '/Speed (ctrl)'], '[1 2]', [300 230 340 250]);
add_selector([name '/Torque (ctrl)'], '[3 4]', [300 280 340 300]);
add_selector([name '/Flux (ctrl)'], '[5 6]', [300 330 340 350]);
add([name '/Mux speed'], 'built-in/Mux', [400 225 405 265], 'Inputs', '2');
add([name '/Mux torque'], 'built-in/Mux', [400 275 405 315], 'Inputs', '2');
add([name '/Scope'], 'built-in/Scope', [480 230 520 350], 'NumInputPorts', '4');
for sel = {'Speed (ctrl)', 'Torque (ctrl)', 'Flux (ctrl)'}
    connect(sys, 'Flux-vector control/2', [sel{1} '/1']);
end
connect(sys, 'Speed (ctrl)/1', 'Mux speed/1');
connect(sys, 'Mechanics/1', 'Mux speed/2');
connect(sys, 'Torque (ctrl)/1', 'Mux torque/1');
connect(sys, 'Machine/2', 'Mux torque/2');
connect(sys, 'Mux speed/1', 'Scope/1');
connect(sys, 'Mux torque/1', 'Scope/2');
connect(sys, 'Machine/1', 'Scope/3');
connect(sys, 'Flux (ctrl)/1', 'Scope/4');

save_system(name, fullfile(s.folder, [name '.slx']));
fprintf('Saved %s\n', fullfile(s.folder, [name '.slx']));
end

% -------------------------------------------------------------------------------
function add_control_system(blk, mask_params, pos)
% Masked subsystem with the S-function. The mask parameters are passed to the
% S-function in their order, which must be the order of the S-function parameters.
add(blk, 'built-in/Subsystem', pos);
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

% Contents: the measurements to the S-function, the duty ratios and the monitored
% signals [w_M_ref w_M tau_M_ref tau_M psi_s_ref psi_s theta_m i_d i_q] out
inputs = {'w_M_ref', 'i_s_abc', 'u_dc', 'theta_M'};
for k = 1:numel(inputs)
    add([blk '/' inputs{k}], 'built-in/Inport', [40 30+40*k 70 45+40*k]);
end
add([blk '/S-Function'], 'built-in/S-Function', [160 50 300 210], ...
    'FunctionName', 'sfun_sm_fvc', ...
    'Parameters', strjoin(mask_params(:, 1)', ', '));
add([blk '/d_abc'], 'built-in/Outport', [380 85 410 100]);
add([blk '/signals'], 'built-in/Outport', [380 165 410 180]);
for k = 1:numel(inputs)
    connect(blk, [inputs{k} '/1'], sprintf('S-Function/%d', k));
end
connect(blk, 'S-Function/1', 'd_abc/1');
connect(blk, 'S-Function/2', 'signals/1');
end

function add_pwm(blk, T_s, pos)
% Carrier comparison: the symmetrical triangular carrier with the period 2*T_s,
% starting from its maximum, is compared with the duty ratios, which are updated
% at its extrema. This corresponds to CarrierComparison in motulator (without the
% counter quantization). The switching instants are located by the zero-crossing
% detection of the Relational Operator block.
add(blk, 'built-in/Subsystem', pos);
add([blk '/d_abc'], 'built-in/Inport', [40 43 70 57]);
add([blk '/Carrier'], 'simulink/Sources/Repeating Sequence', [30 90 70 120], ...
    'rep_seq_t', sprintf('[0 %s %s]', num(T_s), num(2*T_s)), ...
    'rep_seq_y', '[1 0 1]');
add([blk '/Compare'], 'built-in/RelationalOperator', [120 40 150 110], ...
    'Operator', '>', 'ZeroCross', 'on');
add([blk '/To double'], 'built-in/DataTypeConversion', [190 60 240 90], ...
    'OutDataTypeStr', 'double');
add([blk '/q_abc'], 'built-in/Outport', [290 68 320 82]);
connect(blk, 'd_abc/1', 'Compare/1');
connect(blk, 'Carrier/1', 'Compare/2');
connect(blk, 'Compare/1', 'To double/1');
connect(blk, 'To double/1', 'q_abc/1');
end

function add_converter(blk, pos)
% Ideal two-level converter: u_s_ab = u_dc*q_ab, where q_ab is the space vector of
% the switching states q_abc (no zero sequence, as in motulator)
add(blk, 'built-in/Subsystem', pos);
add([blk '/q_abc'], 'built-in/Inport', [40 43 70 57]);
add([blk '/u_dc'], 'built-in/Inport', [40 103 70 117]);
add([blk '/abc2ab'], 'built-in/Gain', [110 35 180 65], ...
    'Gain', '[2/3 -1/3 -1/3; 0 1/sqrt(3) -1/sqrt(3)]', ...
    'Multiplication', 'Matrix(K*u)');
add([blk '/Product'], 'built-in/Product', [230 60 260 100], 'Inputs', '**');
add([blk '/u_s_ab'], 'built-in/Outport', [300 73 330 87]);
connect(blk, 'q_abc/1', 'abc2ab/1');
connect(blk, 'abc2ab/1', 'Product/1');
connect(blk, 'u_dc/1', 'Product/2');
connect(blk, 'Product/1', 'u_s_ab/1');
end

function add_machine(blk, pos)
% Synchronous machine with constant inductances (SynchronousMachinePars), with the
% stator flux linkage in rotor coordinates as the state, as in motulator
add(blk, 'built-in/Subsystem', pos);
add([blk '/u_s_ab'], 'built-in/Inport', [40 43 70 57]);
add([blk '/w_M'], 'built-in/Inport', [40 123 70 137]);
add([blk '/theta_M'], 'built-in/Inport', [40 163 70 177]);
add([blk '/psi_s_dq'], 'built-in/Integrator', [130 75 160 105], ...
    'InitialCondition', '[machine.psi_f; 0]');
add([blk '/par'], 'built-in/Constant', [40 200 180 220], 'Value', ...
    '[machine.n_p machine.R_s machine.L_d machine.L_q machine.psi_f]');
fcn = [blk '/Machine model'];
add(fcn, 'simulink/User-Defined Functions/MATLAB Function', [230 40 350 220]);
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
% The 3x1 currents as a 1-D vector for the S-function input
add([blk '/1-D'], 'built-in/Reshape', [400 105 430 135], ...
    'OutputDimensionality', '1-D array');
add([blk '/i_s_abc'], 'built-in/Outport', [480 113 510 127]);
add([blk '/tau_M'], 'built-in/Outport', [480 173 510 187]);
connect(blk, 'u_s_ab/1', 'Machine model/1');
connect(blk, 'psi_s_dq/1', 'Machine model/2');
connect(blk, 'w_M/1', 'Machine model/3');
connect(blk, 'theta_M/1', 'Machine model/4');
connect(blk, 'par/1', 'Machine model/5');
connect(blk, 'Machine model/1', 'psi_s_dq/1');
connect(blk, 'Machine model/2', '1-D/1');
connect(blk, '1-D/1', 'i_s_abc/1');
connect(blk, 'Machine model/3', 'tau_M/1');
end

function add_mechanics(blk, pos)
% Mechanical system without friction (MechanicalSystem)
add(blk, 'built-in/Subsystem', pos);
add([blk '/tau_M'], 'built-in/Inport', [40 43 70 57]);
add([blk '/tau_L'], 'built-in/Inport', [40 83 70 97]);
add([blk '/Sum'], 'built-in/Sum', [110 50 130 70], 'Inputs', '+-');
add([blk '/Inverse inertia'], 'built-in/Gain', [160 45 210 75], 'Gain', '1/mechanics.J');
add([blk '/Integrator w_M'], 'built-in/Integrator', [240 45 270 75], ...
    'InitialCondition', '0');
add([blk '/Integrator theta_M'], 'built-in/Integrator', [300 95 330 125], ...
    'InitialCondition', '0');
add([blk '/w_M'], 'built-in/Outport', [380 53 410 67]);
add([blk '/theta_M'], 'built-in/Outport', [380 103 410 117]);
connect(blk, 'tau_M/1', 'Sum/1');
connect(blk, 'tau_L/1', 'Sum/2');
connect(blk, 'Sum/1', 'Inverse inertia/1');
connect(blk, 'Inverse inertia/1', 'Integrator w_M/1');
connect(blk, 'Integrator w_M/1', 'w_M/1');
connect(blk, 'Integrator w_M/1', 'Integrator theta_M/1');
connect(blk, 'Integrator theta_M/1', 'theta_M/1');
end

function add_selector(blk, indices, pos)
% Selector of the monitored controller signals (9 elements)
add(blk, 'built-in/Selector', pos, 'NumberOfDimensions', '1', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', indices, ...
    'InputPortWidth', '9');
end

function add(blk, type, pos, varargin)
add_block(type, blk, 'Position', pos, varargin{:});
end

function connect(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'on');
end

function str = num(x)
% Number with full precision
str = sprintf('%.17g', x);
end
