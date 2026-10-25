function add_machine(blk, pos, script, state, ic, inputs, par, folder)
%ADD_MACHINE Subsystem of a machine model.
%   ADD_MACHINE(BLK, POS, SCRIPT, STATE, IC, INPUTS, PAR) adds the subsystem BLK
%   at the position POS: a MATLAB Function block (the code SCRIPT) computing the
%   derivatives of the flux linkages (the state STATE of an integrator with the
%   initial condition IC), the phase currents, and the torque from the stator
%   voltage, the state, the inputs INPUTS (the rotor speed and angle), and the
%   parameters PAR (a Constant block). With the folder FOLDER (and SCRIPT and PAR
%   empty), the function is mfun_gradnet_machine of the folder, whose parameter
%   is the struct machine of the mask.
import motulator.internal.*
add(blk, 'built-in/Subsystem', pos);
fcn = [blk '/Machine model'];
height = 44*(numel(inputs) + 3);
if nargin < 8
    add(fcn, 'simulink/User-Defined Functions/MATLAB Function', ...
        [230 40 350 40 + height]);
    chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', fcn);
    chart.Script = strjoin(script', newline);
else
    add_mfunction(fcn, [230 40 350 40 + height], folder, ...
        'mfun_gradnet_machine', '', [2 3 1], 'machine');
end
% The state is integrated to the left of the function, with the feedback line above
add([blk '/u_s_ab'], 'built-in/Inport', [140 0 170 14]);
add([blk '/' state], 'built-in/Integrator', [140 0 170 30], ...
    'InitialCondition', ic);
for k = 1:numel(inputs)
    add([blk '/' inputs{k}], 'built-in/Inport', [40 0 70 14]);
end
if ~isempty(par)
    add([blk '/par'], 'built-in/Constant', [40 0 180 20], ...
        'Value', par);
end
% The 3x1 currents as a 1-D vector for the S-function input
add([blk '/1-D'], 'built-in/Reshape', [400 0 430 30], ...
    'OutputDimensionality', '1-D array');
add([blk '/i_s_abc'], 'built-in/Outport', [480 0 510 14]);
add([blk '/tau_M'], 'built-in/Outport', [480 0 510 14]);
fcn_inputs = [{'u_s_ab', state}, inputs];
if ~isempty(par)
    fcn_inputs{end + 1} = 'par';
end
for k = 1:numel(fcn_inputs)
    y = port_y(blk, 'Machine model', 'Inport', k);
    align(blk, fcn_inputs{k}, 'Outport', 1, y);
    connect(blk, [fcn_inputs{k} '/1'], sprintf('Machine model/%d', k));
end
y = port_y(blk, 'Machine model', 'Outport', 2);
align(blk, '1-D', 'Inport', 1, y);
align(blk, 'i_s_abc', 'Inport', 1, y);
y = port_y(blk, 'Machine model', 'Outport', 3);
align(blk, 'tau_M', 'Inport', 1, y);
route(blk, 'Machine model/1', [state '/1'], 'x', 370, 'y', 15, 'x', 110);
connect(blk, 'Machine model/2', '1-D/1');
connect(blk, '1-D/1', 'i_s_abc/1');
connect(blk, 'Machine model/3', 'tau_M/1');
end
