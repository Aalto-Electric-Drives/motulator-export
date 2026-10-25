function add_ac_filter(blk, pos, script, ic, par, outputs)
%ADD_AC_FILTER Subsystem of an AC filter model.
%   ADD_AC_FILTER(BLK, POS, SCRIPT, IC, PAR, OUTPUTS) adds the subsystem BLK at
%   the position POS: a MATLAB Function block (the code SCRIPT) computing the
%   derivatives of the states (the state of an integrator with the initial
%   condition IC) and the outputs OUTPUTS from the converter voltage, the states,
%   the grid voltage, and the parameters PAR (a Constant block).
import motulator.internal.*
add(blk, 'built-in/Subsystem', pos);
fcn = [blk '/Filter model'];
height = 44*max(4, numel(outputs) + 1);
add(fcn, 'simulink/User-Defined Functions/MATLAB Function', [230 40 350 40 + height]);
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', fcn);
chart.Script = strjoin(script', newline);
% The state is integrated to the left of the function, with the feedback line above
add([blk '/u_c_ab'], 'built-in/Inport', [140 0 170 14]);
add([blk '/x'], 'built-in/Integrator', [140 0 170 30], 'InitialCondition', ic);
add([blk '/e_g_ab'], 'built-in/Inport', [40 0 70 14]);
add([blk '/par'], 'built-in/Constant', [40 0 180 20], 'Value', par);
fcn_inputs = {'u_c_ab', 'x', 'e_g_ab', 'par'};
for k = 1:numel(fcn_inputs)
    y = port_y(blk, 'Filter model', 'Inport', k);
    align(blk, fcn_inputs{k}, 'Outport', 1, y);
    connect(blk, [fcn_inputs{k} '/1'], sprintf('Filter model/%d', k));
end
route(blk, 'Filter model/1', 'x/1', 'x', 370, 'y', 15, 'x', 110);
% The vectors as 1-D vectors for the S-function input
for k = 1:numel(outputs)
    vec = ['1-D ' outputs{k}];
    add([blk '/' vec], 'built-in/Reshape', [400 0 430 30], ...
        'OutputDimensionality', '1-D array');
    add([blk '/' outputs{k}], 'built-in/Outport', [480 0 510 14]);
    y = port_y(blk, 'Filter model', 'Outport', k + 1);
    align(blk, vec, 'Inport', 1, y);
    align(blk, outputs{k}, 'Inport', 1, y);
    connect(blk, sprintf('Filter model/%d', k + 1), [vec '/1']);
    connect(blk, [vec '/1'], [outputs{k} '/1']);
end
end
