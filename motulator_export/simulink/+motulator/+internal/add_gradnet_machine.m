function add_gradnet_machine(blk, pos, code, folder)
%ADD_GRADNET_MACHINE Block of SynchronousMachine with a GradNet current map.
%   ADD_GRADNET_MACHINE(BLK, POS, CODE, FOLDER) adds the block of the synchronous
%   machine with a GradNet current map with spatial harmonics
%   (SpatialSaturatedSynchronousMachinePars), modeled by the S-function
%   sfun_gradnet_machine (CODE 'c', the C code of the PLECS model) or by the
%   MATLAB function mfun_gradnet_machine of the folder FOLDER (CODE 'matlab'),
%   with the stator flux linkage in rotor coordinates as the state, starting from
%   the PM-flux linkage psi_f of the map. The mask has the machine model par,
%   from which its initialization collects the struct machine of the parameters.
import motulator.internal.*
if strcmp(code, 'matlab')
    add_machine(blk, pos, '', 'psi_s_dq', '[machine.psi_f; 0]', ...
        {'w_M', 'theta_M'}, '', folder);
else
    [~, ~, plant] = control_blocks_c();
    add_sfunction(blk, pos, numel(plant.gradnet_params));
end
init = strjoin({
    '% Parameters of the machine model of the S-function or the MATLAB function'
    'map = par.magnetic_map_fcn;'
    ['machine = struct(''n_p'', par.n_p, ''R_s'', par.R_s, ''k'', map.k, ' ...
    '''psi_f'', map.psi_f, ''current_map'', map);']
    }, newline);
add_mask(blk, 'SynchronousMachine (motulator)', ...
    ['Synchronous machine with a GradNet current map with spatial harmonics ' ...
    '(SynchronousMachine with SpatialSaturatedSynchronousMachinePars), modeled in ' ...
    'rotor coordinates with the stator flux linkage as the state, starting from ' ...
    'the PM-flux linkage, as in motulator. The inputs are the stator voltage ' ...
    'u_s_ab (V) in stationary coordinates and the rotor speed w_M (rad/s) and ' ...
    'angle theta_M (rad), and the outputs the phase currents i_s_abc (A) and the ' ...
    'torque tau_M (Nm). The machine model par is a struct with the fields n_p, ' ...
    'R_s, and magnetic_map_fcn, the struct of the GradNet with the order k of the ' ...
    'harmonics and the PM-flux linkage psi_f.'], ...
    {'par', ['par: Machine model (SpatialSaturatedSynchronousMachinePars) as a ' ...
    'struct'], 'machine.par'}, init, icon(blk, 'SynchronousMachine'));
if ~strcmp(code, 'matlab')
    set_param([blk '/Magnetic model'], ...
        'Parameters', strjoin(plant.gradnet_params, ', '));
end
end

function add_sfunction(blk, pos, n_params)
% The S-function sfun_gradnet_machine, whose inputs are the negated line-to-line
% voltages -u_ac and -u_bc, the rotor angle, and the rotor speed, and outputs the
% phase currents a and b, the phase currents, the torque, the speed, and the angle.
% Its ports exist only if its parameters can be evaluated, so it is added with
% the N_PARAMS dummy parameters, which are set after the mask exists.
import motulator.internal.*
dummy = strjoin(repmat({'1'}, 1, n_params), ', ');
add(blk, 'built-in/Subsystem', pos);
add([blk '/Magnetic model'], 'built-in/S-Function', ...
    [230 40 370 240], 'FunctionName', 'sfun_gradnet_machine', 'Parameters', dummy);
% The stator voltage as the negated line-to-line voltages
add([blk '/u_s_ab'], 'built-in/Inport', [40 0 70 14]);
add([blk '/-u_line'], 'built-in/Gain', [110 0 180 30], ...
    'Gain', '-[1.5 sqrt(3)/2; 0 sqrt(3)]', 'Multiplication', 'Matrix(K*u)');
% The angle (input 3) above the speed (input 2), in the order of the S-function
add([blk '/theta_M'], 'built-in/Inport', [40 0 70 14], 'Port', '2');
add([blk '/w_M'], 'built-in/Inport', [40 0 70 14], 'Port', '2');
y = port_y(blk, 'Magnetic model', 'Inport', 1);
align(blk, '-u_line', 'Outport', 1, y);
align(blk, 'u_s_ab', 'Outport', 1, y);
align(blk, 'theta_M', 'Outport', 1, ...
    port_y(blk, 'Magnetic model', 'Inport', 2));
align(blk, 'w_M', 'Outport', 1, ...
    port_y(blk, 'Magnetic model', 'Inport', 3));
connect(blk, 'u_s_ab/1', '-u_line/1');
connect(blk, '-u_line/1', 'Magnetic model/1');
connect(blk, 'theta_M/1', 'Magnetic model/2');
connect(blk, 'w_M/1', 'Magnetic model/3');
% Outputs: the phase currents and the torque, the others are terminated
outputs = {'i_a, i_b', 'i_s_abc', 'tau_M', 'w_M out', 'theta_M out'};
for k = 1:numel(outputs)
    if any(k == [2 3])
        add([blk '/' outputs{k}], 'built-in/Outport', [450 0 480 14]);
    else
        add([blk '/' outputs{k}], 'built-in/Terminator', ...
            [450 0 470 20]);
    end
    align(blk, outputs{k}, 'Inport', 1, ...
        port_y(blk, 'Magnetic model', 'Outport', k));
    connect(blk, sprintf('Magnetic model/%d', k), [outputs{k} '/1']);
end
set_param([blk '/i_s_abc'], 'Port', '1');
set_param([blk '/tau_M'], 'Port', '2');
end
