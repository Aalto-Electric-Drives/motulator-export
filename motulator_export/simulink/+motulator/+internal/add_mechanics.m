function add_mechanics(blk, pos)
%ADD_MECHANICS Block of MechanicalSystem without friction.
%   Integrators of the rotor speed and angle, starting from the initial state
%   w_M0 and theta_M0. The mask has the inertia J and the initial state.
import motulator.internal.*
add(blk, 'built-in/Subsystem', pos);
add([blk '/Sum'], 'built-in/Sum', [110 30 130 100], 'Inputs', '+-', ...
    'IconShape', 'rectangular');
add([blk '/tau_M'], 'built-in/Inport', [40 0 70 14]);
add([blk '/tau_L'], 'built-in/Inport', [40 0 70 14]);
add([blk '/Inverse inertia'], 'built-in/Gain', [160 0 210 30], 'Gain', '1/J');
add([blk '/Integrator w_M'], 'built-in/Integrator', [240 0 270 30], ...
    'InitialCondition', 'w_M0');
add([blk '/Integrator theta_M'], 'built-in/Integrator', [320 0 350 30], ...
    'InitialCondition', 'theta_M0');
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
add_mask(blk, 'MechanicalSystem (motulator)', ...
    ['Stiff mechanical system without friction (MechanicalSystem), starting from ' ...
    'the rotor speed w_M0 and angle theta_M0 (the state of motulator). The ' ...
    'inputs are the electromagnetic torque tau_M (Nm) and the external load ' ...
    'torque tau_L (Nm), and the outputs the rotor speed w_M (rad/s) and angle ' ...
    'theta_M (rad).'], {
    'J', 'J: Total moment of inertia (kgm^2)', 'mechanics.J'
    'w_M0', 'w_M0: Initial rotor speed (rad/s)', 'mechanics.state.w_M'
    'theta_M0', 'theta_M0: Initial rotor angle (rad)', 'mechanics.state.theta_M'
    }, '', icon(blk, 'MechanicalSystem'));
end
