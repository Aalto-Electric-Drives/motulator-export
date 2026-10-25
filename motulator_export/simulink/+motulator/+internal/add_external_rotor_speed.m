function add_external_rotor_speed(blk, pos)
%ADD_EXTERNAL_ROTOR_SPEED Block of ExternalRotorSpeed.
%   Integrator of the rotor angle from the external rotor speed, starting from the
%   initial angle theta_M0 (the state of motulator). The outputs are those of the
%   block of MechanicalSystem.
import motulator.internal.*
add(blk, 'built-in/Subsystem', pos);
add([blk '/w_M_ext'], 'built-in/Inport', [40 0 70 14]);
add([blk '/Integrator theta_M'], 'built-in/Integrator', [160 0 190 30], ...
    'InitialCondition', 'theta_M0');
add([blk '/w_M'], 'built-in/Outport', [240 0 270 14]);
add([blk '/theta_M'], 'built-in/Outport', [240 0 270 14]);
y = port_y(blk, 'w_M_ext', 'Outport', 1);
align(blk, 'w_M', 'Inport', 1, y);
align(blk, 'Integrator theta_M', 'Inport', 1, y + 60);
align(blk, 'theta_M', 'Inport', 1, y + 60);
connect(blk, 'w_M_ext/1', 'w_M/1');
route(blk, 'w_M_ext/1', 'Integrator theta_M/1', 'x', 110);
connect(blk, 'Integrator theta_M/1', 'theta_M/1');
add_mask(blk, 'ExternalRotorSpeed (motulator)', ...
    ['Rotor angle integrated from the external rotor speed (ExternalRotorSpeed), ' ...
    'starting from the angle theta_M0 (the state of motulator). The input is the ' ...
    'external rotor speed w_M_ext (rad/s), and the outputs the rotor speed w_M ' ...
    '(rad/s) and angle theta_M (rad).'], {
    'theta_M0', 'theta_M0: Initial rotor angle (rad)', 'mechanics.state.theta_M'
    }, '', icon(blk, 'ExternalRotorSpeed'));
end
