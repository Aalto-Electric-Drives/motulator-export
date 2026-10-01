function add_mechanics(blk, pos)
%ADD_MECHANICS Mechanical system without friction (MechanicalSystem).
blocks.add(blk, 'built-in/Subsystem', pos);
blocks.add([blk '/Sum'], 'built-in/Sum', [110 30 130 100], 'Inputs', '+-', ...
    'IconShape', 'rectangular');
blocks.add([blk '/tau_M'], 'built-in/Inport', [40 0 70 14]);
blocks.add([blk '/tau_L'], 'built-in/Inport', [40 0 70 14]);
blocks.add([blk '/Inverse inertia'], 'built-in/Gain', [160 0 210 30], ...
    'Gain', '1/mechanics.J');
blocks.add([blk '/Integrator w_M'], 'built-in/Integrator', [240 0 270 30], ...
    'InitialCondition', '0');
blocks.add([blk '/Integrator theta_M'], 'built-in/Integrator', [320 0 350 30], ...
    'InitialCondition', '0');
blocks.add([blk '/w_M'], 'built-in/Outport', [400 0 430 14]);
blocks.add([blk '/theta_M'], 'built-in/Outport', [400 0 430 14]);
y = blocks.port_y(blk, 'Sum', 'Outport', 1);
blocks.align(blk, 'tau_M', 'Outport', 1, blocks.port_y(blk, 'Sum', 'Inport', 1));
blocks.align(blk, 'tau_L', 'Outport', 1, blocks.port_y(blk, 'Sum', 'Inport', 2));
for b = {'Inverse inertia', 'Integrator w_M', 'w_M'}
    blocks.align(blk, b{1}, 'Inport', 1, y);
end
blocks.align(blk, 'Integrator theta_M', 'Inport', 1, y + 60);
blocks.align(blk, 'theta_M', 'Inport', 1, y + 60);
blocks.connect(blk, 'tau_M/1', 'Sum/1');
blocks.connect(blk, 'tau_L/1', 'Sum/2');
blocks.connect(blk, 'Sum/1', 'Inverse inertia/1');
blocks.connect(blk, 'Inverse inertia/1', 'Integrator w_M/1');
blocks.connect(blk, 'Integrator w_M/1', 'w_M/1');
blocks.route(blk, 'Integrator w_M/1', 'Integrator theta_M/1', 'x', 295);
blocks.connect(blk, 'Integrator theta_M/1', 'theta_M/1');
end
