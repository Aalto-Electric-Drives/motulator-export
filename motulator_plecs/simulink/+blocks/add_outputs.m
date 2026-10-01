function add_outputs(sys, cs, scope, x0, y_ctrl)
%ADD_OUTPUTS Output ports 'mdl' and 'ctrl' and the scope.
%   ADD_OUTPUTS(SYS, CS, SCOPE, X0, Y_CTRL) adds the output port 'mdl' of the
%   signals of the system model (the output of the block 'Mux mdl') and the output
%   port 'ctrl' of the monitored signals (the output 'signals' of the control
%   system CS), and a scope of the signals selected from [mdl; ctrl] by the rows
%   {name, indices} of SCOPE. The scope is placed from X0 to the right, and the
%   monitored signals run in the lane Y_CTRL below the system model.

% Output port 'mdl'
blocks.add([sys '/mdl'], 'built-in/Outport', [x0 + 240 0 x0 + 270 14]);
blocks.align(sys, 'mdl', 'Inport', 1, blocks.port_y(sys, 'Mux mdl', 'Outport', 1));
blocks.connect(sys, 'Mux mdl/1', 'mdl/1');

% Scope, the signals selected from [mdl; ctrl]
blocks.add([sys '/Mux scope'], 'built-in/Mux', [x0 0 x0 + 5 80], 'Inputs', '2');
blocks.align(sys, 'Mux scope', 'Inport', 2, y_ctrl);
y_ctrl = blocks.port_y(sys, 'Mux scope', 'Inport', 2); % After the rounding
n = size(scope, 1);
blocks.add([sys '/Scope'], 'built-in/Scope', [x0 + 150 0 x0 + 180 40*n], ...
    'NumInputPorts', num2str(n));
y = blocks.port_y(sys, 'Mux scope', 'Outport', 1);
blocks.align(sys, 'Scope', 'Inport', min(2, n), y);
for k = 1:n
    sel = scope{k, 1};
    blocks.add([sys '/' sel], 'built-in/Selector', [x0 + 60 0 x0 + 110 20], ...
        'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', ...
        'Indices', mat2str(scope{k, 2}), 'InputPortWidth', '-1');
    blocks.align(sys, sel, 'Outport', 1, blocks.port_y(sys, 'Scope', 'Inport', k));
    blocks.route(sys, 'Mux scope/1', [sel '/1'], 'x', x0 + 30);
    blocks.connect(sys, [sel '/1'], sprintf('Scope/%d', k));
end
blocks.route(sys, 'Mux mdl/1', 'Mux scope/1', 'x', x0 - 30);
blocks.route(sys, [cs '/2'], 'Mux scope/2', 'x', 320, 'y', y_ctrl);

% Output port 'ctrl' below the scope
pos = get_param([sys '/Scope'], 'Position');
blocks.add([sys '/ctrl'], 'built-in/Outport', [x0 + 240 0 x0 + 270 14]);
blocks.align(sys, 'ctrl', 'Inport', 1, pos(4) + 40);
blocks.route(sys, [cs '/2'], 'ctrl/1', 'x', 320, 'y', y_ctrl, 'x', x0 - 20);
end
