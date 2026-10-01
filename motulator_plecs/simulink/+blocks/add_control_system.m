function add_control_system(blk, c, pos)
%ADD_CONTROL_SYSTEM Masked subsystem with the S-function of a control system.
%   ADD_CONTROL_SYSTEM(BLK, C, POS) adds the control system described by the struct
%   C (written by motulator_plecs.simulink), with the fields
%
%     sfunction      Name of the S-function
%     mask_type      Type and description of the mask
%     description
%     mask           Mask parameters {variable, prompt, tab, value}
%     mask_init      Initialization commands of the mask
%     params         Parameters of the S-function (expressions in the mask workspace)
%     inputs         Names of the inputs (the references and the measurements)
%     outputs        Names of the outputs of the S-function: the duty ratios and the
%                    groups of the monitored signals
%
%   The subsystem has the outputs 'd_abc' and 'signals' (the monitored signals).

add(blk, 'built-in/Subsystem', pos);

% Contents. The ports of the S-function are created only if its parameters can be
% evaluated, so it is added with dummy values before the mask is created.
n_in = numel(c.inputs);
n_out = numel(c.outputs);
height = 40*max(n_in, n_out);
sfun = [blk '/S-Function'];
add(sfun, 'built-in/S-Function', [160 40 300 40 + height], ...
    'FunctionName', c.sfunction, ...
    'Parameters', strjoin(repmat({'1'}, 1, numel(c.params)), ', '));
for k = 1:n_in
    add([blk '/' c.inputs{k}], 'built-in/Inport', [80 0 110 14]);
    blocks.align(blk, c.inputs{k}, 'Outport', 1, ...
        blocks.port_y(blk, 'S-Function', 'Inport', k));
    blocks.connect(blk, [c.inputs{k} '/1'], sprintf('S-Function/%d', k));
end
add([blk '/d_abc'], 'built-in/Outport', [600 0 630 14]);
blocks.align(blk, 'd_abc', 'Inport', 1, blocks.port_y(blk, 'S-Function', 'Outport', 1));
blocks.connect(blk, 'S-Function/1', 'd_abc/1');
% The groups of the monitored signals to one output. The mux is far enough for the
% names of the lines, and its height is adjusted so that its ports are aligned with
% the outputs of the S-function.
add([blk '/Mux'], 'built-in/Mux', [520 0 525 height], 'Inputs', num2str(n_out - 1));
m = n_out - 1;
for iter = 1:3*(m > 1)
    blocks.align(blk, 'Mux', 'Inport', 1, ...
        blocks.port_y(blk, 'S-Function', 'Outport', 2));
    dy = blocks.port_y(blk, 'S-Function', 'Outport', n_out) ...
        - blocks.port_y(blk, 'Mux', 'Inport', m);
    pos = get_param([blk '/Mux'], 'Position');
    set_param([blk '/Mux'], 'Position', pos + [0 0 0 round(dy*m/(m - 1))]);
end
blocks.align(blk, 'Mux', 'Inport', 1, blocks.port_y(blk, 'S-Function', 'Outport', 2));
handles = get_param(sfun, 'PortHandles');
for k = 2:n_out
    blocks.route(blk, sprintf('S-Function/%d', k), sprintf('Mux/%d', k - 1));
    set_param(get_param(handles.Outport(k), 'Line'), 'Name', c.outputs{k});
end
add([blk '/signals'], 'built-in/Outport', [600 0 630 14]);
blocks.align(blk, 'signals', 'Inport', 1, blocks.port_y(blk, 'Mux', 'Outport', 1));
blocks.connect(blk, 'Mux/1', 'signals/1');

% Mask
mask = Simulink.Mask.create(blk);
mask.Type = c.mask_type;
mask.Description = c.description;
mask.Initialization = c.mask_init;
tabs = mask.addDialogControl('tabcontainer', 'Tabs');
[tab_names, ~, tab_index] = unique(c.mask(:, 3), 'stable');
for k = 1:numel(tab_names)
    tab = tabs.addDialogControl('tab', sprintf('Tab%d', k));
    tab.Prompt = tab_names{k};
end
for k = 1:size(c.mask, 1)
    mask.addParameter('Type', 'edit', 'Name', c.mask{k, 1}, ...
        'Prompt', c.mask{k, 2}, 'Value', c.mask{k, 4}, ...
        'Tunable', 'off', 'Container', sprintf('Tab%d', tab_index(k)));
end
set_param(sfun, 'Parameters', strjoin(c.params, ', '));
end

function add(blk, type, pos, varargin)
blocks.add(blk, type, pos, varargin{:});
end
