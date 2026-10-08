function add_netlist(blk, net)
%ADD_NETLIST Contents of a subsystem from a netlist of a control system.
%   ADD_NETLIST(BLK, NET) builds the contents of the subsystem BLK from the struct
%   NET (written by motulator_export.simulink), with the fields
%
%     blocks   Blocks {path, type, position, parameters}, the paths relative to
%              BLK, a subsystem before its contents. The S-functions have dummy
%              parameters until the mask of BLK exists.
%     aligns   Alignments {system, block, kind, number, target, kind, number,
%              offset}: the block is moved vertically so that its port is at the
%              height of the port of the target plus the offset.
%     lines    Lines {system, source, destination, corners}, the ports as
%              'block/number' and the corners as in blocks.route.
%     masks    Masks of the blocks of the classes {path, type, description,
%              initialization, parameters {variable, prompt, value}, display}.
%
%   The layout is computed by motulator_export, but Simulink places the ports of
%   the blocks itself, so the blocks are aligned with the ports. The unconnected
%   outputs of the S-functions and the subsystems are terminated.

types = struct('S_Function', 'built-in/S-Function', ...
    'Subsystem', 'built-in/Subsystem', 'Selector', 'built-in/Selector', ...
    'Mux', 'built-in/Mux', 'Goto', 'built-in/Goto', 'From', 'built-in/From', ...
    'Inport', 'built-in/Inport', 'Outport', 'built-in/Outport');
for k = 1:size(net.blocks, 1)
    [path, type, pos, params] = net.blocks{k, :};
    if isempty(params)
        params = {};
    end
    blocks.add([blk '/' path], types.(strrep(type, '-', '_')), pos, params{:});
end
for k = 1:size(net.aligns, 1)
    [sys, b, kind, n, t, t_kind, t_n, dy] = net.aligns{k, :};
    sys = fullpath(blk, sys);
    blocks.align(sys, b, kind, n, blocks.port_y(sys, t, t_kind, t_n) + dy);
end
for k = 1:size(net.lines, 1)
    [sys, src, dst, corners] = net.lines{k, :};
    if isempty(corners)
        corners = {};
    end
    blocks.route(fullpath(blk, sys), src, dst, corners{:});
end
% Masks of the blocks of the classes, the outer ones first
for k = 1:size(net.masks, 1)
    [path, type, description, init, params, display] = net.masks{k, :};
    mask = Simulink.Mask.create([blk '/' path]);
    mask.Type = type;
    mask.Description = description;
    mask.Initialization = init;
    mask.Display = display;
    for j = 1:size(params, 1)
        mask.addParameter('Type', 'edit', 'Name', params{j, 1}, ...
            'Prompt', params{j, 2}, 'Value', params{j, 3}, 'Tunable', 'off');
    end
end
% Terminate the unconnected outputs of the S-functions and the subsystems (e.g., the
% realized voltage of the PWM over the ongoing period, if not used)
for k = 1:size(net.blocks, 1)
    [path, type] = net.blocks{k, 1:2};
    if ~any(strcmp(type, {'S-Function', 'Subsystem'}))
        continue
    end
    handles = get_param([blk '/' path], 'PortHandles');
    for n = 1:numel(handles.Outport)
        if get_param(handles.Outport(n), 'Line') ~= -1
            continue
        end
        p = get_param(handles.Outport(n), 'Position');
        i = max([0, find(path == '/', 1, 'last')]);
        name = sprintf('%s out %d', path(i + 1:end), n);
        sys = fullpath(blk, path(1:max(i - 1, 0)));
        blocks.add([sys '/' name], 'built-in/Terminator', ...
            [p(1) + 5, p(2) - 6, p(1) + 15, p(2) + 6], 'ShowName', 'off');
        add_line(sys, sprintf('%s/%d', path(i + 1:end), n), [name '/1']);
    end
end
end

function sys = fullpath(blk, path)
% System of a path relative to the subsystem
if isempty(path)
    sys = blk;
else
    sys = [blk '/' path];
end
end
