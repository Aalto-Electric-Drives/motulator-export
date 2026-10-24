function add_netlist(blk, net)
%ADD_NETLIST Contents of a subsystem from a netlist of a control system.
%   ADD_NETLIST(BLK, NET) builds the contents of the subsystem BLK from the struct
%   NET (written by motulator_export.simulink), with the fields
%
%     blocks   Blocks {path, type, position, parameters}, the paths relative to
%              BLK, a subsystem before its contents. The S-functions have dummy
%              parameters until the lines exist. The parameters of a MATLAB
%              Function block are {function, sample time, output widths}, see
%              motulator.internal.add_mfunction.
%     aligns   Alignments {system, block, kind, number, target, kind, number,
%              offset}: the block is moved vertically so that its port is at the
%              height of the port of the target plus the offset.
%     lines    Lines {system, source, destination, corners}, the ports as
%              'block/number' and the corners as in motulator.internal.route.
%     masks    Masks of the blocks of the classes {path, type, description,
%              initialization, parameters {variable, prompt, value}, display}.
%     sfunctions  Parameters of the S-functions {path, parameters} (only with
%              S-functions)
%     folder   Folder of the files of the functions of the MATLAB Function blocks
%              (only with MATLAB Function blocks)
%
%   The layout is computed by motulator_export, but Simulink places the ports of
%   the blocks itself, so the blocks are aligned with the ports. The unconnected
%   outputs of the S-functions, the MATLAB Function blocks, and the subsystems are
%   terminated.
import motulator.internal.*

types = struct('S_Function', 'built-in/S-Function', ...
    'Subsystem', 'built-in/Subsystem', 'Selector', 'built-in/Selector', ...
    'Mux', 'built-in/Mux', 'Goto', 'built-in/Goto', 'From', 'built-in/From', ...
    'Inport', 'built-in/Inport', 'Outport', 'built-in/Outport', ...
    'UnitDelay', 'built-in/UnitDelay');
for k = 1:size(net.blocks, 1)
    [path, type, pos, params] = net.blocks{k, :};
    if isempty(params)
        params = {};
    end
    if strcmp(type, 'MATLAB Function')
        add_mfunction([blk '/' path], pos, net.folder, params{:});
    else
        add([blk '/' path], types.(strrep(type, '-', '_')), pos, params{:});
    end
end
for k = 1:size(net.aligns, 1)
    [sys, b, kind, n, t, t_kind, t_n, dy] = net.aligns{k, :};
    sys = fullpath(blk, sys);
    align(sys, b, kind, n, port_y(sys, t, t_kind, t_n) + dy);
end
for k = 1:size(net.lines, 1)
    [sys, src, dst, corners] = net.lines{k, :};
    if isempty(corners)
        corners = {};
    end
    route(fullpath(blk, sys), src, dst, corners{:});
end
% The ports of an S-function are created only if its parameters can be evaluated,
% so its parameters are set after the lines. They are set before the masks, since
% setting them evaluates the mask of the S-function, which fails in a library.
if isfield(net, 'sfunctions')
    for k = 1:size(net.sfunctions, 1)
        set_param([blk '/' net.sfunctions{k, 1}], ...
            'Parameters', net.sfunctions{k, 2});
    end
end
% Masks of the blocks of the classes, the outer ones first
for k = 1:size(net.masks, 1)
    [path, type, description, init, params, display] = net.masks{k, :};
    add_mask([blk '/' path], type, description, params, init, ...
        display);
end
% Terminate the unconnected outputs of the S-functions, the MATLAB Function blocks,
% and the subsystems (e.g., the realized voltage of the PWM over the ongoing period,
% if not used)
for k = 1:size(net.blocks, 1)
    [path, type] = net.blocks{k, 1:2};
    if ~any(strcmp(type, {'S-Function', 'MATLAB Function', 'Subsystem'}))
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
        add([sys '/' name], 'built-in/Terminator', ...
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
