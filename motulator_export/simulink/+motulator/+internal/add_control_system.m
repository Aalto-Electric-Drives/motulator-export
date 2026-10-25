function add_control_system(blk, c, pos)
%ADD_CONTROL_SYSTEM Masked subsystem of a control system.
%   ADD_CONTROL_SYSTEM(BLK, C, POS) adds the control system described by the struct
%   C (written by motulator_export.simulink), in which each class of motulator is a
%   block, with the fields
%
%     mask_type      Type and description of the mask
%     description
%     mask           Mask parameters {variable, prompt, tab, value}
%     mask_init      Initialization commands of the mask
%     netlist        Contents of the subsystem, see motulator.internal.add_netlist
%     sfunctions     Rows {path, parameters} of the S-functions
%     tunable        (optional) Names of the tunable mask parameters, which stay
%                    tunable in the generated code
%     folder         Folder of the functions of the MATLAB Function blocks (only
%                    with MATLAB Function blocks)
%     embed          (optional) True to copy the code of the functions into the
%                    MATLAB Function blocks, see motulator.internal.add_netlist
%     data_stores    (optional) Data Store Memory blocks {name, initial value} shared
%                    by the MATLAB Function blocks
%
%   The subsystem has the outputs 'd_abc' and 'signals' (the monitored signals).
%   The S-functions are added with dummy values, and their parameters are set
%   after the lines, see motulator.internal.add_netlist.
import motulator.internal.*

add(blk, 'built-in/Subsystem', pos);
net = c.netlist;
if isfield(c, 'folder')
    net.folder = c.folder;
end
if isfield(c, 'embed')
    net.embed = c.embed;
end
if ~isempty(c.sfunctions)
    net.sfunctions = c.sfunctions;
end
add_netlist(blk, net);
if isfield(c, 'data_stores')
    add_data_stores(blk, c.data_stores);
end
add_tabbed_mask(blk, c);
if isfield(c, 'tunable')
    % The parameters of the masks of the classes may refer to the tunable
    % parameters, so they are tunable, too (Simulink does not allow a non-tunable
    % parameter to refer to a tunable one)
    for k = 1:size(c.netlist.masks, 1)
        params = Simulink.Mask.get([blk '/' c.netlist.masks{k, 1}]).Parameters;
        for j = 1:numel(params)
            params(j).Tunable = 'on';
        end
    end
end
end

function add_data_stores(blk, stores)
% Data Store Memory blocks in a row below the other blocks, real doubles (as the
% MATLAB Function blocks require)
import motulator.internal.*
handles = setdiff(find_system(blk, 'SearchDepth', 1, 'Type', 'Block'), {blk});
pos = cell2mat(get_param(handles, 'Position'));
x = min(pos(:, 1));
y = max(pos(:, 4)) + 40;
for k = 1:size(stores, 1)
    [name, value] = stores{k, :};
    add([blk '/' name], 'built-in/DataStoreMemory', [x y x + 100 y + 30], ...
        'DataStoreName', name, 'InitialValue', value, ...
        'OutDataTypeStr', 'double', 'SignalType', 'real');
    x = x + 140;
end
end

function add_tabbed_mask(blk, c)
% Mask with the parameters on tabs
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
tunable = {};
if isfield(c, 'tunable')
    tunable = cellstr(c.tunable);
end
on_off = {'off', 'on'};
for k = 1:size(c.mask, 1)
    mask.addParameter('Type', 'edit', 'Name', c.mask{k, 1}, ...
        'Prompt', c.mask{k, 2}, 'Value', c.mask{k, 4}, ...
        'Tunable', on_off{1 + any(strcmp(c.mask{k, 1}, tunable))}, ...
        'Container', sprintf('Tab%d', tab_index(k)));
end
end
