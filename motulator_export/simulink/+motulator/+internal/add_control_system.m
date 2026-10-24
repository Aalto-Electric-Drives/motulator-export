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
%     folder         Folder of the functions of the MATLAB Function blocks (only
%                    with MATLAB Function blocks)
%
%   The subsystem has the outputs 'd_abc' and 'signals' (the monitored signals).
%   The ports of an S-function are created only if its parameters can be evaluated,
%   so the S-functions are added with dummy values and their parameters are set
%   after the masks are created.
import motulator.internal.*

add(blk, 'built-in/Subsystem', pos);
net = c.netlist;
if isfield(c, 'folder')
    net.folder = c.folder;
end
add_netlist(blk, net);
add_tabbed_mask(blk, c);
for k = 1:size(c.sfunctions, 1)
    set_param([blk '/' c.sfunctions{k, 1}], 'Parameters', c.sfunctions{k, 2});
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
for k = 1:size(c.mask, 1)
    mask.addParameter('Type', 'edit', 'Name', c.mask{k, 1}, ...
        'Prompt', c.mask{k, 2}, 'Value', c.mask{k, 4}, ...
        'Tunable', 'off', 'Container', sprintf('Tab%d', tab_index(k)));
end
end
