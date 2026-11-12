function add_control_system(blk, c, pos)
%ADD_CONTROL_SYSTEM Subsystem of a control system.
%   ADD_CONTROL_SYSTEM(BLK, C, POS) adds the control system described by the struct
%   C (written by motulator_export.simulink), in which each class of motulator is a
%   block, with the fields
%
%     mask_type      Type of the mask
%     description    Description of the mask, or of the subsystem without a mask
%     mask           Mask parameters {variable, prompt, tab, value} (only with
%                    S-functions; the MATLAB Function blocks refer to the struct
%                    ctrl_par of the parameters of the blocks, see
%                    motulator.internal.control_params)
%     mask_init      Initialization commands of the mask
%     netlist        Contents of the subsystem, see motulator.internal.add_netlist
%     sfunctions     Rows {path, parameters} of the S-functions
%     folder         Folder of the functions of the MATLAB Function blocks (only
%                    with MATLAB Function blocks)
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
if ~isempty(c.sfunctions)
    net.sfunctions = c.sfunctions;
end
add_netlist(blk, net);
if isfield(c, 'mask')
    add_tabbed_mask(blk, c);
else
    set_param(blk, 'Description', c.description);
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
        'Prompt', c.mask{k, 2}, 'Value', c.mask{k, 4}, 'Tunable', 'off', ...
        'Container', sprintf('Tab%d', tab_index(k)));
end
end
