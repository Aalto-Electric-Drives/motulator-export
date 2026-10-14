function add_mfunction(blk, pos, folder, name, sample_time, widths, param)
%ADD_MFUNCTION MATLAB Function block running a function of a file.
%   ADD_MFUNCTION(BLK, POS, FOLDER, NAME, SAMPLE_TIME, WIDTHS) adds the MATLAB
%   Function block BLK at the position POS, with the code of the function NAME
%   (the file FOLDER/NAME.m, written by motulator_export.simulink), the sample time
%   SAMPLE_TIME (an expression, or '' for an inherited sample time), and the widths
%   WIDTHS of the outputs. The code is copied into the block, so the model does not
%   need the file. The argument p of the function is a nontunable parameter, the
%   struct of the parameters from the mask (see blocks.add_netlist).
%
%   ADD_MFUNCTION(..., PARAM) names the parameter PARAM instead of p.
if nargin < 7
    param = 'p';
end
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
config = get_param(blk, 'MATLABFunctionConfiguration');
config.FunctionScript = fileread(fullfile(folder, [name '.m']));
if isempty(sample_time)
    config.UpdateMethod = 'Inherited';
else
    config.UpdateMethod = 'Discrete';
    config.SampleTime = sample_time;
end
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
data = chart.find('-isa', 'Stateflow.Data', 'Name', param);
if ~isempty(data)
    data.Scope = 'Parameter';
    data.Tunable = false;
end
outputs = chart.find('-isa', 'Stateflow.Data', 'Scope', 'Output');
[~, order] = sort(arrayfun(@(d) d.Port, outputs));
outputs = outputs(order);
for k = 1:numel(widths)
    outputs(k).Props.Array.Size = num2str(widths(k));
end
end
