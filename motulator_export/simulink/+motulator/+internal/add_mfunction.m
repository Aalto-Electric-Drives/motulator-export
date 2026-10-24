function add_mfunction(blk, pos, folder, name, sample_time, widths, param)
%ADD_MFUNCTION MATLAB Function block calling a function of a file.
%   ADD_MFUNCTION(BLK, POS, FOLDER, NAME, SAMPLE_TIME, WIDTHS) adds the MATLAB
%   Function block BLK at the position POS, calling the function NAME (the file
%   FOLDER/NAME.m, written by motulator_export.simulink), with the sample time
%   SAMPLE_TIME (an expression, or '' for an inherited sample time) and the widths
%   WIDTHS of the outputs. The block has the arguments of the function (the first
%   line of the file), and the model needs the file on the MATLAB path (the folder
%   of the library, which motulator.setup adds to the path). Each MATLAB Function
%   block has its own copy of the persistent variables of the function. The
%   argument p of the function is a nontunable parameter, the struct of the
%   parameters from the mask (see motulator.internal.add_netlist).
%
%   ADD_MFUNCTION(..., PARAM) names the parameter PARAM instead of p.
if nargin < 7
    param = 'p';
end
% The signature of the function, e.g., 'y = name(u, p)'
signature = regexp(fileread(fullfile(folder, [name '.m'])), ...
    '^function\s+([^\r\n]*)', 'tokens', 'once');
signature = signature{1};
script = {
    ['function ' strrep(signature, [name '('], 'fcn(')]
    ['% Calls the function of the file ' name '.m of the folder of the library']
    [signature ';']
    'end'
    };
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
config = get_param(blk, 'MATLABFunctionConfiguration');
config.FunctionScript = strjoin(script', newline);
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
% Simulink resizes the block when its code is set (by an amount depending on the
% code), so its size is restored to that of the layout, at which the blocks are
% aligned and the lines routed (a line starting at another port joins its line)
set_param(blk, 'Position', pos);
end
