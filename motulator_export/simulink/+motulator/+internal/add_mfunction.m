function add_mfunction(blk, pos, folder, name, sample_time, widths, param)
%ADD_MFUNCTION MATLAB Function block with the code of a function of a file.
%   ADD_MFUNCTION(BLK, POS, FOLDER, NAME, SAMPLE_TIME, WIDTHS, PARAM) adds the
%   MATLAB Function block BLK at the position POS, with the sample time
%   SAMPLE_TIME (an expression, or '' for an inherited sample time) and the widths
%   WIDTHS of the outputs. The block contains a copy of the code of the function
%   NAME (the file FOLDER/NAME.m, written by motulator_export.simulink), so that
%   the model does not need the file: the function of the block has the arguments
%   of the function NAME (the first line of the file) and calls it. Each MATLAB
%   Function block has its own copy of the persistent variables of the function.
%   The global variables of the code are Data Store Memory blocks of the model.
%
%   The last argument of the function is the struct of its parameters, which is
%   the expression PARAM of a variable of the model, e.g., 'ctrl_par.SpeedController'
%   of the struct ctrl_par of the parameters of the blocks of a control system (see
%   motulator.internal.add_netlist). The variable (e.g., ctrl_par) is a nontunable
%   parameter of the block, which passes the struct to the function. With PARAM
%   empty, the function has no parameters.
source = fileread(fullfile(folder, [name '.m']));
% The signature of the function, e.g., 'y = name(u, p)', as the function of the
% block, e.g., 'y = fcn(u, ctrl_par)', calling 'y = name(u, ctrl_par.SpeedController)'
signature = regexp(source, '^function\s+([^\r\n]*)', 'tokens', 'once');
parts = regexp(signature{1}, '^(.*\()(.*)\)\s*$', 'tokens', 'once');
[head, args] = parts{:};
args = strtrim(strsplit(args, ','));
variable = strtok(param, '.');
call_args = args;
if ~isempty(param)
    args{end} = variable;
    call_args{end} = param;
end
script = strjoin({
    ['function ' strrep(head, [name '('], 'fcn(') strjoin(args, ', ') ')']
    ['% Calls the function ' name ' below (a copy of the file ' name '.m)']
    [head strjoin(call_args, ', ') ');']
    'end'
    ''
    source
    }', newline);
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
config = get_param(blk, 'MATLABFunctionConfiguration');
config.FunctionScript = script;
if isempty(sample_time)
    config.UpdateMethod = 'Inherited';
else
    config.UpdateMethod = 'Discrete';
    config.SampleTime = sample_time;
end
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
data = chart.find('-isa', 'Stateflow.Data', 'Name', variable);
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
lines = regexp(script, '^\s*global\s+([\w ]+)', 'tokens', 'lineanchors');
for k = 1:numel(lines)
    for store = strsplit(strtrim(lines{k}{1}))
        data = chart.find('-isa', 'Stateflow.Data', 'Name', store{1});
        if isempty(data)
            data = Stateflow.Data(chart);
            data.Name = store{1};
        end
        data.Scope = 'Data Store Memory';
    end
end
% Simulink resizes the block when its code is set (by an amount depending on the
% code), so its size is restored to that of the layout, at which the blocks are
% aligned and the lines routed (a line starting at another port joins its line)
set_param(blk, 'Position', pos);
end
