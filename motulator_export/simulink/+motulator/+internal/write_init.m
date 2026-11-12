function write_init(file, model, sections)
%WRITE_INIT Write the initialization script of a model.
%   WRITE_INIT(FILE, MODEL, SECTIONS) writes the script FILE, which defines the
%   parameters of the model MODEL in the base workspace: the fields of the structs
%   of the rows {title, struct} of SECTIONS as variables, their values (nested
%   structs, numeric arrays, and character vectors) as assignments, e.g.,
%   ctrl_par.SpeedController.J = 0.015. The variables are cleared first, so that no
%   fields of other models are left. The model runs the script at the start of a
%   simulation (see Simulation.build).
names = {};
for k = 1:size(sections, 1)
    names = [names; fieldnames(sections{k, 2})]; %#ok<AGROW>
end
fid = fopen(file, 'w');
fprintf(fid, ['%% Parameters of the Simulink model %s, which runs this\n' ...
    '%% script at the start of a simulation (its callback InitFcn). Written by\n' ...
    '%% Simulation.build of motulator.\n'], model);
fprintf(fid, 'clear %s\n', strjoin(names', ' '));
for k = 1:size(sections, 1)
    [title, variables] = sections{k, :};
    fprintf(fid, '\n%%%% %s\n', title);
    fields = fieldnames(variables);
    for j = 1:numel(fields)
        write_value(fid, fields{j}, variables.(fields{j}));
    end
end
fclose(fid);
end

function write_value(fid, name, value)
% Assignments of a value: those of its fields for a struct
if isstruct(value)
    fields = fieldnames(value);
    for k = 1:numel(fields)
        write_value(fid, [name '.' fields{k}], value.(fields{k}));
    end
    if isempty(fields)
        fprintf(fid, '%s = struct();\n', name);
    end
elseif ischar(value) || isstring(value)
    fprintf(fid, '%s = ''%s'';\n', name, strrep(char(value), '''', ''''''));
elseif isempty(value)
    fprintf(fid, '%s = [];\n', name);
else
    % The rows of a matrix on their lines
    rows = cell(size(value, 1), 1);
    for k = 1:size(value, 1)
        rows{k} = strjoin(arrayfun(@number, double(value(k, :)), ...
            'UniformOutput', false), ' ');
    end
    text = strjoin(rows, sprintf('; ...\n    '));
    if ~isscalar(value)
        text = ['[' text ']'];
    end
    fprintf(fid, '%s = %s;\n', name, text);
end
end

function text = number(x)
% The shortest decimal number that gives the value (both parts of a complex one)
if ~isreal(x)
    text = ['complex(' number(real(x)) ', ' number(imag(x)) ')'];
    return
end
for digits = 15:17
    text = sprintf('%.*g', digits, x);
    if str2double(text) == x
        break
    end
end
end
