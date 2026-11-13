function copy_functions(sys, folder, params)
%COPY_FUNCTIONS Copy the functions of a model into its folder.
%   COPY_FUNCTIONS(SYS, FOLDER, PARAMS) copies into the folder FOLDER of the model
%   SYS the m-files that the model needs, from the folder of the library: the
%   functions that its MATLAB Function blocks call (the tags of the blocks, see
%   motulator.internal.add_mfunction), the function PARAMS of the parameters of
%   the blocks of the control system (e.g., 'fvc_params'), and the functions that
%   it calls (e.g., sm_reference_luts). With the files, the model needs neither
%   the library nor the package.
import motulator.internal.*
source = library_folder();
blocks = find_system(sys, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Tag', '^mfun_');
names = unique([{params}; get_param(blocks, 'Tag')]);
% The other functions of the folder that the function of the parameters calls
code = fileread(fullfile(source, [params '.m']));
files = dir(fullfile(source, '*.m'));
for k = 1:numel(files)
    [~, name] = fileparts(files(k).name);
    if isempty(regexp(name, '^(mfun_|control_blocks_|slblocks$)|_params$', 'once')) ...
            && ~isempty(regexp(code, ['\<' name '\('], 'once'))
        names{end + 1} = name; %#ok<AGROW>
    end
end
for k = 1:numel(names)
    file = fullfile(source, [names{k} '.m']);
    target = fullfile(folder, [names{k} '.m']);
    if ~strcmp(file, target)
        copyfile(file, target);
    end
end
end
