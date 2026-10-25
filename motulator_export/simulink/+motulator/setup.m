function setup()
%SETUP Add the folder of the Simulink library of motulator to the MATLAB path.
%   MOTULATOR.SETUP adds the folder of the library, of the S-functions, and of
%   the MATLAB functions of its blocks to the MATLAB path, for the session. The
%   models built by Simulation call it when they are loaded. The package folder
%   containing +motulator must be on the path. It warns of compiled S-functions
%   of other folders that shadow those of the library, e.g., those next to the
%   models of earlier versions of motulator-export.
folder = motulator.internal.library_folder();
if ~any(strcmp(folder, strsplit(path, pathsep)))
    addpath(folder);
end
shadowing = {};
files = dir(fullfile(folder, ['sfun_*.' mexext]));
for k = 1:numel(files)
    [~, name] = fileparts(files(k).name);
    found = which(name);
    if ~isempty(found) && ~strcmp(fileparts(found), folder)
        shadowing{end + 1} = found; %#ok<AGROW>
    end
end
if ~isempty(shadowing)
    warning('motulator:shadowed', ['These compiled S-functions shadow those of ' ...
        'the library of motulator; delete them:%s'], sprintf('\n  %s', shadowing{:}));
end
end
