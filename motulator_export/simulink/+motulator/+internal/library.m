function lib = library(code)
%LIBRARY Simulink library of the code 'c' or 'matlab', built if needed.
%   LIB = MOTULATOR.INTERNAL.LIBRARY(CODE) loads the library, after building it
%   with motulator.build_library if it does not exist or was built from another
%   version of the data of its blocks, and returns its name.
import motulator.internal.*
motulator.setup();
lib = library_name(code);
[~, version] = feval(['control_blocks_' code]);
file = fullfile(library_folder(), [lib '.slx']);
if exist(file, 'file')
    load_system(file);
    if strcmp(get_param(lib, 'Description'), version)
        return
    end
    close_system(lib, 0);
end
motulator.build_library(code);
load_system(file);
end
