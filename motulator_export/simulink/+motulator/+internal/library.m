function lib = library()
%LIBRARY Simulink library of motulator, built if needed.
%   LIB = MOTULATOR.INTERNAL.LIBRARY() loads the library motulator_lib, after
%   building it with motulator.build_library if it does not exist or was built
%   from another version of the data of its blocks, and returns its name.
import motulator.internal.*
motulator.setup();
lib = 'motulator_lib';
[~, version] = control_blocks_c();
file = fullfile(library_folder(), [lib '.slx']);
if exist(file, 'file')
    load_system(file);
    if strcmp(get_param(lib, 'Description'), version)
        return
    end
    close_system(lib, 0);
end
motulator.build_library();
load_system(file);
end
