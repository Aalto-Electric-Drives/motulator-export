function lib = library_name(code)
%LIBRARY_NAME Name of the Simulink library of the code 'c' or 'matlab'.
%   The blocks of the control systems of the library motulator_lib are
%   S-functions running the C port of motulator, and those of the library
%   motulator_lib_matlab are MATLAB Function blocks running the MATLAB port.
switch code
    case 'c'
        lib = 'motulator_lib';
    case 'matlab'
        lib = 'motulator_lib_matlab';
    otherwise
        error('motulator:arguments', 'The code must be ''c'' or ''matlab''.');
end
end
