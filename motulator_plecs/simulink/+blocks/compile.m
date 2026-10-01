function compile(sfunction, folder)
%COMPILE Compile the S-function folder/sfunction.c into the folder.
%   The S-functions include the C port of motulator, which uses the C99 complex
%   type, so mex needs gcc, clang, or MinGW-w64 (not MSVC), see mex -setup C.
c_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'c');
clear(sfunction);
mex('-outdir', folder, ['-I' c_dir], 'CFLAGS=$CFLAGS -std=gnu99', ...
    fullfile(folder, [sfunction '.c']));
end
