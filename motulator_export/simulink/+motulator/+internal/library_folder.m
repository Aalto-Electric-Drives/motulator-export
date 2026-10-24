function folder = library_folder()
%LIBRARY_FOLDER Folder of the Simulink libraries and the code of their blocks.
%   The folder library next to the package +motulator contains the S-functions
%   (sfun_*.c, compiled there by mex), the MATLAB functions of the blocks
%   (mfun_*.m), the data of the blocks of the control systems
%   (control_blocks_*.m), and the built libraries.
folder = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'library');
end
