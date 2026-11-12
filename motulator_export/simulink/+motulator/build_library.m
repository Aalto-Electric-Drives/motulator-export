function build_library()
%BUILD_LIBRARY Build the Simulink library of the blocks of motulator.
%   MOTULATOR.BUILD_LIBRARY builds the library motulator_lib, whose control
%   systems run the C port of motulator in S-functions, which are compiled with
%   mex (it needs a C99 compiler with complex.h: gcc, clang, or MinGW-w64, not
%   MSVC, see mex -setup C). The models with MATLAB Function blocks, which run the
%   MATLAB port of motulator and need no compiler, are built without a library
%   (see Simulation.build).
%
%   Each class of motulator is a block with a mask of the arguments of its class:
%   the control systems (e.g., VectorControlSystem with FluxVectorController),
%   with the blocks of their classes inside them, and the parts of the system
%   models (e.g., SynchronousMachine and MechanicalSystem). The library is saved
%   in the folder library of the package and shown in the Library Browser.
%   Simulation builds it when needed, so it rarely needs to be built by hand.
motulator.setup();
folder = motulator.internal.library_folder();
lib = 'motulator_lib';
[controls, version] = control_blocks_c();
if bdIsLoaded(lib)
    close_system(lib, 0);
end
file = fullfile(folder, [lib '.slx']);
if exist(file, 'file')
    delete(file);
end

% The S-functions of the control systems and the GradNet machine
sfunctions = {'sfun_gradnet_machine'};
for k = 1:numel(controls)
    sfunctions = [sfunctions, cellstr(controls{k}.sfunction)]; %#ok<AGROW>
end
for k = 1:numel(sfunctions)
    fprintf('Compiling %s\n', sfunctions{k});
    motulator.internal.compile(sfunctions{k}, folder);
end

new_system(lib, 'Library');
% The control systems in the first column, the parts of the drives in the second,
% and the converter and the parts of the grid converter systems in the third
y = 40;
for k = 1:numel(controls)
    c = controls{k};
    c.folder = folder;
    h = 40*numel(c.inputs) + 20;
    motulator.internal.add_control_system([lib '/' c.name], c, [40 y 180 y + h]);
    y = y + h + 60;
end
column(lib, 300, {
    @(blk, pos) motulator.internal.add_synchronous_machine(blk, pos), ...
        'SynchronousMachine', 120
    @(blk, pos) motulator.internal.add_gradnet_machine(blk, pos, 'c', folder), ...
        'SynchronousMachine (GradNet)', 120
    @(blk, pos) motulator.internal.add_induction_machine(blk, pos), ...
        'InductionMachine', 80
    @(blk, pos) motulator.internal.add_mechanics(blk, pos), 'MechanicalSystem', 80
    @(blk, pos) motulator.internal.add_external_rotor_speed(blk, pos), ...
        'ExternalRotorSpeed', 80
    });
column(lib, 500, {
    @(blk, pos) motulator.internal.add_carrier_comparison(blk, pos), ...
        'CarrierComparison', 60
    @(blk, pos) motulator.internal.add_converter(blk, pos), 'VoltageSourceConverter', 60
    @(blk, pos) motulator.internal.add_lcl_filter(blk, pos), 'LCLFilter', 120
    @(blk, pos) motulator.internal.add_l_filter(blk, pos), 'LFilter', 80
    @(blk, pos) motulator.internal.add_three_phase_source(blk, pos), ...
        'ThreePhaseSource', 40
    });

set_param(lib, 'Description', version, 'EnableLBRepository', 'on');
save_system(lib, file);
close_system(lib);
write_slblocks(folder);
fprintf('Saved %s\n', file);
end

function column(lib, x, blocks)
% Blocks {builder, name, height} from top to bottom at x
y = 40;
for k = 1:size(blocks, 1)
    [builder, name, h] = blocks{k, :};
    builder([lib '/' name], [x y x + 100 y + h]);
    y = y + h + 60;
end
end

function write_slblocks(folder)
% Register the library in the Library Browser
lines = {'function blkStruct = slblocks'
    '%SLBLOCKS Library of motulator in the Library Browser.'
    '%   Written by motulator.build_library.'
    'Browser.Library = ''motulator_lib'';'
    'Browser.Name = ''motulator'';'
    'blkStruct.Browser = Browser;'
    'end'};
fid = fopen(fullfile(folder, 'slblocks.m'), 'w');
fprintf(fid, '%s\n', lines{:});
fclose(fid);
end
