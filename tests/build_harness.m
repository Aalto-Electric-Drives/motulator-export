function build_harness(name, folder, block, init_script, input_widths, T_s)
%BUILD_HARNESS Build a model running a control system of the library alone.
%   BUILD_HARNESS(NAME, FOLDER, BLOCK, INIT_SCRIPT, INPUT_WIDTHS, T_S) builds the
%   model NAME in the folder FOLDER, for the tests, with the control system BLOCK
%   of the library with MATLAB Function blocks (motulator_lib_matlab), the
%   root-level inputs of its inputs (of the widths INPUT_WIDTHS, given by the
%   external input of sim), and the outputs 'd_abc' and 'signals'. The script
%   INIT_SCRIPT of the folder defines the variables of the mask of the block in the
%   base workspace, and the model runs it at the start of each simulation. The
%   solver is discrete with the fixed step T_S (an expression), so that the outputs
%   are those of the sampling instants.
lib = motulator.internal.library('matlab');
evalin('base', init_script);
if bdIsLoaded(name)
    close_system(name, 0);
end
slx = fullfile(folder, [name '.slx']);
if exist(slx, 'file')
    delete(slx);
end
new_system(name);
set_param(name, 'InitFcn', [init_script ';'], 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', T_s, 'SaveTime', 'on', 'SaveOutput', 'on', 'SaveFormat', 'Array', ...
    'LimitDataPoints', 'off', 'ReturnWorkspaceOutputs', 'on');
n = numel(input_widths);
add_block([lib '/' block], [name '/Control system'], ...
    'Position', [300 40 440 40 + 40*n]);
for k = 1:n
    in = sprintf('u%d', k);
    add_block('built-in/Inport', [name '/' in], ...
        'Position', [100 30 + 40*k 130 44 + 40*k], ...
        'PortDimensions', num2str(input_widths(k)), 'SampleTime', T_s, ...
        'Interpolate', 'off');
    add_line(name, [in '/1'], sprintf('Control system/%d', k));
end
add_block('built-in/Outport', [name '/d_abc'], 'Position', [520 50 550 64]);
add_block('built-in/Outport', [name '/signals'], 'Position', [520 100 550 114]);
add_line(name, 'Control system/1', 'd_abc/1');
add_line(name, 'Control system/2', 'signals/1');
save_system(name, slx);
end
