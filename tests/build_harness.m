function build_harness(s)
%BUILD_HARNESS Build a model running a control system alone, for the tests.
%   BUILD_HARNESS(S) builds the model S.name in the folder S.folder, with the
%   subsystem of the control system S.control (see blocks.add_control_system), the
%   root-level inputs of its inputs (of the widths S.input_widths, given by the
%   external input of sim), and the outputs 'd_abc' and 'signals'. The parameters
%   are defined by the script S.init_script in the folder (see blocks.new_model).
%   The solver is discrete with the fixed step S.T_s, so that the outputs are those
%   of the sampling instants.
slx = blocks.new_model(s);
sys = s.name;
set_param(sys, 'Solver', 'FixedStepDiscrete', 'FixedStep', blocks.num(s.T_s));
s.control.folder = s.folder;
n = numel(s.input_widths);
blocks.add_control_system([sys '/Control system'], s.control, ...
    [300 40 440 40 + 40*n]);
for k = 1:n
    name = sprintf('u%d', k);
    blocks.add([sys '/' name], 'built-in/Inport', [100 30 + 40*k 130 44 + 40*k], ...
        'PortDimensions', num2str(s.input_widths(k)), ...
        'SampleTime', blocks.num(s.T_s), 'Interpolate', 'off');
    add_line(sys, [name '/1'], sprintf('Control system/%d', k));
end
blocks.add([sys '/d_abc'], 'built-in/Outport', [520 50 550 64]);
blocks.add([sys '/signals'], 'built-in/Outport', [520 100 550 114]);
add_line(sys, 'Control system/1', 'd_abc/1');
add_line(sys, 'Control system/2', 'signals/1');
save_system(sys, slx);
end
