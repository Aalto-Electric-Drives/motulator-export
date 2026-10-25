function add_step(blk, steps, pos)
%ADD_STEP Step signal: a Step block, or a sum of Step blocks for several steps.
%   ADD_STEP(BLK, STEPS, POS) adds the step signal with the rows [time before after]
%   of STEPS (as StepSignal of motulator_export): a Step block for one row, or a
%   subsystem summing a Step block per row.
import motulator.internal.*
if size(steps, 1) == 1
    add_single(blk, steps, pos);
    return
end
add(blk, 'built-in/Subsystem', pos);
n = size(steps, 1);
add([blk '/Sum'], 'built-in/Sum', [120 20 140 20 + 50*n], ...
    'Inputs', repmat('+', 1, n), 'IconShape', 'rectangular');
for k = 1:n
    name = sprintf('Step %d', k);
    add_single([blk '/' name], steps(k, :), [40 0 70 30]);
    align(blk, name, 'Outport', 1, port_y(blk, 'Sum', 'Inport', k));
    connect(blk, [name '/1'], sprintf('Sum/%d', k));
end
add([blk '/y'], 'built-in/Outport', [190 0 220 14]);
align(blk, 'y', 'Inport', 1, port_y(blk, 'Sum', 'Outport', 1));
connect(blk, 'Sum/1', 'y/1');
end

function add_single(blk, step, pos)
import motulator.internal.*
add(blk, 'built-in/Step', pos, 'Time', num(step(1)), ...
    'Before', num(step(2)), 'After', num(step(3)), 'SampleTime', '0');
end
