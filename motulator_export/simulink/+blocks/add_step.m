function add_step(blk, steps, pos)
%ADD_STEP Step signal: a Step block, or a sum of Step blocks for several steps.
%   ADD_STEP(BLK, STEPS, POS) adds the step signal with the rows [time before after]
%   of STEPS (as StepSignal of motulator_export): a Step block for one row, or a
%   subsystem summing a Step block per row.
if size(steps, 1) == 1
    add_single(blk, steps, pos);
    return
end
blocks.add(blk, 'built-in/Subsystem', pos);
n = size(steps, 1);
blocks.add([blk '/Sum'], 'built-in/Sum', [120 20 140 20 + 50*n], ...
    'Inputs', repmat('+', 1, n), 'IconShape', 'rectangular');
for k = 1:n
    name = sprintf('Step %d', k);
    add_single([blk '/' name], steps(k, :), [40 0 70 30]);
    blocks.align(blk, name, 'Outport', 1, blocks.port_y(blk, 'Sum', 'Inport', k));
    blocks.connect(blk, [name '/1'], sprintf('Sum/%d', k));
end
blocks.add([blk '/y'], 'built-in/Outport', [190 0 220 14]);
blocks.align(blk, 'y', 'Inport', 1, blocks.port_y(blk, 'Sum', 'Outport', 1));
blocks.connect(blk, 'Sum/1', 'y/1');
end

function add_single(blk, step, pos)
blocks.add(blk, 'built-in/Step', pos, 'Time', blocks.num(step(1)), ...
    'Before', blocks.num(step(2)), 'After', blocks.num(step(3)), 'SampleTime', '0');
end
