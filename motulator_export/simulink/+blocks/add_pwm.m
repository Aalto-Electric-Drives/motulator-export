function add_pwm(blk, T_s, pos)
%ADD_PWM Carrier comparison (CarrierComparison of motulator).
%   The symmetrical triangular carrier with the period 2*T_s, starting from its
%   maximum, is compared with the duty ratios, which are updated at its extrema.
%   This corresponds to CarrierComparison in motulator (without the counter
%   quantization). The switching instants are located by the zero-crossing
%   detection of the Relational Operator block.
blocks.add(blk, 'built-in/Subsystem', pos);
blocks.add([blk '/Compare'], 'built-in/RelationalOperator', [130 40 160 120], ...
    'Operator', '>', 'ZeroCross', 'on');
blocks.add([blk '/d_abc'], 'built-in/Inport', [40 0 70 14]);
blocks.add([blk '/Carrier'], 'simulink/Sources/Repeating Sequence', ...
    [40 0 80 30], ...
    'rep_seq_t', sprintf('[0 %s %s]', blocks.num(T_s), blocks.num(2*T_s)), ...
    'rep_seq_y', '[1 0 1]');
blocks.add([blk '/To double'], 'built-in/DataTypeConversion', [200 0 250 30], ...
    'OutDataTypeStr', 'double');
blocks.add([blk '/q_abc'], 'built-in/Outport', [290 0 320 14]);
blocks.align(blk, 'd_abc', 'Outport', 1, blocks.port_y(blk, 'Compare', 'Inport', 1));
blocks.align(blk, 'Carrier', 'Outport', 1, ...
    blocks.port_y(blk, 'Compare', 'Inport', 2));
y = blocks.port_y(blk, 'Compare', 'Outport', 1);
blocks.align(blk, 'To double', 'Inport', 1, y);
blocks.align(blk, 'q_abc', 'Inport', 1, y);
blocks.connect(blk, 'd_abc/1', 'Compare/1');
blocks.connect(blk, 'Carrier/1', 'Compare/2');
blocks.connect(blk, 'Compare/1', 'To double/1');
blocks.connect(blk, 'To double/1', 'q_abc/1');
end
