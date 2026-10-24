function add_carrier_comparison(blk, pos)
%ADD_CARRIER_COMPARISON Block of CarrierComparison of motulator.
%   The symmetrical triangular carrier with the period 2*T_s, starting from its
%   maximum, is compared with the duty ratios, which the control system updates
%   at its extrema. This corresponds to CarrierComparison of motulator without
%   the counter quantization. The switching instants are located by the
%   zero-crossing detection of the Relational Operator block. The mask has the
%   sampling period T_s of the control system.
import motulator.internal.*
add(blk, 'built-in/Subsystem', pos);
add([blk '/Compare'], 'built-in/RelationalOperator', [130 40 160 120], ...
    'Operator', '>', 'ZeroCross', 'on');
add([blk '/d_abc'], 'built-in/Inport', [40 0 70 14]);
add([blk '/Carrier'], 'simulink/Sources/Repeating Sequence', [40 0 80 30], ...
    'rep_seq_t', '[0 T_s 2*T_s]', 'rep_seq_y', '[1 0 1]');
add([blk '/To double'], 'built-in/DataTypeConversion', [200 0 250 30], ...
    'OutDataTypeStr', 'double');
add([blk '/q_abc'], 'built-in/Outport', [290 0 320 14]);
align(blk, 'd_abc', 'Outport', 1, port_y(blk, 'Compare', 'Inport', 1));
align(blk, 'Carrier', 'Outport', 1, port_y(blk, 'Compare', 'Inport', 2));
y = port_y(blk, 'Compare', 'Outport', 1);
align(blk, 'To double', 'Inport', 1, y);
align(blk, 'q_abc', 'Inport', 1, y);
connect(blk, 'd_abc/1', 'Compare/1');
connect(blk, 'Carrier/1', 'Compare/2');
connect(blk, 'Compare/1', 'To double/1');
connect(blk, 'To double/1', 'q_abc/1');
add_mask(blk, 'CarrierComparison (motulator)', ...
    ['Carrier comparison (CarrierComparison, without the counter quantization): ' ...
    'the switching states q_abc from the duty ratios d_abc and the symmetrical ' ...
    'triangular carrier of the period 2*T_s, starting from its maximum.'], ...
    {'T_s', 'T_s: Sampling period (s) of the control system', ...
    'vector_ctrl.cfg.T_s'}, '', icon(blk, 'CarrierComparison'));
end
