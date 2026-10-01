function add_converter(blk, pos)
%ADD_CONVERTER Ideal two-level converter with a stiff DC bus.
%   u_s_ab = u_dc*q_ab, where q_ab is the space vector of the switching states q_abc
%   (no zero sequence, as in motulator).
blocks.add(blk, 'built-in/Subsystem', pos);
blocks.add([blk '/Product'], 'built-in/Product', [230 50 260 130], 'Inputs', '**');
blocks.add([blk '/q_abc'], 'built-in/Inport', [40 0 70 14]);
blocks.add([blk '/abc2ab'], 'built-in/Gain', [110 0 180 30], ...
    'Gain', '[2/3 -1/3 -1/3; 0 1/sqrt(3) -1/sqrt(3)]', ...
    'Multiplication', 'Matrix(K*u)');
blocks.add([blk '/u_dc'], 'built-in/Constant', [110 0 180 20], ...
    'Value', 'converter.u_dc');
blocks.add([blk '/u_s_ab'], 'built-in/Outport', [300 0 330 14]);
blocks.align(blk, 'abc2ab', 'Outport', 1, blocks.port_y(blk, 'Product', 'Inport', 1));
blocks.align(blk, 'q_abc', 'Outport', 1, blocks.port_y(blk, 'Product', 'Inport', 1));
blocks.align(blk, 'u_dc', 'Outport', 1, blocks.port_y(blk, 'Product', 'Inport', 2));
blocks.align(blk, 'u_s_ab', 'Inport', 1, blocks.port_y(blk, 'Product', 'Outport', 1));
blocks.connect(blk, 'q_abc/1', 'abc2ab/1');
blocks.connect(blk, 'abc2ab/1', 'Product/1');
blocks.connect(blk, 'u_dc/1', 'Product/2');
blocks.connect(blk, 'Product/1', 'u_s_ab/1');
end
