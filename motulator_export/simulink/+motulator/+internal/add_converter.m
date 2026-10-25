function add_converter(blk, pos)
%ADD_CONVERTER Block of VoltageSourceConverter with a stiff DC bus.
%   The ideal two-level converter, u_c_ab = u_dc*q_ab, where q_ab is the space
%   vector of the switching states q_abc (no zero sequence, as in motulator). The
%   mask has the DC-bus voltage u_dc.
import motulator.internal.*
add(blk, 'built-in/Subsystem', pos);
add([blk '/Product'], 'built-in/Product', [230 50 260 130], 'Inputs', '**');
add([blk '/q_abc'], 'built-in/Inport', [40 0 70 14]);
add([blk '/abc2ab'], 'built-in/Gain', [110 0 180 30], ...
    'Gain', '[2/3 -1/3 -1/3; 0 1/sqrt(3) -1/sqrt(3)]', 'Multiplication', 'Matrix(K*u)');
add([blk '/u_dc'], 'built-in/Constant', [110 0 180 20], 'Value', 'u_dc');
add([blk '/u_c_ab'], 'built-in/Outport', [300 0 330 14]);
align(blk, 'abc2ab', 'Outport', 1, port_y(blk, 'Product', 'Inport', 1));
align(blk, 'q_abc', 'Outport', 1, port_y(blk, 'Product', 'Inport', 1));
align(blk, 'u_dc', 'Outport', 1, port_y(blk, 'Product', 'Inport', 2));
align(blk, 'u_c_ab', 'Inport', 1, port_y(blk, 'Product', 'Outport', 1));
connect(blk, 'q_abc/1', 'abc2ab/1');
connect(blk, 'abc2ab/1', 'Product/1');
connect(blk, 'u_dc/1', 'Product/2');
connect(blk, 'Product/1', 'u_c_ab/1');
add_mask(blk, 'VoltageSourceConverter (motulator)', ...
    ['Ideal two-level converter with a stiff DC bus (VoltageSourceConverter): the ' ...
    'converter voltage u_c_ab (V) in stationary coordinates from the switching ' ...
    'states q_abc, without the dead time and the threshold voltage.'], ...
    {'u_dc', 'u_dc: DC-bus voltage (V)', 'converter.u_dc'}, '', ...
    icon(blk, 'VoltageSourceConverter'));
end
