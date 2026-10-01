function align(sys, blk, kind, n, y)
%ALIGN Move a block vertically so that its port n ('Inport' or 'Outport') is at y.
pos = get_param([sys '/' blk], 'Position');
dy = y - blocks.port_y(sys, blk, kind, n);
set_param([sys '/' blk], 'Position', pos + [0 dy 0 dy]);
end
