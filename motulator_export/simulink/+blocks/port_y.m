function y = port_y(sys, blk, kind, n)
%PORT_Y Vertical position of the port n of the kind 'Inport' or 'Outport'.
p = blocks.port_pos(sys, sprintf('%s/%d', blk, n), kind);
y = p(2);
end
