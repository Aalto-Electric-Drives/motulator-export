function connect(sys, src, dst)
%CONNECT Straight line between the aligned ports src and dst ('block/port').
add_line(sys, src, dst);
end
