function route(sys, src, dst, varargin)
%ROUTE Line through given corners.
%   ROUTE(SYS, SRC, DST, 'x', X1, 'y', Y1, ...) draws a line from the output port
%   SRC to the input port DST ('block/port') through the corners given by the pairs
%   'x', value (horizontal segment to x) and 'y', value (vertical segment to y).
%   The line enters DST horizontally. A line from a port that already has a line is
%   a branch of it.
p = blocks.port_pos(sys, src, 'Outport');
q = blocks.port_pos(sys, dst, 'Inport');
points = p;
for k = 1:2:numel(varargin)
    if varargin{k} == 'x'
        p = [varargin{k + 1} p(2)];
    else
        p = [p(1) varargin{k + 1}];
    end
    points(end + 1, :) = p; %#ok<AGROW>
end
if p(2) ~= q(2)
    points(end + 1, :) = [p(1) q(2)];
end
add_line(sys, [points; q]);
end
