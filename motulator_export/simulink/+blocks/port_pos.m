function p = port_pos(sys, port, kind)
%PORT_POS Position of the port 'block/port' of the kind 'Inport' or 'Outport'.
k = find(port == '/', 1, 'last');
handles = get_param([sys '/' port(1:k - 1)], 'PortHandles');
p = get_param(handles.(kind)(str2double(port(k + 1:end))), 'Position');
end
