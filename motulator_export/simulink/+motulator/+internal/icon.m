function display = icon(blk, name)
%ICON Drawing commands of the icon of a masked subsystem.
%   DISPLAY = ICON(BLK, NAME) shows the name NAME (the class) and the labels of
%   the ports, the names of the Inport and Outport blocks of the subsystem BLK.
lines = {sprintf('disp(''%s'');', name)};
kinds = {'Inport', 'input'; 'Outport', 'output'};
for j = 1:2
    ports = find_system(blk, 'SearchDepth', 1, 'BlockType', kinds{j, 1});
    for k = 1:numel(ports)
        lines{end + 1} = sprintf('port_label(''%s'', %s, ''%s'');', kinds{j, 2}, ...
            get_param(ports{k}, 'Port'), get_param(ports{k}, 'Name')); %#ok<AGROW>
    end
end
display = strjoin(lines, newline);
end
