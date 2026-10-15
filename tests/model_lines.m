function rows = model_lines(sys)
%MODEL_LINES Connections of the lines below a subsystem, for the tests.
%   ROWS = MODEL_LINES(SYS) gives the lines of the subsystem SYS and of the
%   subsystems in it (also under the masks) as the rows 'system|source|destination'
%   of a cell column: the system is the path of the subsystem containing the line
%   relative to SYS ('' for SYS itself), and the source and the destination are
%   'block/port'. A branched line has a row for each destination. The lines to the
%   Terminator blocks of the unconnected outputs are left out.
lines = find_system(sys, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
    'FindAll', 'on', 'Type', 'line');
rows = cell(0, 1);
for k = 1:numel(lines)
    h = lines(k);
    if get_param(h, 'LineParent') ~= -1
        continue % A branch, whose destinations are those of its root line
    end
    parent = get_param(h, 'Parent');
    if ~strcmp(get_param(parent, 'SFBlockType'), 'NONE')
        continue % Inside a MATLAB Function block (its implementation)
    end
    system = regexprep(parent(numel(sys) + 1:end), '^/', '');
    src = get_param(h, 'SrcPortHandle');
    dsts = get_param(h, 'DstPortHandle');
    for d = reshape(dsts, 1, [])
        if strcmp(get_param(get_param(d, 'Parent'), 'BlockType'), 'Terminator')
            continue
        end
        rows{end + 1, 1} = strjoin({system, port(src), port(d)}, '|'); %#ok<AGROW>
    end
end
end

function s = port(h)
% Port as 'block/number'
s = sprintf('%s/%d', get_param(get_param(h, 'Parent'), 'Name'), ...
    get_param(h, 'PortNumber'));
end
