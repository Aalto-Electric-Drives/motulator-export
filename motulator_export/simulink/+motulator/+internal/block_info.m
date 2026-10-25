function info = block_info(code, name)
%BLOCK_INFO Data of a block of a control system of the library.
%   INFO = BLOCK_INFO(CODE, NAME) gives the data of the block NAME of the library
%   of the code CODE (written by motulator_export, see control_blocks_c.m), e.g.,
%   its inputs, the names of its monitored signals (signals), and their groups
%   (outputs, rows {name, signals}).
blocks = feval(['control_blocks_' code]);
for k = 1:numel(blocks)
    if strcmp(blocks{k}.name, name)
        info = blocks{k};
        return
    end
end
error('motulator:library', 'No block %s in the library.', name);
end
