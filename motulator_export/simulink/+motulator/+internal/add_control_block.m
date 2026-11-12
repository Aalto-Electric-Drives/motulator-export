function add_control_block(blk, c, lib, info, pos)
%ADD_CONTROL_BLOCK Block of the control system of a model.
%   ADD_CONTROL_BLOCK(BLK, C, LIB, INFO, POS) adds the block BLK of the control
%   system C (see control_system) at the position POS: a link to its block in the
%   library LIB, or, with LIB empty, the control system itself, built from its data
%   INFO (see block_info) with MATLAB Function blocks, which contain the code of
%   their functions, so that the model does not need the library. With the block C.block
%   of another library given (see Simulation.build), it is a link to that block.
import motulator.internal.*
if isfield(c, 'block')
    add_block(c.block, blk, 'Position', pos);
elseif isempty(lib)
    info.folder = library_folder();
    add_control_system(blk, info, pos);
else
    add_block([lib '/' c.name], blk, 'Position', pos);
end
end
