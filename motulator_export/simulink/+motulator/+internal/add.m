function add(blk, type, pos, varargin)
%ADD Add a block at the position [left top right bottom] with the parameters.
add_block(type, blk, 'Position', pos, varargin{:});
end
