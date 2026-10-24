function add_source(blk, src, pos, T_s)
%ADD_SOURCE Source of a reference or of the load torque.
%   ADD_SOURCE(BLK, SRC, POS) adds the Constant block of a constant SRC, or the
%   Step blocks of a StepSignal SRC at the position POS (see add_step).
%   ADD_SOURCE(BLK, SRC, POS, T_S) adds a reference of the control system, whose
%   steps switch at the same sampling instants as in motulator (StepSignal.sampled
%   with the sampling period T_S).
import motulator.internal.*
if isnumeric(src)
    add(blk, 'built-in/Constant', pos + [-10 0 10 -10], 'Value', num(src));
else
    if nargin > 3
        src = src.sampled(T_s);
    end
    add_step(blk, src.steps(), pos);
end
end
