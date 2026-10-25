function block = control_block(c, lib)
%CONTROL_BLOCK Source block of the control system of a model.
%   BLOCK = CONTROL_BLOCK(C, LIB) gives the block of the control system C (see
%   control_system) in the library LIB, or the block C.block of another library if
%   given (see Simulation.build).
if isfield(c, 'block')
    block = c.block;
else
    block = [lib '/' c.name];
end
end
