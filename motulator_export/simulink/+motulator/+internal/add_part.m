function add_part(lib, name, blk, pos)
%ADD_PART Block of a part of a system model.
%   ADD_PART(LIB, NAME, BLK, POS) adds the block BLK of the part NAME (e.g.,
%   'SynchronousMachine', the name of its block in the library of motulator) at
%   the position POS: a link to the block of the library LIB, or, with LIB empty,
%   the block itself, built as that of the library (the GradNet machine with a
%   MATLAB Function block calling its function), so that the model does not need
%   the library.
import motulator.internal.*
if ~isempty(lib)
    add_block([lib '/' name], blk, 'Position', pos);
    return
end
switch name
    case 'SynchronousMachine'
        add_synchronous_machine(blk, pos);
    case 'SynchronousMachine (GradNet)'
        add_gradnet_machine(blk, pos, 'matlab', library_folder());
    case 'InductionMachine'
        add_induction_machine(blk, pos);
    case 'MechanicalSystem'
        add_mechanics(blk, pos);
    case 'ExternalRotorSpeed'
        add_external_rotor_speed(blk, pos);
    case 'CarrierComparison'
        add_carrier_comparison(blk, pos);
    case 'VoltageSourceConverter'
        add_converter(blk, pos);
    case 'LCLFilter'
        add_lcl_filter(blk, pos);
    case 'LFilter'
        add_l_filter(blk, pos);
    case 'ThreePhaseSource'
        add_three_phase_source(blk, pos);
    otherwise
        error('motulator:library', 'No part %s.', name);
end
end
