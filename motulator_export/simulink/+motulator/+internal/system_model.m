function [variables, builder, signals] = system_model(mdl)
%SYSTEM_MODEL Parameters, builder, and signals of a system model.
%   [VARIABLES, BUILDER, SIGNALS] = SYSTEM_MODEL(MDL) gives the parameters
%   VARIABLES of the system model MDL (its parts, named as in motulator, to which
%   the masks of their blocks refer), the function BUILDER that adds its blocks to
%   a model (build_drive or build_grid), and the names of the signals of the
%   output port mdl of the model.
if isa(mdl, 'motulator.drive.model.Drive')
    variables = struct('machine', mdl.machine.params(), ...
        'mechanics', mdl.mechanics.params(), 'converter', mdl.converter.params());
    builder = @(varargin) motulator.internal.build_drive(varargin{:});
    signals = {'i_a', 'i_b', 'i_c', 'w_M', 'theta_M', 'tau_M'};
elseif isa(mdl, 'motulator.grid.model.GridConverterSystem')
    variables = struct('converter', mdl.converter.params(), ...
        'ac_filter', mdl.ac_filter.params(), 'ac_source', mdl.ac_source.params());
    builder = @(varargin) motulator.internal.build_grid(varargin{:});
    signals = {'i_c_a', 'i_c_b', 'i_c_c'};
    if isa(mdl.ac_filter, 'motulator.grid.model.LCLFilter')
        signals = [signals, {'i_g_a', 'i_g_b', 'i_g_c'}];
    end
else
    error('motulator:unsupported', 'Only Drive and GridConverterSystem supported.');
end
end
