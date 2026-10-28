classdef GridConverterSystem < motulator.internal.Object
    %GRIDCONVERTERSYSTEM Continuous-time model of a grid converter system.
    %   MDL = GRIDCONVERTERSYSTEM(CONVERTER, AC_FILTER, AC_SOURCE) as in motulator.

    properties
        converter  % Converter model (VoltageSourceConverter)
        ac_filter  % AC filter (LCLFilter or LFilter)
        ac_source  % Grid voltage source (ThreePhaseSource)
    end

    methods
        function obj = GridConverterSystem(varargin)
            args = {'converter', 'ac_filter', 'ac_source'};
            obj.set_args(varargin, args, args);
        end
    end
end
