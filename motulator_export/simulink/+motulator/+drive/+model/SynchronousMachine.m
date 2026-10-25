classdef SynchronousMachine < motulator.internal.Object
    %SYNCHRONOUSMACHINE Synchronous machine model.
    %   MACHINE = SYNCHRONOUSMACHINE(PAR) as in motulator, with constant inductances
    %   (SynchronousMachinePars) or a GradNet current map with spatial harmonics
    %   (SpatialSaturatedSynchronousMachinePars). The stator flux linkage in rotor
    %   coordinates is the state, starting from the PM-flux linkage.

    properties
        par  % Machine model parameters
    end

    methods
        function obj = SynchronousMachine(varargin)
            obj.set_args(varargin, {'par'}, {'par'});
        end

        function name = block(obj)
            %BLOCK Block of the library.
            gradnet = 'motulator.drive.model.SpatialSaturatedSynchronousMachinePars';
            if isa(obj.par, gradnet)
                name = 'SynchronousMachine (GradNet)';
            elseif isa(obj.par, 'motulator.drive.model.SynchronousMachinePars')
                name = 'SynchronousMachine';
            else
                error('motulator:unsupported', ['Only SynchronousMachinePars and ' ...
                    'SpatialSaturatedSynchronousMachinePars supported.']);
            end
        end
    end
end
