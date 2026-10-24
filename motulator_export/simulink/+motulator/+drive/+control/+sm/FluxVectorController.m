classdef FluxVectorController < motulator.internal.Object
    %FLUXVECTORCONTROLLER Flux-vector controller of a synchronous machine.
    %   VECTOR_CTRL = FLUXVECTORCONTROLLER(PAR, CFG) as in motulator, with the
    %   machine model PAR (SynchronousMachinePars, or
    %   SaturatedSynchronousMachinePars with a GradNet flux map) and the
    %   configuration CFG (FluxVectorControllerCfg).

    properties
        par  % Machine model parameters
        cfg  % Configuration (FluxVectorControllerCfg)
    end

    methods
        function obj = FluxVectorController(varargin)
            obj.set_args(varargin, {'par', 'cfg'}, {'par', 'cfg'});
            motulator.internal.check_class(obj.par, ...
                {'motulator.drive.model.SynchronousMachinePars', ...
                'motulator.drive.model.SaturatedSynchronousMachinePars'});
        end
    end
end
