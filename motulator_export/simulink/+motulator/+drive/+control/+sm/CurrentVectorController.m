classdef CurrentVectorController < motulator.internal.Object
    %CURRENTVECTORCONTROLLER Current-vector controller of a synchronous machine.
    %   VECTOR_CTRL = CURRENTVECTORCONTROLLER(PAR, CFG) as in motulator, with the
    %   machine model PAR (SynchronousMachinePars, or
    %   SaturatedSynchronousMachinePars with a GradNet flux map) and the
    %   configuration CFG (CurrentVectorControllerCfg).

    properties
        par  % Machine model parameters
        cfg  % Configuration (CurrentVectorControllerCfg)
    end

    methods
        function obj = CurrentVectorController(varargin)
            obj.set_args(varargin, {'par', 'cfg'}, {'par', 'cfg'});
            motulator.internal.check_class(obj.par, ...
                {'motulator.drive.model.SynchronousMachinePars', ...
                'motulator.drive.model.SaturatedSynchronousMachinePars'});
        end
    end
end
