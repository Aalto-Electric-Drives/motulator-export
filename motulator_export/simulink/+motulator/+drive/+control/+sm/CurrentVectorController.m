classdef CurrentVectorController < motulator.internal.Object
    %CURRENTVECTORCONTROLLER Current-vector controller of a synchronous machine.
    %   VECTOR_CTRL = CURRENTVECTORCONTROLLER(PAR, CFG) as in motulator, with the
    %   machine model PAR (SynchronousMachinePars, or
    %   SaturatedSynchronousMachinePars with a GradNet flux map) and the
    %   configuration CFG (CurrentVectorControllerCfg).
    %   VECTOR_CTRL.OBSERVER.W_M sets the initial speed estimate of the observer
    %   (rad/s), as in motulator.

    properties
        par  % Machine model parameters
        cfg  % Configuration (CurrentVectorControllerCfg)
        observer = struct('w_M', 0)  % Initial state of the observer
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
