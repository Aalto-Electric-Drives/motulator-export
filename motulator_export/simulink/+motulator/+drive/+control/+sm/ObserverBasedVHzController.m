classdef ObserverBasedVHzController < motulator.internal.Object
    %OBSERVERBASEDVHZCONTROLLER Observer-based V/Hz controller of a synchronous
    %   machine.
    %   VHZ_CTRL = OBSERVERBASEDVHZCONTROLLER(PAR, CFG) as in motulator, with the
    %   machine model PAR (SynchronousMachinePars, or
    %   SaturatedSynchronousMachinePars with a GradNet flux map) and the
    %   configuration CFG (ObserverBasedVHzControllerCfg).

    properties
        par  % Machine model parameters
        cfg  % Configuration (ObserverBasedVHzControllerCfg)
    end

    methods
        function obj = ObserverBasedVHzController(varargin)
            obj.set_args(varargin, {'par', 'cfg'}, {'par', 'cfg'});
            motulator.internal.check_class(obj.par, ...
                {'motulator.drive.model.SynchronousMachinePars', ...
                'motulator.drive.model.SaturatedSynchronousMachinePars'});
        end
    end
end
