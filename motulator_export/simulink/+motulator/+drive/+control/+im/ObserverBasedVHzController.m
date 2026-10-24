classdef ObserverBasedVHzController < motulator.internal.Object
    %OBSERVERBASEDVHZCONTROLLER Observer-based V/Hz controller of an induction
    %   machine.
    %   VHZ_CTRL = OBSERVERBASEDVHZCONTROLLER(PAR, CFG) as in motulator, with the
    %   machine model PAR (InductionMachineInvGammaPars) and the configuration CFG
    %   (ObserverBasedVHzControllerCfg). With L_M = inf (and R_s = R_R = L_sgm =
    %   0), the controller is pure open-loop V/Hz control.

    properties
        par  % Machine model parameters (InductionMachineInvGammaPars)
        cfg  % Configuration (ObserverBasedVHzControllerCfg)
    end

    methods
        function obj = ObserverBasedVHzController(varargin)
            obj.set_args(varargin, {'par', 'cfg'}, {'par', 'cfg'});
            motulator.internal.check_class(obj.par, ...
                'motulator.drive.model.InductionMachineInvGammaPars');
        end
    end
end
