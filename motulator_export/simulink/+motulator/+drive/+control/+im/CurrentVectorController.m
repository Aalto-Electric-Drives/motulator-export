classdef CurrentVectorController < motulator.internal.Object
    %CURRENTVECTORCONTROLLER Current-vector controller of an induction machine.
    %   VECTOR_CTRL = CURRENTVECTORCONTROLLER(PAR, CFG) as in motulator, with the
    %   machine model PAR (InductionMachineInvGammaPars) and the configuration CFG
    %   (CurrentVectorControllerCfg).

    properties
        par  % Machine model parameters (InductionMachineInvGammaPars)
        cfg  % Configuration (CurrentVectorControllerCfg)
    end

    methods
        function obj = CurrentVectorController(varargin)
            obj.set_args(varargin, {'par', 'cfg'}, {'par', 'cfg'});
            motulator.internal.check_class(obj.par, ...
                'motulator.drive.model.InductionMachineInvGammaPars');
        end
    end
end
