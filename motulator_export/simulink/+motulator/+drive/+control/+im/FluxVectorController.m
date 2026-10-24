classdef FluxVectorController < motulator.internal.Object
    %FLUXVECTORCONTROLLER Flux-vector controller of an induction machine.
    %   VECTOR_CTRL = FLUXVECTORCONTROLLER(PAR, CFG) as in motulator, with the
    %   machine model PAR (InductionMachineInvGammaPars) and the configuration CFG
    %   (FluxVectorControllerCfg).

    properties
        par  % Machine model parameters (InductionMachineInvGammaPars)
        cfg  % Configuration (FluxVectorControllerCfg)
    end

    methods
        function obj = FluxVectorController(varargin)
            obj.set_args(varargin, {'par', 'cfg'}, {'par', 'cfg'});
            motulator.internal.check_class(obj.par, ...
                'motulator.drive.model.InductionMachineInvGammaPars');
        end
    end
end
