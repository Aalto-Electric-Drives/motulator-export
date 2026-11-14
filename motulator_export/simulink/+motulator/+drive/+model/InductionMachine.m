classdef InductionMachine < motulator.internal.Object
    %INDUCTIONMACHINE Induction machine model.
    %   MACHINE = INDUCTIONMACHINE(PAR) as in motulator, with constant parameters
    %   (InductionMachineInvGammaPars or InductionMachinePars). The block models the
    %   machine with the inverse-Gamma parameters, which both classes provide, in
    %   stator coordinates with the stator and rotor flux linkages as the states.

    properties
        par  % Machine model parameters
    end

    methods
        function obj = InductionMachine(varargin)
            obj.set_args(varargin, {'par'}, {'par'});
        end

        function name = block(~)
            %BLOCK Block of the library.
            name = 'InductionMachine';
        end

        function s = params(obj)
            %PARAMS Arguments as a struct, with the inverse-Gamma parameters.
            par = obj.par;
            s = struct('par', struct('n_p', par.n_p, 'R_s', par.R_s, ...
                'R_R', par.R_R, 'L_sgm', par.L_sgm, 'L_M', par.L_M, 'G_c', par.G_c));
        end
    end
end
