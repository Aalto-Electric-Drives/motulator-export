classdef InductionMachine < motulator.internal.Object
    %INDUCTIONMACHINE Induction machine model.
    %   MACHINE = INDUCTIONMACHINE(PAR) as in motulator, with constant parameters
    %   (InductionMachinePars or InductionMachineInvGammaPars). The machine is
    %   modeled with the inverse-Gamma model, which is equivalent to the Gamma
    %   model, with the stator and rotor flux linkages as the states.

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
            if isa(par, 'motulator.drive.model.InductionMachinePars')
                par = motulator.drive.model.InductionMachineInvGammaPars.from_gamma_pars(par);
            end
            s = struct('par', par.params());
        end
    end
end
