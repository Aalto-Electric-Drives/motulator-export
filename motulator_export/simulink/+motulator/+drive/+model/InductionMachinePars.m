classdef InductionMachinePars < motulator.internal.Object
    %INDUCTIONMACHINEPARS Gamma-model parameters of an induction machine.
    %   PAR = INDUCTIONMACHINEPARS(n_p=..., R_s=..., R_r=..., L_ell=..., L_s=...) as
    %   in motulator, with constant parameters. PAR.INV_GAMMA_PARS(PSI_S) gives the
    %   corresponding inverse-Gamma-model parameters (InductionMachineInvGammaPars),
    %   the same at any stator flux linkage magnitude PSI_S. The core losses are not
    %   modeled (G_c = 0).

    properties
        n_p       % Number of pole pairs
        R_s       % Stator resistance (Ohm)
        R_r       % Rotor resistance (Ohm)
        L_ell     % Leakage inductance (H)
        L_s       % Stator inductance (H)
        G_c = 0   % Core-loss conductance (S), only 0
    end

    methods
        function obj = InductionMachinePars(varargin)
            args = {'n_p', 'R_s', 'R_r', 'L_ell', 'L_s'};
            obj.set_args(varargin, [args, {'G_c'}], args);
            motulator.internal.check_zero(obj, 'G_c', 'Core losses');
        end

        function inv = inv_gamma_pars(obj, ~)
            %INV_GAMMA_PARS Inverse-Gamma-model parameters.
            gamma = obj.L_s/(obj.L_s + obj.L_ell);  % Magnetic coupling factor
            inv = motulator.drive.model.InductionMachineInvGammaPars( ...
                n_p=obj.n_p, R_s=obj.R_s, R_R=gamma^2*obj.R_r, ...
                L_sgm=gamma*obj.L_ell, L_M=gamma*obj.L_s);
        end
    end
end
