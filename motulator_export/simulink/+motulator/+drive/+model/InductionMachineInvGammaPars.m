classdef InductionMachineInvGammaPars < motulator.internal.Object
    %INDUCTIONMACHINEINVGAMMAPARS Inverse-Gamma-model parameters of an induction
    %   machine.
    %   PAR = INDUCTIONMACHINEINVGAMMAPARS(n_p=..., R_s=..., R_R=..., L_sgm=...,
    %   L_M=...) as in motulator, with constant parameters, and
    %   INDUCTIONMACHINEINVGAMMAPARS.FROM_GAMMA_PARS(PAR) from the Gamma-model
    %   parameters PAR (InductionMachinePars). The core losses are not modeled
    %   (G_c = 0).

    properties
        n_p       % Number of pole pairs
        R_s       % Stator resistance (Ohm)
        R_R       % Rotor resistance (Ohm)
        L_sgm     % Leakage inductance (H)
        L_M       % Magnetizing inductance (H)
        G_c = 0   % Core-loss conductance (S), only 0
    end

    methods
        function obj = InductionMachineInvGammaPars(varargin)
            args = {'n_p', 'R_s', 'R_R', 'L_sgm', 'L_M'};
            obj.set_args(varargin, [args, {'G_c'}], args);
            motulator.internal.check_zero(obj, 'G_c', 'Core losses');
        end
    end

    methods (Static)
        function par = from_gamma_pars(gamma)
            %FROM_GAMMA_PARS Inverse-Gamma-model parameters from the Gamma model.
            g = gamma.L_s/(gamma.L_s + gamma.L_ell);
            par = motulator.drive.model.InductionMachineInvGammaPars( ...
                'n_p', gamma.n_p, 'R_s', gamma.R_s, 'R_R', g^2*gamma.R_r, ...
                'L_sgm', g*gamma.L_ell, 'L_M', g*gamma.L_s, 'G_c', gamma.G_c);
        end
    end
end
