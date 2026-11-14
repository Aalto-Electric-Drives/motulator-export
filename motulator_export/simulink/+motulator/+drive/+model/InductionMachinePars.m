classdef InductionMachinePars < motulator.internal.Object
    %INDUCTIONMACHINEPARS Gamma-model parameters of an induction machine.
    %   PAR = INDUCTIONMACHINEPARS(n_p=..., R_s=..., R_r=..., L_ell=..., L_s=...) as
    %   in motulator, with constant parameters. PAR also provides the corresponding
    %   inverse-Gamma-model parameters (gamma, L_M, L_sgm, and R_R). The core losses
    %   are not modeled (G_c = 0).

    properties
        n_p       % Number of pole pairs
        R_s       % Stator resistance (Ohm)
        R_r       % Rotor resistance (Ohm)
        L_ell     % Leakage inductance (H)
        L_s       % Stator inductance (H)
        G_c = 0   % Core-loss conductance (S), only 0
    end

    properties (Dependent)
        gamma     % Magnetic coupling factor
        L_M       % Inverse-Gamma magnetizing inductance (H)
        L_sgm     % Inverse-Gamma leakage inductance (H)
        R_R       % Inverse-Gamma rotor resistance (Ohm)
    end

    methods
        function obj = InductionMachinePars(varargin)
            args = {'n_p', 'R_s', 'R_r', 'L_ell', 'L_s'};
            obj.set_args(varargin, [args, {'G_c'}], args);
            motulator.internal.check_zero(obj, 'G_c', 'Core losses');
        end

        function value = get.gamma(obj)
            value = obj.L_s/(obj.L_s + obj.L_ell);
        end

        function value = get.L_M(obj)
            value = obj.gamma*obj.L_s;
        end

        function value = get.L_sgm(obj)
            value = obj.gamma*obj.L_ell;
        end

        function value = get.R_R(obj)
            value = obj.gamma^2*obj.R_r;
        end
    end
end
