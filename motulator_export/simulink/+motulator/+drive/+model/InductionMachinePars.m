classdef InductionMachinePars < motulator.internal.Object
    %INDUCTIONMACHINEPARS Gamma-model parameters of an induction machine.
    %   PAR = INDUCTIONMACHINEPARS(n_p=..., R_s=..., R_r=..., L_ell=..., L_s=...) as
    %   in motulator, with constant parameters. The core losses are not modeled
    %   (G_c = 0).

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
    end
end
