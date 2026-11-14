classdef InductionMachineInvGammaPars < motulator.internal.Object
    %INDUCTIONMACHINEINVGAMMAPARS Inverse-Gamma-model parameters of an induction
    %   machine.
    %   PAR = INDUCTIONMACHINEINVGAMMAPARS(n_p=..., R_s=..., R_R=..., L_sgm=...,
    %   L_M=...) as in motulator, with constant parameters. The core losses are not
    %   modeled (G_c = 0).

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
end
