classdef SynchronousMachinePars < motulator.internal.Object
    %SYNCHRONOUSMACHINEPARS Parameters of a synchronous machine with constant
    %   inductances.
    %   PAR = SYNCHRONOUSMACHINEPARS(n_p=..., R_s=..., L_d=..., L_q=..., psi_f=...)
    %   as in motulator. The core losses are not modeled (G_c = 0).

    properties
        n_p       % Number of pole pairs
        R_s       % Stator resistance (Ohm)
        L_d       % d-axis inductance (H)
        L_q       % q-axis inductance (H)
        psi_f     % PM-flux linkage (Vs)
        G_c = 0   % Core-loss conductance (S), only 0
    end

    methods
        function obj = SynchronousMachinePars(varargin)
            args = {'n_p', 'R_s', 'L_d', 'L_q', 'psi_f'};
            obj.set_args(varargin, [args, {'G_c'}], args);
            motulator.internal.check_zero(obj, 'G_c', 'Core losses');
        end
    end
end
