classdef InverterNonlinearity < motulator.internal.Object
    %INVERTERNONLINEARITY Duty-ratio error model of the converter.
    %   D_ERR = INVERTERNONLINEARITY(t_d=..., u_th=..., i_0=...) as in motulator,
    %   with the current-direction function sign = tanh(i/i_0), or the sign
    %   function if i_0 = 0 (the default, np.sign of motulator). The control system
    %   compensates for the error of the dead time t_d and the threshold voltage
    %   u_th (PWM with feedforward = true).

    properties
        t_d = 0   % Dead time (s)
        u_th = 0  % Threshold voltage (V)
        i_0 = 0   % Current scale (A) of sign = tanh(i/i_0), 0 = the sign function
    end

    methods
        function obj = InverterNonlinearity(varargin)
            obj.set_args(varargin, {'t_d', 'u_th', 'i_0'}, {});
        end
    end
end
