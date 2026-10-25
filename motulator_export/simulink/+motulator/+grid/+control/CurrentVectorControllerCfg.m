classdef CurrentVectorControllerCfg < motulator.internal.Object
    %CURRENTVECTORCONTROLLERCFG Configuration of grid-following control.
    %   CFG = CURRENTVECTORCONTROLLERCFG(i_max=..., L=..., ...) as in motulator, []
    %   meaning None (the default).

    properties
        i_max                  % Maximum current (A, peak)
        L                      % Inductance estimate (H)
        alpha_c = 2*pi*400     % Current-control bandwidth (rad/s)
        alpha_i = []           % Integral-action bandwidth (rad/s), [] = alpha_c
        u_nom = sqrt(2/3)*400  % Nominal grid voltage (V, peak, line-neutral)
        w_nom = 2*pi*50        % Nominal grid angular frequency (rad/s)
        alpha_pll = 2*pi*20    % PLL frequency-tracking bandwidth (rad/s)
        T_s = 125e-6           % Sampling period (s)
    end

    methods
        function obj = CurrentVectorControllerCfg(varargin)
            obj.set_args(varargin, {'i_max', 'L'}, {'i_max', 'L'});
        end
    end
end
