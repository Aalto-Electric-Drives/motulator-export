classdef ObserverBasedGridFormingControllerCfg < motulator.internal.Object
    %OBSERVERBASEDGRIDFORMINGCONTROLLERCFG Configuration of disturbance-observer-based
    %   grid-forming control.
    %   CFG = OBSERVERBASEDGRIDFORMINGCONTROLLERCFG(i_max=..., L=..., ...) as in
    %   motulator, [] meaning None (the default).

    properties
        i_max                  % Maximum current (A, peak)
        L                      % Inductance estimate (H)
        R = 0                  % Resistance estimate (Ohm)
        R_a = []               % Active resistance (Ohm), [] = default
        k_v = []               % Voltage gain, [] = default
        alpha_o = 2*pi*50      % Observer bandwidth (rad/s)
        alpha_c = 2*pi*400     % Current-control bandwidth (rad/s)
        u_nom = sqrt(2/3)*400  % Nominal grid voltage (V, peak, line-neutral)
        w_nom = 2*pi*50        % Nominal grid angular frequency (rad/s)
        T_s = 125e-6           % Sampling period (s)
        i_d_max = []           % Maximum d-axis current of the power limitation (A)
        alpha_l = 2*pi*50      % Bandwidth of the power limitation (rad/s)
    end

    methods
        function obj = ObserverBasedGridFormingControllerCfg(varargin)
            obj.set_args(varargin, {'i_max', 'L'}, {'i_max', 'L'});
        end
    end
end
