classdef ObserverBasedVHzControllerCfg < motulator.internal.Object
    %OBSERVERBASEDVHZCONTROLLERCFG Configuration of the observer-based V/Hz
    %   controller.
    %   CFG = OBSERVERBASEDVHZCONTROLLERCFG(i_s_max=..., ...) as in motulator, []
    %   meaning None (the default). The observer gain k_o is [k0 k1], k_o(w_m) =
    %   k0 + k1*abs(w_m), or [] for the default.

    properties
        i_s_max                % Maximum stator current (A)
        alpha_psi = 2*pi*100   % Flux-control bandwidth (rad/s)
        alpha_tau = 2*pi*20    % Torque-control bandwidth (rad/s)
        alpha_f = 2*pi*1       % Bandwidth of the low-pass filter of the torque (rad/s)
        alpha_o = 2*pi*200     % Angle estimation pole (rad/s)
        k_o = []               % Observer gain [k0 k1], [] = default
        k_u = 0.9              % Voltage utilization factor
        k_mtpv = 0.9           % MTPV margin
        psi_s_min = []         % Minimum stator flux (Vs), [] = psi_f
        psi_s_max = inf        % Maximum stator flux (Vs)
        T_s = 250e-6           % Sampling period (s)
    end

    methods
        function obj = ObserverBasedVHzControllerCfg(varargin)
            obj.set_args(varargin, {'i_s_max'}, {'i_s_max'});
        end
    end
end
