classdef FluxVectorControllerCfg < motulator.internal.Object
    %FLUXVECTORCONTROLLERCFG Configuration of the flux-vector controller.
    %   CFG = FLUXVECTORCONTROLLERCFG(i_s_max=..., ...) as in motulator, [] meaning
    %   None (the default). The observer gain k_o is [k0 k1], k_o(w_m) = k0 +
    %   k1*abs(w_m), or [] for the default. The offline reference generation
    %   (online_ref = false) and k_f = [] only.

    properties
        i_s_max                % Maximum stator current (A)
        alpha_tau = 2*pi*100   % Torque-control bandwidth (rad/s)
        alpha_psi = []         % Flux-control bandwidth (rad/s), [] = alpha_tau
        alpha_i = []           % Integral-action bandwidth (rad/s), [] = alpha_tau
        alpha_o = []           % Speed estimation poles (rad/s), [] = default
        alpha_ref = 2*pi*100   % Bandwidth of the online reference generation
        k_o = []               % Observer gain [k0 k1], [] = default
        k_f = []               % PM-flux estimation gain, only []
        psi_s_min = []         % Minimum stator flux (Vs), [] = psi_f
        psi_s_max = inf        % Maximum stator flux (Vs)
        k_u = 0.9              % Voltage utilization factor
        k_mtpv = 0.85          % MTPV margin
        J = []                 % Inertia (kgm^2) of the speed observer, [] = none
        sensorless = true      % Sensorless mode (or sensored)
        online_ref = false     % Online reference generation, only false
        T_s = 125e-6           % Sampling period (s)
    end

    methods
        function obj = FluxVectorControllerCfg(varargin)
            obj.set_args(varargin, {'i_s_max'}, {'i_s_max'});
            if ~isempty(obj.k_f) || obj.online_ref
                error('motulator:unsupported', ...
                    'Only k_f = [] and online_ref = false supported.');
            end
        end
    end
end
