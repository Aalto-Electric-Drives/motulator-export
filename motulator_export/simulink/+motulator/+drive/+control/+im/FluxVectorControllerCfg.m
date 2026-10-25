classdef FluxVectorControllerCfg < motulator.internal.Object
    %FLUXVECTORCONTROLLERCFG Configuration of the flux-vector controller.
    %   CFG = FLUXVECTORCONTROLLERCFG(psi_s_nom=..., i_s_max=..., ...) as in
    %   motulator, [] meaning None (the default). The default observer gain k_o
    %   only.

    properties
        psi_s_nom             % Nominal stator flux linkage (Vs)
        i_s_max               % Maximum stator current (A)
        alpha_tau = 2*pi*100  % Torque-control bandwidth (rad/s)
        alpha_psi = []        % Flux-control bandwidth (rad/s), [] = alpha_tau
        alpha_i = []          % Integral-action bandwidth (rad/s), [] = alpha_tau
        alpha_o = []          % Speed estimation poles (rad/s), [] = default
        k_o = []              % Observer gain, only [] (the default)
        tau_M_max = inf       % Maximum torque reference (Nm)
        k_u = 0.9             % Voltage utilization factor
        k_b = 0.9             % Breakdown torque margin
        J = []                % Inertia (kgm^2) of the speed observer, [] = none
        sensorless = true     % Sensorless mode (or sensored)
        T_s = 125e-6          % Sampling period (s)
    end

    methods
        function obj = FluxVectorControllerCfg(varargin)
            obj.set_args(varargin, {'psi_s_nom', 'i_s_max'}, {'psi_s_nom', 'i_s_max'});
            if ~isempty(obj.k_o)
                error('motulator:unsupported', 'Only k_o = [] supported.');
            end
        end
    end
end
