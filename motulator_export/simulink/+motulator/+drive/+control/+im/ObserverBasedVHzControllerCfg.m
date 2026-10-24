classdef ObserverBasedVHzControllerCfg < motulator.internal.Object
    %OBSERVERBASEDVHZCONTROLLERCFG Configuration of the observer-based V/Hz
    %   controller.
    %   CFG = OBSERVERBASEDVHZCONTROLLERCFG(psi_s_nom=..., i_s_max=..., ...) as in
    %   motulator, [] meaning None (the default). The default observer gain k_o
    %   only; pure open-loop V/Hz control is given by L_M = inf of the machine
    %   model, as in motulator.

    properties
        psi_s_nom              % Nominal stator flux linkage (Vs)
        i_s_max                % Maximum stator current (A)
        alpha_psi = 2*pi*100   % Flux-control bandwidth (rad/s)
        alpha_tau = 2*pi*20    % Torque-control bandwidth (rad/s)
        alpha_f = 2*pi*1       % Bandwidth of the low-pass filter of the torque (rad/s)
        k_o = []               % Observer gain, only [] (the default)
        k_u = 0.9              % Voltage utilization factor
        k_b = 0.9              % Breakdown torque margin
        T_s = 250e-6           % Sampling period (s)
    end

    methods
        function obj = ObserverBasedVHzControllerCfg(varargin)
            obj.set_args(varargin, {'psi_s_nom', 'i_s_max'}, {'psi_s_nom', 'i_s_max'});
            if ~isempty(obj.k_o)
                error('motulator:unsupported', 'Only k_o = [] supported.');
            end
        end
    end
end
