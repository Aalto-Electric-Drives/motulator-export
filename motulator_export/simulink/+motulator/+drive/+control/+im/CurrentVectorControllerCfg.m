classdef CurrentVectorControllerCfg < motulator.internal.Object
    %CURRENTVECTORCONTROLLERCFG Configuration of the current-vector controller.
    %   CFG = CURRENTVECTORCONTROLLERCFG(psi_s_nom=..., i_s_max=..., ...) as in
    %   motulator, [] meaning None (the default). The default observer gain k_o and
    %   the continuous-time current controller (discrete = false) only.

    properties
        psi_s_nom             % Nominal stator flux linkage (Vs)
        i_s_max               % Maximum stator current (A)
        alpha_c = 2*pi*200    % Current-control bandwidth (rad/s)
        alpha_i = []          % Integral-action bandwidth (rad/s), [] = alpha_c
        alpha_o = []          % Speed estimation pole (rad/s), [] = default
        k_o = []              % Observer gain, only [] (the default)
        w_s_nom = 2*pi*50     % Nominal stator angular frequency (rad/s)
        k_u = 0.95            % Voltage utilization factor
        k_fw = []             % Field-weakening gain (1/H), [] = default
        J = []                % Inertia (kgm^2) of the speed observer, [] = none
        sensorless = true     % Sensorless mode (or sensored)
        T_s = 125e-6          % Sampling period (s)
        discrete = false      % Discrete-time current controller, only false
    end

    methods
        function obj = CurrentVectorControllerCfg(varargin)
            obj.set_args(varargin, {'psi_s_nom', 'i_s_max'}, {'psi_s_nom', 'i_s_max'});
            if ~isempty(obj.k_o) || obj.discrete
                error('motulator:unsupported', ...
                    'Only k_o = [] and discrete = false supported.');
            end
        end
    end
end
