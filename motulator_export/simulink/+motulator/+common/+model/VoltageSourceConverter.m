classdef VoltageSourceConverter < motulator.internal.Object
    %VOLTAGESOURCECONVERTER Ideal two-level converter with a stiff DC bus.
    %   CONVERTER = VOLTAGESOURCECONVERTER(u_dc=...) as in motulator. The dead time
    %   t_d and the threshold voltage u_th are not modeled in Simulink, so they
    %   must be zero (their compensation in the control system is supported, see
    %   PWM and InverterNonlinearity).

    properties
        u_dc      % DC-bus voltage (V)
        t_d = 0   % Dead time (s)
        u_th = 0  % Threshold voltage (V)
    end

    methods
        function obj = VoltageSourceConverter(varargin)
            obj.set_args(varargin, {'u_dc', 't_d', 'u_th'}, {'u_dc'});
            if obj.t_d ~= 0 || obj.u_th ~= 0
                error('motulator:unsupported', ...
                    'The dead time and the threshold voltage are not modeled.');
            end
        end
    end
end
