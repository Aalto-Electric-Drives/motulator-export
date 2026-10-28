classdef VoltageSourceConverter < motulator.internal.Object
    %VOLTAGESOURCECONVERTER Ideal two-level converter with a stiff DC bus.
    %   CONVERTER = VOLTAGESOURCECONVERTER(u_dc=...) as in motulator. The converter
    %   is modeled with the carrier comparison (modulator=CarrierComparison() of
    %   motulator) and the computational delay of one sampling period. The dead
    %   time and the threshold voltage u_th are not modeled in Simulink, so u_th
    %   must be zero (their compensation in the control system is supported, see
    %   PWM and InverterNonlinearity).

    properties
        u_dc      % DC-bus voltage (V)
        u_th = 0  % Threshold voltage (V)
    end

    methods
        function obj = VoltageSourceConverter(varargin)
            obj.set_args(varargin, {'u_dc', 'u_th'}, {'u_dc'});
            if obj.u_th ~= 0
                error('motulator:unsupported', ...
                    'The threshold voltage is not modeled.');
            end
        end
    end
end
