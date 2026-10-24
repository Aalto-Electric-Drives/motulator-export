classdef NominalValues < motulator.internal.Object
    %NOMINALVALUES Nominal values.
    %   NOM = NOMINALVALUES(U=..., I=..., f=..., P=..., tau=...) as in motulator.

    properties
        U    % Voltage (V, rms, line-line)
        I    % Current (A, rms)
        f    % Frequency (Hz)
        P    % Power (W)
        tau = 0  % Torque (Nm)
    end

    methods
        function obj = NominalValues(varargin)
            obj.set_args(varargin, {'U', 'I', 'f', 'P', 'tau'}, {'U', 'I', 'f', 'P'});
        end
    end
end
