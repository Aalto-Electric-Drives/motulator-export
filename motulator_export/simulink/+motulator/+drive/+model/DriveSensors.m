classdef DriveSensors < motulator.internal.Object
    %DRIVESENSORS Sensors of a machine drive.
    %   SENSORS = DRIVESENSORS(I_C, U_DC, W_M, THETA_M) as in motulator, with the
    %   phase-current sensors I_C and the DC-bus voltage sensor U_DC (by default
    %   Sensor()), and the rotor speed sensor W_M and the rotor position sensor
    %   THETA_M (by default [], i.e., no sensor), e.g., DriveSensors(theta_M=Sensor())
    %   for sensored control of a synchronous machine.

    properties
        i_c           % Phase-current sensors (Sensor)
        u_dc          % DC-bus voltage sensor (Sensor)
        w_M = []      % Rotor speed sensor (Sensor), [] for no sensor
        theta_M = []  % Rotor position sensor (Sensor), [] for no sensor
    end

    methods
        function obj = DriveSensors(varargin)
            obj.set_args(varargin, {'i_c', 'u_dc', 'w_M', 'theta_M'}, {});
            if isempty(obj.i_c)
                obj.i_c = motulator.common.model.Sensor();
            end
            if isempty(obj.u_dc)
                obj.u_dc = motulator.common.model.Sensor();
            end
        end
    end
end
