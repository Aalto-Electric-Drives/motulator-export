classdef Drive < motulator.internal.Object
    %DRIVE Continuous-time model of a machine drive.
    %   MDL = DRIVE(MACHINE, MECHANICS, CONVERTER) as in motulator, with the sensors
    %   given by name, e.g., DRIVE(MACHINE, MECHANICS, CONVERTER, sensors=SENSORS)
    %   (by default DriveSensors(), i.e., the phase-current and DC-bus voltage
    %   sensors).

    properties
        machine       % Machine model (SynchronousMachine or InductionMachine)
        mechanics     % Mechanical system model (MechanicalSystem or ExternalRotorSpeed)
        converter     % Converter model (VoltageSourceConverter)
        sensors = []  % Sensors (DriveSensors)
    end

    methods
        function obj = Drive(varargin)
            args = {'machine', 'mechanics', 'converter'};
            obj.set_args(varargin, args, args);
            if isempty(obj.sensors)
                obj.sensors = motulator.drive.model.DriveSensors();
            end
        end
    end
end
