classdef Drive < motulator.internal.Object
    %DRIVE Continuous-time model of a machine drive.
    %   MDL = DRIVE(MACHINE, MECHANICS, CONVERTER) as in motulator.

    properties
        machine    % Machine model (SynchronousMachine or InductionMachine)
        mechanics  % Mechanical system model (MechanicalSystem or ExternalRotorSpeed)
        converter  % Converter model (VoltageSourceConverter)
    end

    methods
        function obj = Drive(varargin)
            args = {'machine', 'mechanics', 'converter'};
            obj.set_args(varargin, args, args);
        end
    end
end
