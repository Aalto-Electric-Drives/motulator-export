classdef Sensor < motulator.internal.Object
    %SENSOR Ideal sensor of a measured signal.
    %   SENSOR = SENSOR() as in motulator, whose sensor can also have a bandwidth, a
    %   gain error, an offset, and noise. The models have ideal sensors.

    methods
        function obj = Sensor(varargin)
            obj.set_args(varargin, {}, {});
        end
    end
end
