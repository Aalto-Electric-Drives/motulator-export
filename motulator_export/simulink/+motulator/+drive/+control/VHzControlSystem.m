classdef VHzControlSystem < motulator.internal.Object
    %VHZCONTROLSYSTEM V/Hz control system of a machine drive.
    %   CTRL = VHZCONTROLSYSTEM(VHZ_CTRL, slew_rate=..., pwm=...) as in motulator,
    %   with the V/Hz controller VHZ_CTRL (ObserverBasedVHzController), the slew
    %   rate of the speed reference (mechanical rad/s^2), and the PWM (by default
    %   PWM(overmodulation='MME')). SET_SPEED_REF(W_M_REF) sets the speed reference
    %   (mechanical rad/s): a constant or a StepSignal.

    properties
        vhz_ctrl         % V/Hz controller
        slew_rate = inf  % Slew rate of the speed reference (rad/s^2)
        pwm = []         % Pulse-width modulator (PWM)
        w_M_ref = []     % Speed reference (rad/s), a constant or a StepSignal
    end

    methods
        function obj = VHzControlSystem(varargin)
            obj.set_args(varargin, {'vhz_ctrl', 'slew_rate', 'pwm'}, {'vhz_ctrl'});
            if isempty(obj.pwm)
                obj.pwm = motulator.common.control.PWM('overmodulation', 'MME');
            end
        end

        function set_speed_ref(obj, w_M_ref)
            %SET_SPEED_REF Set the speed reference (mechanical rad/s).
            obj.w_M_ref = w_M_ref;
        end
    end
end
