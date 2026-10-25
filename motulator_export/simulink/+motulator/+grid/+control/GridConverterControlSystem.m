classdef GridConverterControlSystem < motulator.internal.Object
    %GRIDCONVERTERCONTROLSYSTEM Control system of a grid converter.
    %   CTRL = GRIDCONVERTERCONTROLSYSTEM(INNER_CTRL, pwm=...) as in motulator, with
    %   the inner controller INNER_CTRL (CurrentVectorController or
    %   ObserverBasedGridFormingController) and the PWM (by default PWM()), without
    %   the DC-bus voltage controller. The references are set by SET_POWER_REF,
    %   SET_REACTIVE_POWER_REF (grid-following control), and SET_AC_VOLTAGE_REF
    %   (grid-forming control): constants or StepSignals.

    properties
        inner_ctrl                 % Inner controller
        dc_bus_voltage_ctrl = []   % DC-bus voltage controller, only []
        pwm = []                   % Pulse-width modulator (PWM)
        p_g_ref = []               % Active power reference (W)
        q_g_ref = []               % Reactive power reference (VAr)
        v_c_ref = []               % AC voltage reference (V)
    end

    methods
        function obj = GridConverterControlSystem(varargin)
            obj.set_args(varargin, {'inner_ctrl', 'dc_bus_voltage_ctrl', 'pwm'}, ...
                {'inner_ctrl'});
            if ~isempty(obj.dc_bus_voltage_ctrl)
                error('motulator:unsupported', 'DC-bus voltage control not supported.');
            end
            if isempty(obj.pwm)
                obj.pwm = motulator.common.control.PWM();
            end
        end

        function set_power_ref(obj, p_g_ref)
            %SET_POWER_REF Set the active power reference (W).
            obj.p_g_ref = p_g_ref;
        end

        function set_reactive_power_ref(obj, q_g_ref)
            %SET_REACTIVE_POWER_REF Set the reactive power reference (VAr).
            obj.q_g_ref = q_g_ref;
        end

        function set_ac_voltage_ref(obj, v_c_ref)
            %SET_AC_VOLTAGE_REF Set the AC voltage reference (V, peak, line-neutral).
            obj.v_c_ref = v_c_ref;
        end
    end
end
