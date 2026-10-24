function c = control_system(ctrl, mdl)
%CONTROL_SYSTEM Block of the library and parameters of a control system.
%   C = CONTROL_SYSTEM(CTRL, MDL) gives the struct C of the control system CTRL
%   of the system model MDL with the fields
%
%     name       Name of its block in the library
%     variables  Parameters of the model (the arguments of the top-level class,
%                named as in motulator, to which the mask of the block refers)
%     T_s        Sampling period (s)
%     T_s_expr   Sampling period as an expression of the parameters
%     refs       References {name of the input, constant or StepSignal}
import motulator.internal.*
if isa(ctrl, 'motulator.drive.control.VectorControlSystem')
    vc = ctrl.vector_ctrl;
    if isa(vc, 'motulator.drive.control.sm.FluxVectorController')
        c.name = 'Flux-vector control';
        plant = {'machine', 'motulator.drive.model.SynchronousMachine'};
    elseif isa(vc, 'motulator.drive.control.im.CurrentVectorController')
        c.name = 'Current-vector control';
        plant = {'machine', 'motulator.drive.model.InductionMachine'};
    else
        error('motulator:unsupported', ['Only FluxVectorController and ' ...
            'CurrentVectorController supported in VectorControlSystem.']);
    end
    c.variables = struct('vector_ctrl', vc.params(), ...
        'speed_ctrl', ctrl.speed_ctrl.params(), 'pwm', ctrl.pwm.params());
    c.T_s = vc.cfg.T_s;
    c.T_s_expr = 'vector_ctrl.cfg.T_s';
    c.refs = {'w_M_ref', required(ctrl.w_M_ref, 'speed reference', 'set_speed_ref')};
elseif isa(ctrl, 'motulator.drive.control.VHzControlSystem')
    vhz = ctrl.vhz_ctrl;
    check_class(vhz, ...
        'motulator.drive.control.im.ObserverBasedVHzController');
    c.name = 'Observer-based VHz control';
    plant = {'machine', 'motulator.drive.model.InductionMachine'};
    c.variables = struct('vhz_ctrl', vhz.params(), 'slew_rate', ctrl.slew_rate, ...
        'pwm', ctrl.pwm.params());
    c.T_s = vhz.cfg.T_s;
    c.T_s_expr = 'vhz_ctrl.cfg.T_s';
    c.refs = {'w_M_ref', required(ctrl.w_M_ref, 'speed reference', 'set_speed_ref')};
elseif isa(ctrl, 'motulator.grid.control.GridConverterControlSystem')
    inner = ctrl.inner_ctrl;
    p_g_ref = required(ctrl.p_g_ref, 'active power reference', 'set_power_ref');
    if isa(inner, 'motulator.grid.control.CurrentVectorController')
        c.name = 'Grid-following control';
        plant = {'ac_filter', 'motulator.grid.model.LCLFilter'};
        c.refs = {'p_g_ref', p_g_ref; 'q_g_ref', required(ctrl.q_g_ref, ...
            'reactive power reference', 'set_reactive_power_ref')};
    elseif isa(inner, 'motulator.grid.control.ObserverBasedGridFormingController')
        c.name = 'Grid-forming control';
        plant = {'ac_filter', 'motulator.grid.model.LFilter'};
        c.refs = {'p_g_ref', p_g_ref; 'v_c_ref', required(ctrl.v_c_ref, ...
            'AC voltage reference', 'set_ac_voltage_ref')};
    else
        error('motulator:unsupported', ['Only CurrentVectorController and ' ...
            'ObserverBasedGridFormingController supported.']);
    end
    c.variables = struct('inner_ctrl', inner.params(), 'pwm', ctrl.pwm.params());
    c.T_s = inner.cfg.T_s;
    c.T_s_expr = 'inner_ctrl.cfg.T_s';
else
    error('motulator:unsupported', ['Only VectorControlSystem, VHzControlSystem, ' ...
        'and GridConverterControlSystem supported.']);
end
% The part of the system model that the control system needs
[part, cls] = plant{:};
if ~any(strcmp(part, fieldnames(mdl))) || ~isa(mdl.(part), cls)
    error('motulator:unsupported', '%s needs a system model with the %s %s.', ...
        c.name, part, cls);
end
end

function value = required(value, name, setter)
% A reference, which must be set
if isempty(value)
    error('motulator:arguments', 'The %s must be set (%s).', name, setter);
end
end
