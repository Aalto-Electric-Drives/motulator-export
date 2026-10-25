"""
Building blocks shared by the drive systems: the mechanics (`MechanicalSystem` or
`ExternalRotorSpeed`), the speed controller (or the torque-control mode without it),
and the scope.

"""

from typing import Any, cast

import numpy as np
from motulator.common.model._converter import (
    DiodeBridgeDCBus,
    DiodeBridgeDCBusStates,
    VoltageSourceConverter,
)
from motulator.common.model._pwm import CarrierComparison
from motulator.common.utils._utils import get_value, wrap
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._common import SpeedController
from motulator.drive.model import (
    Drive,
    ExternalRotorSpeed,
    MechanicalSystem,
    SynchronousMachine,
)

from motulator_export._common import (
    StepSignal,
    check_supported_dc_bus,
    check_supported_pwm,
    reference,
)
from motulator_export.plecs._common import (
    MACH,
    ControlBlock,
    _add_ctrl_output,
    _add_step,
    _check_delay,
    _check_initial_states,
)
from motulator_export.plecs._schematic import _fmt, _probe, _Schematic, _scope

# Machine signals in the output port "mdl"
MDL_OUTPUTS = ["i_a", "i_b", "i_c", "w_M", "theta_M", "tau_M"]


def _check_supported_plant(mdl: Drive, dead_time: bool = False) -> None:
    """
    Raise an error if the mechanics, the converter, the DC bus, or the PWM is not
    supported.

    The dead time of the carrier comparison (`mdl.pwm.t_d`, by default that of the
    converter) and the threshold voltage of the converter (`u_th`) are supported only
    if `dead_time` is True, and only with the current-direction function `np.sign` of
    the converter. The computational delay must be one sampling period (`delay=1`,
    or `delay=2` with `single_update`).

    """
    mechanics = mdl.mechanics
    if not isinstance(mechanics, ExternalRotorSpeed) and (
        not isinstance(mechanics, MechanicalSystem) or mechanics.B_L != 0
    ):
        raise NotImplementedError(
            "Only MechanicalSystem without friction and ExternalRotorSpeed supported"
        )
    if mdl.lc_filter is not None:
        raise NotImplementedError("LC filter not supported")
    if type(mdl.converter) is not VoltageSourceConverter:
        raise NotImplementedError("Only VoltageSourceConverter supported")
    if mdl.dc_bus is not None:
        check_supported_dc_bus(mdl.dc_bus, mdl.converter)
    if mdl.machine.i_s_sensor is not None or mdl.converter.u_dc_sensor is not None:
        raise NotImplementedError("Sensor models not supported")
    if not isinstance(mdl.pwm, CarrierComparison):
        raise NotImplementedError("Only CarrierComparison (pwm=True) supported")
    if mdl.pwm.t_d != 0 or mdl.converter.u_th != 0:
        if not dead_time:
            raise NotImplementedError(
                "Converter dead time and threshold voltage not supported"
            )
        if mdl.converter.sign is not np.sign:
            raise NotImplementedError(
                "Only the dead time and the threshold voltage with sign=np.sign "
                "supported"
            )
        if mdl.pwm.t_d < 0 or mdl.converter.u_th < 0:
            raise NotImplementedError("Only t_d >= 0 and u_th >= 0 supported")
    _check_delay(mdl)
    _check_initial_drive_states(mdl)


def _check_initial_drive_states(mdl: Drive) -> None:
    """
    Raise an error if a state of the drive (other than the speed and the angle of
    the mechanics and the DC-bus voltage, which are exported) is not at its initial
    value. The synchronous machine model has its own rotor angle, which must be n_p
    times that of the mechanics (modulo 2*pi), as in the exported models.
    """
    machine = mdl.machine
    mech = mdl.mechanics.state
    exported = [("mechanics.state.theta_M", mech.theta_M)]
    if isinstance(mdl.mechanics, MechanicalSystem):  # The speed is a state
        exported.append(("mechanics.state.w_M", mdl.mechanics.state.w_M))
    states: list[tuple[str, complex, complex]]
    if isinstance(machine, SynchronousMachine):
        # The angle n_p*theta_M closest to the angle of the machine model
        theta_m = machine.state.theta_m.real
        theta_m0 = theta_m - wrap(theta_m - machine.par.n_p * mech.theta_M.real)
        states = [
            ("machine.state.psi_s_dq", machine.state.psi_s_dq, machine.par.psi_f),
            (
                "machine.state.theta_m (n_p times the angle of the mechanics)",
                theta_m,
                theta_m0,
            ),
        ]
    else:
        states = [
            ("machine.state.psi_s_ab", machine.state.psi_s_ab, 0j),
            ("machine.state.psi_r_ab", machine.state.psi_r_ab, 0j),
        ]
    dc_bus = mdl.dc_bus
    if dc_bus is not None:
        exported.append(("dc_bus.state.u_dc", dc_bus.state.u_dc))
    if isinstance(dc_bus, DiodeBridgeDCBus):
        bridge = cast(DiodeBridgeDCBusStates, dc_bus.state)
        states.append(("dc_bus.state.i_L", bridge.i_L, 0j))
        states.append(("dc_bus.state.exp_j_theta_g", bridge.exp_j_theta_g, 1))
    _check_initial_states(states, exported)


def _check_supported_speed_control(
    ctrl: VectorControlSystem, dead_time: bool = False
) -> None:
    """
    Raise an error if the speed controller (SpeedController, or none in the
    torque-control mode) or the PWM is not supported.

    The duty-ratio error model `d_err` of the PWM is supported only if `dead_time` is
    True (see `_control.pwm_values`).

    """
    if ctrl.speed_ctrl is not None and not isinstance(ctrl.speed_ctrl, SpeedController):
        raise NotImplementedError("Only SpeedController (or none) supported")
    if type(ctrl).modulate is not VectorControlSystem.modulate:
        raise NotImplementedError("Overridden modulate method not supported")
    check_supported_pwm(ctrl, d_err=dead_time)


def _check_supported_vhz_control(ctrl: VHzControlSystem) -> None:
    """
    Raise an error if the V/Hz control system (its methods or its PWM) is not
    supported, the duty-ratio error model `d_err` of the PWM included.
    """
    if type(ctrl).modulate is not VHzControlSystem.modulate:
        raise NotImplementedError("Overridden modulate method not supported")
    check_supported_pwm(ctrl, d_err=True)


def speed_ctrl_values(ctrl: VectorControlSystem) -> dict[str, Any]:
    """
    Get the parameter values of the speed controller (the struct speed_ctrl of the
    model initialization): the arguments of `SpeedController`, whose gains must be
    those of the arguments (the exported controller computes its gains from them).

    The controller stores the integral action bandwidth as the ratio of its gains,
    which may differ from the argument by a rounding error. The default (None, i.e.,
    `alpha_s`) is exported as None if it gives the gains of the controller, so that
    the exported controller computes the same gains as motulator. In the
    torque-control mode (without the speed controller), there are no values.

    """
    if ctrl.speed_ctrl is None:
        return {}
    speed_ctrl = cast(SpeedController, ctrl.speed_ctrl)
    J, alpha_s, u_max = speed_ctrl.J, speed_ctrl.alpha_s, speed_ctrl.u_max
    gains = (speed_ctrl.k_p, speed_ctrl.k_t, speed_ctrl.alpha_i)
    default = SpeedController(J, alpha_s, None, u_max)
    alpha_i = None
    if gains != (default.k_p, default.k_t, default.alpha_i):
        alpha_i = speed_ctrl.alpha_i
        given = SpeedController(J, alpha_s, alpha_i, u_max)
        if not np.allclose((given.k_p, given.k_t, given.alpha_i), gains, rtol=1e-12):
            raise NotImplementedError(
                "The gains of the speed controller must be those of its arguments "
                "(SpeedController), which the exported controller has"
            )
    return {
        "speed_J": J,
        "speed_alpha_s": alpha_s,
        "speed_alpha_i": alpha_i,
        "speed_tau_M_max": u_max,
    }


def external_speed(mdl: Drive) -> bool:
    """Whether the mechanics is the external rotor speed (ExternalRotorSpeed)."""
    return isinstance(mdl.mechanics, ExternalRotorSpeed)


def torque_control(ctrl: VectorControlSystem | VHzControlSystem) -> bool:
    """Whether the control system is a vector control system in torque-control mode."""
    return isinstance(ctrl, VectorControlSystem) and ctrl.speed_ctrl is None


def drive_references(
    mdl: Drive, ctrl: VectorControlSystem | VHzControlSystem, T_s: float
) -> tuple[StepSignal | float, StepSignal | float]:
    """
    Speed reference (`set_speed_ref`), or the torque reference in the torque-control
    mode (`set_torque_ref`), and the load torque of the mechanics
    (`set_external_load_torque`), or the external rotor speed of `ExternalRotorSpeed`
    (`set_external_rotor_speed`): constants or `StepSignal` objects, the reference
    sampled to switch at the same sampling instant as in motulator (the signal of
    the mechanics acts in continuous time).
    """
    if torque_control(ctrl):
        ref = reference(ctrl.ext_ref.tau_M, "torque reference", T_s)
    else:
        ref = reference(ctrl.ext_ref.w_M, "speed reference", T_s)
    mechanics = mdl.mechanics
    if isinstance(mechanics, ExternalRotorSpeed):
        return ref, reference(mechanics.w_M, "external rotor speed")
    return ref, reference(cast(MechanicalSystem, mechanics).inp.tau_L, "load torque")


def mechanics_and_converter_variables(mdl: Drive) -> list[tuple[str, Any]]:
    """
    Get the workspace variables of the mechanics (with the initial speed and angle
    of its state, or of ExternalRotorSpeed the initial angle and the external speed
    at the start), the converter, and the DC bus (with the initial voltage of its
    state).
    """
    mech = mdl.mechanics
    if isinstance(mech, ExternalRotorSpeed):
        variables: list[tuple[str, Any]] = [
            ("mechanics.w_M0", float(get_value(mech.w_M, 0.0)))
        ]
    else:
        mech = cast(MechanicalSystem, mech)
        variables = [
            ("mechanics.J", mech.J),
            ("mechanics.w_M0", float(mech.state.w_M.real)),
        ]
    variables += [("mechanics.theta_M0", float(wrap(mech.state.theta_M.real)))]
    dc_bus = mdl.dc_bus
    if dc_bus is None:
        variables += [("converter.u_dc", mdl.converter.u_dc)]
    t_d = cast(CarrierComparison, mdl.pwm).t_d
    if t_d > 0:
        variables += [("converter.t_d", t_d)]
    if mdl.converter.u_th > 0:
        variables += [("converter.u_th", mdl.converter.u_th)]
    if dc_bus is not None:
        variables += [("dc_bus.C_dc", dc_bus.C_dc)]
        variables += [("dc_bus.u_dc0", float(dc_bus.state.u_dc.real))]
    if isinstance(dc_bus, DiodeBridgeDCBus):
        variables += [("dc_bus.L_dc", dc_bus.L_dc)]
        variables += [("dc_bus.u_g", dc_bus.u_g), ("dc_bus.w_g", dc_bus.w_g)]
    return variables


def _add_phase_lines(sch: _Schematic) -> None:
    """Add the phase lines from the converter to the machine (added last)."""
    for k in range(3):
        sch.wire(("Converter", k + 1), ("Machine", k + 1))


MACHINE_FRAME = "      Frame         [-25, -25; 25, 35]\n"
MACHINE_TERMINALS = [
    ("Port", -30, -10, "left"),
    ("Port", -30, 0, "left"),
    ("Port", -30, 10, "left"),
    ("Rotational", 30, 30, "right"),
]


# Moment of inertia of the PLECS machines with the external rotor speed: zero, since
# the speed source imposes the speed (PLECS rejects an inertia whose state the source
# determines)
EXTERNAL_SPEED_J = "0"


def _add_mechanics(
    sch: _Schematic, signal: StepSignal | float, inertia: bool, external: bool = False
) -> None:
    """
    Add the load torque `signal` (and the inertia, if not included in the machine),
    or with the external rotor speed (`external`), the speed source of the speed
    `signal`.
    """
    x_f, y_f = MACH[0] + 30, MACH[1] + 30  # Flange of the machine
    inertia = inertia and not external  # The speed source imposes the speed
    y_load = y_f + 90 if inertia else y_f + 50
    if inertia:
        sch.component(
            "Inertia",
            "Inertia",
            (x_f, y_f + 30),
            {
                "J": "mechanics.J",
                "w0": "mechanics.w_M0",
                "theta0": "mechanics.theta_M0",
            },
            label="east",
        )
        sch.connect(("Machine", 4), ("Inertia", 1), "Rotational")
        sch.connect(("Inertia", 2), ("Load", 3), "Rotational")
    elif external:  # The speed source of one flange imposes the speed of the machine
        sch.connect(("Machine", 4), ("Load", 1), "Rotational")
    else:
        sch.connect(("Machine", 4), ("Load", 3), "Rotational")
    source = "w_M ext." if external else "tau_L"
    if isinstance(signal, StepSignal):
        # Several steps are summed by a Gain block in front of the load
        x_step = x_f - 70 if np.isscalar(signal.time) else x_f - 120
        load_input = _add_step(sch, source, signal, (x_step, y_load))
    else:
        sch.component("Constant", source, (x_f - 70, y_load), {"Value": _fmt(signal)})
        load_input = (source, 1)
    sch.component(
        "ControlledRotationalSpeed" if external else "ControlledTorque",
        "Load",
        (x_f, y_load),
        {"SecondFlange": "1" if external else "2", "StateSpaceInlining": "2"},
        direction="left",
        label="east",
    )
    if not external:
        sch.component(
            "RotationalReference",
            "Frame",
            (x_f, y_load + 45),
            direction="up",
            show=False,
        )
        sch.connect(("Frame", 1), ("Load", 1), "Rotational")
    sch.signal(load_input, ("Load", 2))


# Probe signals of the PLECS machines: (current, speed, angle, torque)
MACHINE_PROBES = ["Stator phase currents", "Rotational speed", "Rotor position"]
MACHINE_PROBES += ["Electrical torque"]


def _add_drive_outputs(
    sch: _Schematic, block: ControlBlock, outputs: bool, y_sources: int
) -> int:
    """
    Add the scope and its probes, and optionally the output ports.

    The first three monitored signal groups of the block (the speed, the torque, and
    the flux linkage) are shown in the scope with the machine signals. The probes are
    below the sources of the control system, the last of which is at `y_sources`.
    Returns the y-coordinate of the last probe.

    """
    speed, torque, flux = list(block.outputs)[:3]
    if outputs:  # Output ports for scripted simulations: machine and controller signals
        probe = _probe("Machine", MACHINE_PROBES)
        sch.component("PlecsProbe", "Machine signals", (400, 300), extra=probe)
        sch.component("Output", "mdl", (480, 300), {"Index": "1", "Width": "-1"})
        sch.signal(("Machine signals", 1), ("mdl", 1))
        _add_ctrl_output(sch, block)
    # Scope, fed by probes of the controller outputs and the machine, below the control
    # system (not shifted by sch.dx, since the probes have no wires)
    dx, sch.dx = sch.dx, 0
    x, y = 80, max(270, y_sources + 50)  # Position of the first probe
    probes = [
        ("Speed (ctrl)", block.name, [speed]),
        ("Speed", "Machine", [MACHINE_PROBES[1]]),
        ("Torque (ctrl)", block.name, [torque]),
        ("Torque", "Machine", [MACHINE_PROBES[3]]),
        ("Currents", "Machine", [MACHINE_PROBES[0]]),
        ("Flux (ctrl)", block.name, [flux]),
    ]
    for k, (name, comp, signals) in enumerate(probes):
        extra = _probe(comp, signals)
        sch.component("PlecsProbe", name, (x, y + 40 * k), extra=extra)
    # The controller signals (also without the speed reference in torque control)
    # with the machine signal
    for name, group, y_mux in [("speed", speed, y + 40), ("torque", torque, y + 60)]:
        mux = {"Width": f"[{len(block.outputs[group])} 1]"}
        sch.component("SignalMux", f"Mux {name}", (x + 100, y_mux), mux, show=False)
    sch.signal(("Speed (ctrl)", 1), ("Mux speed", 2), [(x + 40, y), (x + 40, y + 35)])
    sch.signal(("Speed", 1), ("Mux speed", 3), [(x + 50, y + 40), (x + 50, y + 45)])
    sch.signal(
        ("Torque (ctrl)", 1), ("Mux torque", 2), [(x + 60, y + 80), (x + 60, y + 55)]
    )
    sch.signal(("Torque", 1), ("Mux torque", 3), [(x + 70, y + 120), (x + 70, y + 65)])
    axes = [
        ("Speed", "Speed (rad/s)"),
        ("Torque", "Torque (Nm)"),
        ("Current", "Current (A)"),
        ("Flux linkage", "Flux linkage (Vs)"),
    ]
    scope = (x + 180, y + 60)
    sch.component("Scope", "Scope", scope, extra=_scope(axes), direction="up")
    # Scope inputs at y + 45, y + 55, y + 65, and y + 75
    sources = [("Mux speed", y + 40), ("Mux torque", y + 60)]
    sources += [("Currents", y + 160), ("Flux (ctrl)", y + 200)]
    for k, (src, y_src) in enumerate(sources):
        x_jog = x + 130 + 5 * k
        points = [(x_jog, y_src), (x_jog, y + 45 + 10 * k)]
        sch.signal((src, 1), ("Scope", k + 1), points)
    sch.dx = dx
    return y + 40 * (len(probes) - 1)
