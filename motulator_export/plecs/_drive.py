"""
Building blocks shared by the drive systems: the mechanics, the speed controller,
and the scope.

"""

from typing import Any, cast

import numpy as np
from motulator.common.model._converter import (
    DCBus,
    DiodeBridgeDCBus,
    DiodeBridgeDCBusStates,
    VoltageSourceConverter,
)
from motulator.common.model._pwm import CarrierComparison
from motulator.common.utils._utils import wrap
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._common import SpeedController
from motulator.drive.model import Drive, MechanicalSystem, SynchronousMachine

from motulator_export.plecs._common import (
    CONV,
    MACH,
    ControlBlock,
    StepSignal,
    _add_ctrl_output,
    _add_step,
    _check_delay,
    _check_initial_states,
    _check_supported_pwm,
    reference,
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
    if not isinstance(mdl.mechanics, MechanicalSystem) or mdl.mechanics.B_L != 0:
        raise NotImplementedError("Only MechanicalSystem without friction supported")
    if mdl.lc_filter is not None:
        raise NotImplementedError("LC filter not supported")
    if type(mdl.converter) is not VoltageSourceConverter:
        raise NotImplementedError("Only VoltageSourceConverter supported")
    if mdl.dc_bus is not None:
        _check_supported_dc_bus(mdl.dc_bus, mdl.converter)
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


def _check_supported_dc_bus(dc_bus: DCBus, converter: VoltageSourceConverter) -> None:
    """
    Raise an error if the DC bus is not supported: it must feed only the converter of
    the drive (e.g., not an active front end), without an external DC current.
    """
    if type(dc_bus) not in (DCBus, DiodeBridgeDCBus):
        raise NotImplementedError("Only DCBus and DiodeBridgeDCBus supported")
    if dc_bus.converters != [converter]:
        raise NotImplementedError("DC bus shared with other converters not supported")
    if dc_bus.inp.i_dc is not None:
        raise NotImplementedError("External DC current not supported")


def _check_initial_drive_states(mdl: Drive) -> None:
    """
    Raise an error if a state of the drive (other than the speed and the angle of
    the mechanics and the DC-bus voltage, which are exported) is not at its initial
    value. The synchronous machine model has its own rotor angle, which must be n_p
    times that of the mechanics (modulo 2*pi), as in the exported models.
    """
    machine = mdl.machine
    mech = cast(MechanicalSystem, mdl.mechanics).state
    exported = [
        ("mechanics.state.w_M", mech.w_M),
        ("mechanics.state.theta_M", mech.theta_M),
    ]
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
    Raise an error if the speed controller or the PWM is not supported.

    The duty-ratio error model `d_err` of the PWM is supported only if `dead_time` is
    True (see `_control.pwm_values`).

    """
    if not isinstance(ctrl.speed_ctrl, SpeedController):
        raise NotImplementedError("Speed-control mode with SpeedController required")
    if type(ctrl).modulate is not VectorControlSystem.modulate:
        raise NotImplementedError("Overridden modulate method not supported")
    _check_supported_pwm(ctrl, d_err=dead_time)


def speed_ctrl_values(ctrl: VectorControlSystem) -> dict[str, Any]:
    """
    Get the parameter values of the speed controller (the struct speed_ctrl of the
    model initialization): the arguments of `SpeedController`, whose gains must be
    those of the arguments (the exported controller computes its gains from them).

    The controller stores the integral action bandwidth as the ratio of its gains,
    which may differ from the argument by a rounding error. The default (None, i.e.,
    `alpha_s`) is exported as None if it gives the gains of the controller, so that
    the exported controller computes the same gains as motulator.

    """
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


def drive_references(
    mdl: Drive, ctrl: VectorControlSystem | VHzControlSystem, T_s: float
) -> tuple[StepSignal | float, StepSignal | float]:
    """
    Speed reference and load torque of the drive, read from the control system
    (`set_speed_ref`) and the mechanics (`set_external_load_torque`): constants or
    `StepSignal` objects, the speed reference sampled to switch at the same sampling
    instant as in motulator (the load torque acts in continuous time).
    """
    w_M_ref = reference(ctrl.ext_ref.w_M, "speed reference", T_s)
    tau_L = reference(cast(MechanicalSystem, mdl.mechanics).inp.tau_L, "load torque")
    return w_M_ref, tau_L


def mechanics_and_converter_variables(mdl: Drive) -> list[tuple[str, Any]]:
    """
    Get the workspace variables of the mechanics (with the initial speed and angle
    of its state), the converter, and the DC bus (with the initial voltage of its
    state).
    """
    mech = cast(MechanicalSystem, mdl.mechanics)
    variables: list[tuple[str, Any]] = [
        ("mechanics.J", mech.J),
        ("mechanics.w_M0", float(mech.state.w_M.real)),
        ("mechanics.theta_M0", float(wrap(mech.state.theta_M.real))),
    ]
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


# Space for the voltmeters of the converter voltages between the converter and the
# machine (horizontal shift of the machine and the components after it)
MEAS_DX = 60


def _add_phase_lines(sch: _Schematic, x_c: int) -> None:
    """
    Add the phase lines from the converter (its AC terminals at the absolute `x_c`)
    to the machine (added at the current `sch.dx`), with the voltmeters of the
    line-to-line converter voltages u_ab and u_bc, measured for the control system
    (used while the gates are blocked). The lines a and c are spread around the line
    b so that the voltmeters fit between them, as the PCC voltmeters of the grid
    converters.
    """
    dx = sch.dx
    x_m = MACH[0] - 30  # The terminals of the machine (shifted by sch.dx)
    y_c = CONV[1]
    x_1, x_v, x_2 = x_c + 15 - dx, x_c + 45 - dx, x_m - 15  # Shifted by sch.dx
    sch.component("Voltmeter", "u_cab", (x_v, y_c - 20), direction="up", show=False)
    sch.component("Voltmeter", "u_cbc", (x_v, y_c + 20), direction="up", show=False)
    meters = [[("u_cab", 1)], [("u_cab", 2), ("u_cbc", 1)], [("u_cbc", 2)]]
    for k in range(3):
        y_t, y = y_c - 10 + 10 * k, y_c - 40 + 40 * k  # The terminals and the line
        out = [] if k == 1 else [(x_1, y_t), (x_1, y)]
        back = [] if k == 1 else [(x_2, y), (x_2, y_t)]
        dsts = [(m, []) for m in meters[k]]
        sch.bus(
            ("Converter", k + 1),
            "Wire",
            [(out + [(x_v, y)], dsts), ([], [(("Machine", k + 1), back)])],
        )


# Probes of the line-to-line converter voltages (the input u_c_line)
CONVERTER_VOLTAGES = _probe("u_cab", ["Measured voltage"]) + _probe(
    "u_cbc", ["Measured voltage"]
)

MACHINE_FRAME = "      Frame         [-25, -25; 25, 35]\n"
MACHINE_TERMINALS = [
    ("Port", -30, -10, "left"),
    ("Port", -30, 0, "left"),
    ("Port", -30, 10, "left"),
    ("Rotational", 30, 30, "right"),
]


def _add_mechanics(sch: _Schematic, tau_L: StepSignal | float, inertia: bool) -> None:
    """Add the load torque (and the inertia, if not included in the machine)."""
    x_f, y_f = MACH[0] + 30, MACH[1] + 30  # Flange of the machine
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
    else:
        sch.connect(("Machine", 4), ("Load", 3), "Rotational")
    if isinstance(tau_L, StepSignal):
        # Several steps are summed by a Gain block in front of the load
        x_step = x_f - 70 if np.isscalar(tau_L.time) else x_f - 120
        load_torque = _add_step(sch, "tau_L", tau_L, (x_step, y_load))
    else:
        sch.component("Constant", "tau_L", (x_f - 70, y_load), {"Value": _fmt(tau_L)})
        load_torque = ("tau_L", 1)
    sch.component(
        "ControlledTorque",
        "Load",
        (x_f, y_load),
        {"SecondFlange": "2", "StateSpaceInlining": "2"},
        direction="left",
        label="east",
    )
    sch.component(
        "RotationalReference", "Frame", (x_f, y_load + 45), direction="up", show=False
    )
    sch.connect(("Frame", 1), ("Load", 1), "Rotational")
    sch.signal(load_torque, ("Load", 2))


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
    mux = {"Width": "[2 1]"}
    sch.component("SignalMux", "Mux speed", (x + 100, y + 40), mux, show=False)
    sch.component("SignalMux", "Mux torque", (x + 100, y + 60), mux, show=False)
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
