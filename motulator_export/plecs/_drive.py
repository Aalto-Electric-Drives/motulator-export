"""
Building blocks shared by the drive systems: the mechanics, the speed controller,
and the scope.

"""

from typing import Any, cast

from motulator.common.control._pwm import PWM
from motulator.common.model._converter import (
    CapacitiveDCBusConverter,
    FrequencyConverter,
    VoltageSourceConverter,
)
from motulator.common.model._pwm import CarrierComparison
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.control._common import SpeedController
from motulator.drive.model import Drive, MechanicalSystem

from motulator_export.plecs._common import (
    MACH,
    ControlBlock,
    MaskParam,
    StepSignal,
    _add_ctrl_output,
    _step,
)
from motulator_export.plecs._schematic import _probe, _Schematic, _scope

# Machine signals in the output port "mdl"
MDL_OUTPUTS = ["i_a", "i_b", "i_c", "w_M", "theta_M", "tau_M"]
TAB_SPEED = "Speed control (SpeedController)"
SPEED_MASK_PARAMS = [
    MaskParam("speed_J", "J: Total inertia (kgm²)", TAB_SPEED, "", True),
    MaskParam(
        "speed_alpha_s",
        "alpha_s: Reference-tracking bandwidth (rad/s)",
        TAB_SPEED,
        "",
        True,
    ),
    MaskParam(
        "speed_alpha_i",
        "alpha_i: Integral-action bandwidth (rad/s), [] = alpha_s",
        TAB_SPEED,
        "",
    ),
    MaskParam("speed_tau_M_max", "tau_M_max: Maximum motor torque (Nm)", TAB_SPEED, ""),
]


def _check_supported_plant(mdl: Drive) -> None:
    """Raise an error if the mechanics, the converter, or the PWM is not supported."""
    if not isinstance(mdl.mechanics, MechanicalSystem) or mdl.mechanics.B_L != 0:
        raise NotImplementedError("Only MechanicalSystem without friction supported")
    if mdl.lc_filter is not None:
        raise NotImplementedError("LC filter not supported")
    if type(mdl.converter) not in (
        VoltageSourceConverter,
        CapacitiveDCBusConverter,
        FrequencyConverter,
    ):
        raise NotImplementedError("Converter not supported")
    if mdl.converter.inp.i_dc is not None:
        raise NotImplementedError("External DC current not supported")
    if not isinstance(mdl.pwm, CarrierComparison):
        raise NotImplementedError("Only CarrierComparison (pwm=True) supported")
    if len(mdl.delay.data) != 1:
        raise NotImplementedError("Only the computational delay of one sample")


def _check_supported_speed_control(ctrl: VectorControlSystem) -> None:
    """Raise an error if the speed controller or the PWM is not supported."""
    if not isinstance(ctrl.speed_ctrl, SpeedController):
        raise NotImplementedError("Speed-control mode with SpeedController required")
    if not isinstance(ctrl.pwm, PWM) or ctrl.pwm.overmodulation != "MPE":
        raise NotImplementedError("Only the MPE overmodulation supported")
    if ctrl.pwm.k_comp != 1.5:
        raise NotImplementedError("Only k_comp = 1.5 supported")


def speed_ctrl_values(
    ctrl: VectorControlSystem, speed_ctrl_args: dict[str, float]
) -> dict[str, Any]:
    """
    Get the mask parameter values of the speed controller.

    The speed controller stores only the resulting gains, so the arguments given to
    `SpeedController` are needed. They are checked against the gains of `ctrl`.

    """
    speed_ctrl = cast(SpeedController, ctrl.speed_ctrl)
    ref = SpeedController(**speed_ctrl_args)
    for attr in ("k_p", "k_t", "alpha_i", "u_max"):
        if getattr(ref, attr) != getattr(speed_ctrl, attr):
            raise ValueError(f"speed_ctrl_args do not match the controller: {attr}")
    return {
        "speed_J": speed_ctrl_args["J"],
        "speed_alpha_s": speed_ctrl_args["alpha_s"],
        "speed_alpha_i": speed_ctrl_args.get("alpha_i"),
        "speed_tau_M_max": speed_ctrl_args.get("tau_M_max", float("inf")),
    }


def mechanics_and_converter_variables(mdl: Drive) -> list[tuple[str, Any]]:
    """Get the workspace variables of the mechanics and the converter."""
    conv = mdl.converter
    variables: list[tuple[str, Any]] = [
        ("mechanics.J", cast(MechanicalSystem, mdl.mechanics).J),
        ("converter.u_dc", conv.u_dc),
    ]
    if isinstance(conv, (CapacitiveDCBusConverter, FrequencyConverter)):
        variables += [("converter.C_dc", conv.C_dc)]
    if isinstance(conv, FrequencyConverter):
        variables += [("converter.L_dc", conv.L_dc)]
        variables += [("converter.u_g", conv.u_g), ("converter.w_g", conv.w_g)]
    return variables


def speed_controller_code(i: dict[str, int]) -> str:
    """C code creating the speed controller from the mask parameters."""
    tau_M_max = i["speed_tau_M_max"]
    return (
        "/* Speed controller (SpeedController) */\n"
        "PIController speed_ctrl = speed_controller(\n"
        f"    P({i['speed_J']}, 0), P({i['speed_alpha_s']}, 0), "
        f"PARAM({i['speed_alpha_i']}),\n"
        f"    PDIM({tau_M_max}) > 0 ? P({tau_M_max}, 0) : INFINITY);\n"
    )


MACHINE_FRAME = "      Frame         [-25, -25; 25, 35]\n"
MACHINE_TERMINALS = [
    ("Port", -30, -10, "left"),
    ("Port", -30, 0, "left"),
    ("Port", -30, 10, "left"),
    ("Rotational", 30, 30, "right"),
]


def _add_mechanics(sch: _Schematic, tau_L: StepSignal, inertia: bool) -> None:
    """Add the load torque (and the inertia, if not included in the machine)."""
    x_f, y_f = MACH[0] + 30, MACH[1] + 30  # Flange of the machine
    y_load = y_f + 90 if inertia else y_f + 50
    if inertia:
        sch.component(
            "Inertia",
            "Inertia",
            (x_f, y_f + 30),
            {"J": "mechanics.J", "w0": "0", "theta0": "0"},
            label="east",
        )
        sch.connect(("Machine", 4), ("Inertia", 1), "Rotational")
        sch.connect(("Inertia", 2), ("Load", 3), "Rotational")
    else:
        sch.connect(("Machine", 4), ("Load", 3), "Rotational")
    sch.component("Step", "tau_L", (x_f - 70, y_load), _step(tau_L))
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
    sch.signal(("tau_L", 1), ("Load", 2))


# Probe signals of the PLECS machines: (current, speed, angle, torque)
MACHINE_PROBES = ["Stator phase currents", "Rotational speed", "Rotor position"]
MACHINE_PROBES += ["Electrical torque"]


def _add_drive_outputs(sch: _Schematic, block: ControlBlock, outputs: bool) -> None:
    """
    Add the scope and its probes, and optionally the output ports.

    The first three monitored signal groups of the block (the speed, the torque, and
    the flux linkage) are shown in the scope with the machine signals.

    """
    speed, torque, flux = list(block.outputs)[:3]
    if outputs:  # Output ports for scripted simulations: machine and controller signals
        probe = _probe("Machine", MACHINE_PROBES)
        sch.component("PlecsProbe", "Machine signals", (400, 300), extra=probe)
        sch.component("Output", "mdl", (480, 300), {"Index": "1", "Width": "-1"})
        sch.signal(("Machine signals", 1), ("mdl", 1))
        _add_ctrl_output(sch, block)
    # Scope, fed by probes of the controller outputs and the machine
    probes = [
        ("Speed (ctrl)", block.name, [speed], 300),
        ("Speed", "Machine", [MACHINE_PROBES[1]], 340),
        ("Torque (ctrl)", block.name, [torque], 380),
        ("Torque", "Machine", [MACHINE_PROBES[3]], 420),
        ("Currents", "Machine", [MACHINE_PROBES[0]], 460),
        ("Flux (ctrl)", block.name, [flux], 500),
    ]
    for name, comp, signals, y in probes:
        extra = _probe(comp, signals)
        sch.component("PlecsProbe", name, (720, y), extra=extra)
    sch.component("SignalMux", "Mux speed", (820, 340), {"Width": "[2 1]"}, show=False)
    sch.component("SignalMux", "Mux torque", (820, 360), {"Width": "[2 1]"}, show=False)
    sch.signal(("Speed (ctrl)", 1), ("Mux speed", 2), [(760, 300), (760, 335)])
    sch.signal(("Speed", 1), ("Mux speed", 3), [(770, 340), (770, 345)])
    sch.signal(("Torque (ctrl)", 1), ("Mux torque", 2), [(780, 380), (780, 355)])
    sch.signal(("Torque", 1), ("Mux torque", 3), [(790, 420), (790, 365)])
    axes = [
        ("Speed", "Speed (rad/s)"),
        ("Torque", "Torque (Nm)"),
        ("Current", "Current (A)"),
        ("Flux linkage", "Flux linkage (Vs)"),
    ]
    sch.component("Scope", "Scope", (900, 360), extra=_scope(axes), direction="up")
    sch.signal(("Mux speed", 1), ("Scope", 1), [(850, 340), (850, 345)])
    sch.signal(("Mux torque", 1), ("Scope", 2), [(855, 360), (855, 355)])
    sch.signal(("Currents", 1), ("Scope", 3), [(860, 460), (860, 365)])
    sch.signal(("Flux (ctrl)", 1), ("Scope", 4), [(865, 500), (865, 375)])
