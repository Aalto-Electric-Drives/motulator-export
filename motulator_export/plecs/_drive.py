"""
Building blocks shared by the drive systems: the mechanics, the speed controller,
and the scope.

"""

from typing import Any, Callable, cast

import numpy as np
from motulator.common.control._pwm import PWM
from motulator.common.model._converter import (
    CapacitiveDCBusConverter,
    FrequencyConverter,
    VoltageSourceConverter,
)
from motulator.common.model._pwm import CarrierComparison
from motulator.common.utils import dead_time_error
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.control._common import SpeedController
from motulator.drive.model import Drive, MechanicalSystem

from motulator_export.plecs._common import (
    MACH,
    ControlBlock,
    MaskParam,
    StepSignal,
    _add_ctrl_output,
    _check_supported_pwm,
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
TAB_PWM = "PWM (PWM)"
PWM_MASK_PARAMS = [
    MaskParam(
        "pwm_t_d",
        "t_d: Dead time (s) in d_err = dead_time_error(i_abc, d_abc, t_d, T_s), "
        "0 = None",
        TAB_PWM,
        "",
        True,
    ),
    MaskParam(
        "pwm_i_0",
        "i_0: Current scale (A) of sign = tanh(i/i_0) in dead_time_error, 0 = np.sign",
        TAB_PWM,
        "",
        True,
    ),
    MaskParam(
        "pwm_feedforward",
        "feedforward: Compensate for d_err (1) or not (0)",
        TAB_PWM,
        "",
        True,
    ),
]


def _check_supported_plant(mdl: Drive, dead_time: bool = False) -> None:
    """
    Raise an error if the mechanics, the converter, or the PWM is not supported.

    The dead time of the carrier comparison (`mdl.pwm.t_d`, by default that of the
    converter) is supported only if `dead_time` is True, and only with the
    current-direction function `np.sign` of the converter.

    """
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
    if mdl.pwm.t_d != 0:
        if not dead_time:
            raise NotImplementedError("Converter dead time not supported")
        if mdl.converter.sign is not np.sign:
            raise NotImplementedError("Only the dead time with sign=np.sign supported")
    if len(mdl.delay.data) != 1:
        raise NotImplementedError("Only the computational delay of one sample")


def _check_supported_speed_control(
    ctrl: VectorControlSystem, dead_time: bool = False
) -> None:
    """
    Raise an error if the speed controller or the PWM is not supported.

    The duty-ratio error model `d_err` of the PWM is supported only if `dead_time` is
    True (see `pwm_values`).

    """
    if not isinstance(ctrl.speed_ctrl, SpeedController):
        raise NotImplementedError("Speed-control mode with SpeedController required")
    if type(ctrl).modulate is not VectorControlSystem.modulate:
        raise NotImplementedError("Overridden modulate method not supported")
    _check_supported_pwm(ctrl.pwm, d_err=dead_time)


def _tanh_sign(i_0: float) -> Callable[[np.ndarray], np.ndarray]:
    """Current-direction function `tanh(i/i_0)`, or `np.sign` if `i_0` is zero."""
    if i_0 == 0:
        return np.sign
    return lambda i: np.tanh(i / i_0)


def pwm_values(pwm: PWM, T_s: float) -> dict[str, Any]:
    """
    Get the mask parameter values of the duty-ratio error model of the PWM.

    Only `d_err = dead_time_error(i_abc, d_abc, t_d, T_s, sign)` with the sampling
    period `T_s` of the control system and `sign = np.sign` (the default) or
    `sign = lambda i: np.tanh(i/i_0)` is supported. Since `d_err` is a function, the
    dead time `t_d` and the current scale `i_0` are identified from its values at the
    duty ratios of 0.5, where `d_err = t_d/(2*T_s)*sign(i)`, and the function is
    checked at test points, also near zero current.

    """
    if pwm.d_err is None:
        return {"pwm_t_d": 0.0, "pwm_i_0": 0.0, "pwm_feedforward": int(pwm.feedforward)}
    d_err = pwm.d_err
    d_half = np.full(3, 0.5)

    def sign(i: float) -> float:
        """Current direction scaled by the error at the duty ratios of 0.5."""
        return float(d_err(np.array([i, -i, 0.0]), d_half)[0])

    # The sign function saturates far from zero current
    t_d = float(f"{2 * T_s * sign(1e6):.12g}")  # Remove the rounding errors
    i_0 = 0.0
    if t_d > 0 and sign(1e-300) < 0.5 * t_d / (2 * T_s):
        # Smooth sign function: bisect the current at which it reaches tanh(1)
        lo, hi = 0.0, 1e6
        target = np.tanh(1) * t_d / (2 * T_s)
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if sign(mid) < target else (lo, mid)
        i_0 = float(f"{0.5 * (lo + hi):.12g}")
    rng = np.random.default_rng(0)
    i_test = rng.uniform(-10, 10, (40, 3))
    i_test[:, 0] = 0  # The signum function is zero at zero current
    # Currents near zero, where the sign functions differ
    i_near = np.array([1e-9, 1e-6, 1e-3, 1e-2, 0.1, 0.3, 1.0])
    if i_0 > 0:
        i_near = np.concatenate((i_near, i_0 * np.array([0.1, 0.5, 1.0, 2.0, 5.0])))
    for k, i in enumerate(i_near):
        i_test[5 + 2 * k, 1:] = [i, -i]
        i_test[6 + 2 * k, 1:] = [-i, 0.5 * i]
    d_test = rng.uniform(0, 1, (40, 3))
    d_test[:5] = [[0, 1, 0.5], [1e-3, 1 - 1e-3, 0.5], [0, 0, 1], [1, 1, 0], [0.5] * 3]
    d_test[5:30, 1:] = 0.5  # Duty ratios at which the error is not suppressed
    for i_abc, d_abc in zip(i_test, d_test, strict=True):
        if t_d <= 0 or not np.allclose(
            d_err(i_abc, d_abc),
            dead_time_error(i_abc, d_abc, t_d, T_s, _tanh_sign(i_0)),
            rtol=0,
            atol=1e-12,
        ):
            raise NotImplementedError(
                "Only d_err = dead_time_error(i_abc, d_abc, t_d, T_s, sign) with "
                "sign=np.sign or sign=lambda i: np.tanh(i/i_0) and the sampling "
                "period T_s of the control system supported"
            )
    return {"pwm_t_d": t_d, "pwm_i_0": i_0, "pwm_feedforward": int(pwm.feedforward)}


def pwm_code(i: dict[str, int]) -> str:
    """C code setting the duty-ratio error model of the PWM from the mask parameters."""
    return (
        "/* Duty-ratio error model of the PWM (dead_time_error) */\n"
        f"pwm_set_dead_time(&ctrl.pwm, P({i['pwm_t_d']}, 0), P({i['T_s']}, 0),\n"
        f"                  P({i['pwm_i_0']}, 0), (int)P({i['pwm_feedforward']}, 0));\n"
    )


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
    t_d = cast(CarrierComparison, mdl.pwm).t_d
    if t_d > 0:
        variables += [("converter.t_d", t_d)]
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
    # Scope, fed by probes of the controller outputs and the machine, below the control
    # system (not shifted by sch.dx, since the probes have no wires)
    dx, sch.dx = sch.dx, 0
    x, y = 80, 240  # Position of the first probe
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
