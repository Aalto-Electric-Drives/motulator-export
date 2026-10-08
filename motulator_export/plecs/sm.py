"""
Export synchronous machine drives to PLECS.

This module converts a motulator synchronous machine drive (a continuous-time system
model and a discrete-time control system) into a PLECS Standalone model. The control
system is a masked subsystem, in which each class of motulator is a block with a
mask of its arguments, in the hierarchy of motulator (see `_sm_control`). The blocks
are C-Script blocks, which include the C port of the motulator control algorithms in
the `c` directory. The derived quantities (e.g., the gains and the lookup tables of
the reference generator) are computed by the C code at the start of the simulation,
as in motulator. The parameters are defined in the initialization commands of the
PLECS model: the system model, and the structs par, cfg, speed_ctrl, and pwm of the
control system, as in the motulator API.

Currently supported:

- Synchronous machine: `SynchronousMachinePars` (the PLECS permanent-magnet
  synchronous machine is used) or `SpatialSaturatedSynchronousMachinePars` with a
  GradNet current map with spatial harmonics (a subsystem that looks like the PLECS
  machine: a C-Script block computes the currents and the torque, which are
  injected with controlled current and torque sources)
- Mechanical system (`MechanicalSystem` without friction), with PLECS blocks
- Converter: carrier comparison (`pwm=True` in `Drive`) with the Symmetrical PWM
  block and the ideal two-level converter of PLECS. The DC bus is stiff
  (`VoltageSourceConverter`), capacitive (`CapacitiveDCBusConverter` without an
  external DC current), or fed by a diode bridge with a DC-bus inductor and
  capacitor (`FrequencyConverter`). The computational delay of one sampling period
  is a PLECS Delay block. The dead time (`t_d` with `sign=np.sign`) is modeled as
  in `motulator_export.plecs.im`.
- Flux-vector control (`FluxVectorController`) in the sensorless or sensored mode,
  with `SynchronousMachinePars` or `SaturatedSynchronousMachinePars` (GradNet flux
  map), and a speed controller (`SpeedController`) in `VectorControlSystem`, with
  the dead-time compensation in the PWM as in `motulator_export.plecs.im`

The PLECS model can be simulated from Python via the XML-RPC interface of PLECS
Standalone (enable it in Preferences > General > RPC interface).

"""

from collections.abc import Callable
from dataclasses import fields
from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.model._converter import FrequencyConverter
from motulator.common.model._pwm import CarrierComparison
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.control._sm_flux_vector import (
    FluxVectorController,
    FluxVectorControllerCfg,
)
from motulator.drive.model import Drive, SynchronousMachine
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SpatialSaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)

from motulator_export.plecs._common import (
    BLANKING_DX,
    C_DIR,
    C_GRADNET_PARAMS,
    C_PARAMS,
    DC_DX,
    ENABLE,
    GRADNET_FIELDS,
    GRADNET_MAX_EMBED_DIM,
    GRADNET_MAX_IN_DIM,
    MACH,
    SRC,
    StepSignal,
    _add_blanking_time,
    _add_control_system,
    _add_converter,
    _add_dc_bus,
    _add_delay,
    _add_pwm,
    _write_model,
)
from motulator_export.plecs._control import INIT_COMMENT, VALUES, pwm_values
from motulator_export.plecs._drive import (
    MACHINE_FRAME,
    MACHINE_PROBES,
    MACHINE_TERMINALS,
    MDL_OUTPUTS,
    _add_drive_outputs,
    _add_mechanics,
    _check_supported_plant,
    _check_supported_speed_control,
    mechanics_and_converter_variables,
    speed_ctrl_values,
)
from motulator_export.plecs._rpc import simulate_plecs
from motulator_export.plecs._schematic import (
    Tap,
    _cscript,
    _inner_schematic,
    _mask,
    _mask_probes,
    _probe,
    _Schematic,
    _terminals,
)
from motulator_export.plecs._sm_control import FVC_BLOCK, init_script


# %%
def _fit_speed_dependent_gain(k: Callable[[float], float], name: str) -> list[float]:
    """Express the gain as k(w_m) = k0 + k1*abs(w_m), checking the form."""
    k0 = float(k(0.0))
    k1 = (float(k(100.0)) - k0) / 100.0
    for w_m in (-300.0, -50.0, 50.0, 300.0, 1e3):
        if not np.isclose(float(k(w_m)), k0 + k1 * abs(w_m), rtol=1e-12, atol=1e-12):
            raise NotImplementedError(f"Only gains k0 + k1*abs(w_m) supported: {name}")
    return [k0, k1]


def export_gradnet(net_map: Any) -> dict[str, Any]:
    """
    Get the parameters of a GradNet map (`FluxMap` or `CurrentMap`) for the C port.

    Single-module GradNets with the PNormGradient or Softmax activations are
    supported, including the current map with spatial harmonics
    (`CurrentMapWithHarmonics`). The network must fit in the arrays of the C port
    (`GRADNET_MAX_IN_DIM` and `GRADNET_MAX_EMBED_DIM` in `c/gradnet.h`).

    """
    import motulator.drive.gradnet as gn  # noqa: PLC0415 (loads PyTorch)

    model = net_map.model
    if model.num_modules != 1 or model.in_dim not in (2, 4):
        raise NotImplementedError("Only single-module GradNets with 2D or 4D inputs")
    block = model.blocks[0]
    embed_dim = len(block.b)
    if model.in_dim > GRADNET_MAX_IN_DIM or embed_dim > GRADNET_MAX_EMBED_DIM:
        raise NotImplementedError(
            f"GradNet too large for the C port: embed_dim={embed_dim} (at most "
            f"{GRADNET_MAX_EMBED_DIM}), in_dim={model.in_dim} (at most "
            f"{GRADNET_MAX_IN_DIM})"
        )
    act = block.act
    if isinstance(act, gn.PNormGradient):
        activation, p = 1, act.q + 1
    elif isinstance(act, gn.Softmax):
        activation, p = 2, 0
    else:
        raise NotImplementedError(f"Activation not supported: {type(act).__name__}")

    def arr(t: Any) -> np.ndarray:
        return t.detach().cpu().numpy().astype(float)

    harmonics = isinstance(net_map, gn.CurrentMapWithHarmonics)
    return {
        "in_dim": model.in_dim,
        "mu_dim": model.in_dim - model.non_mu_dim,
        "k": net_map.k if harmonics else 0,
        "W": arr(block.W),  # (embed_dim, in_dim)
        "b": arr(block.b),
        "mu_log": arr(model.mu_log),
        "bias": arr(model.bias),
        "activation": activation,
        "beta_log": float(act.beta_log.item()),
        "p": p,
        "in_base": float(net_map.psi_base if harmonics else net_map.in_base),
        "out_base": float(net_map.i_base if harmonics else net_map.out_base),
    }


def _has_gradnet_plant(mdl: Drive) -> bool:
    """Whether the machine model is a GradNet current map with spatial harmonics."""
    return isinstance(mdl.machine.par, SpatialSaturatedSynchronousMachinePars)


def _check_supported(mdl: Drive, ctrl: VectorControlSystem) -> None:
    """Raise an error if the drive system is not supported."""
    # System model
    par = mdl.machine.par
    if not isinstance(mdl.machine, SynchronousMachine) or not isinstance(
        par, (SynchronousMachinePars, SpatialSaturatedSynchronousMachinePars)
    ):
        raise NotImplementedError(
            "Only SynchronousMachinePars and SpatialSaturatedSynchronousMachinePars"
        )
    if isinstance(par, SpatialSaturatedSynchronousMachinePars):
        export_gradnet(par.magnetic_map_fcn)  # Raises if not supported
    if par.G_c != 0:
        raise NotImplementedError("Core losses not supported")
    _check_supported_plant(mdl, dead_time=True)
    _check_supported_control(ctrl)


def _check_supported_control(ctrl: VectorControlSystem) -> None:
    """Raise an error if the control system is not supported."""
    fvc = ctrl.vector_ctrl
    if not isinstance(fvc, FluxVectorController):
        raise NotImplementedError("Only FluxVectorController supported")
    if fvc.cfg.online_ref or fvc.cfg.k_f is not None:
        raise NotImplementedError("Only offline references and k_f=None supported")
    if isinstance(fvc.par, SaturatedSynchronousMachinePars):
        if fvc.par.psi_s_dq_fcn is None:
            raise NotImplementedError("A flux map (psi_s_dq_fcn) is required")
        export_gradnet(fvc.par.psi_s_dq_fcn)  # Raises if not supported
    elif not isinstance(fvc.par, SynchronousMachinePars):
        raise NotImplementedError("Machine model of the control system not supported")
    _check_supported_speed_control(ctrl, dead_time=True)


def export_values(
    ctrl: VectorControlSystem, speed_ctrl_args: dict[str, float]
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """
    Get the parameter values of the control system in the motulator API.

    Parameters
    ----------
    ctrl : VectorControlSystem
        Discrete-time control system.
    speed_ctrl_args : dict[str, float]
        Arguments of `SpeedController` used in `ctrl` (the controller stores only the
        resulting gains). They are checked against the gains of `ctrl`.

    Returns
    -------
    values : dict[str, Any]
        Parameter values, None for the defaults (see `_sm_control.init_script`).
    flux_map : dict[str, Any] | None
        GradNet flux map of the control system, None if constant inductances are used.

    """
    fvc = cast(FluxVectorController, ctrl.vector_ctrl)
    cfg: FluxVectorControllerCfg = fvc.cfg

    # alpha_o is resolved in FluxVectorControllerCfg.__post_init__
    default = FluxVectorControllerCfg(
        **{
            f.name: getattr(cfg, f.name)
            for f in fields(cfg)
            if f.name not in ("alpha_o",) and f.init
        }
    )
    alpha_o = None if cfg.alpha_o == default.alpha_o else cfg.alpha_o
    k_o = None if cfg.k_o is None else _fit_speed_dependent_gain(cfg.k_o, "k_o")

    # Machine model of the control system
    flux_map = None
    if isinstance(fvc.par, SaturatedSynchronousMachinePars):
        flux_map = export_gradnet(fvc.par.psi_s_dq_fcn)
        machine = {"n_p": fvc.par.n_p, "R_s": fvc.par.R_s, "L_d": None, "L_q": None}
        machine |= {"psi_f": None, "psi_s_dq_fcn": "est_flux_map"}
    else:
        par = cast(SynchronousMachinePars, fvc.par)
        machine = {"n_p": par.n_p, "R_s": par.R_s, "L_d": par.L_d, "L_q": par.L_q}
        machine |= {"psi_f": par.psi_f, "psi_s_dq_fcn": None}

    values: dict[str, Any] = {
        **machine,
        "i_s_max": cfg.i_s_max,
        "alpha_tau": cfg.alpha_tau,
        "alpha_psi": cfg.alpha_psi,
        "alpha_i": cfg.alpha_i,
        "alpha_o": alpha_o,
        "k_o": k_o,
        "psi_s_min": cfg.psi_s_min,
        "psi_s_max": cfg.psi_s_max,
        "k_u": cfg.k_u,
        "k_mtpv": cfg.k_mtpv,
        "J": cfg.J,
        "sensorless": int(cfg.sensorless),
        "T_s": cfg.T_s,
        **speed_ctrl_values(ctrl, speed_ctrl_args),
        **pwm_values(ctrl.pwm, cfg.T_s),
    }
    return values, flux_map


def export_plant_variables(mdl: Drive) -> list[tuple[str, Any]]:
    """Get the workspace variables of the system model."""
    variables: list[tuple[str, Any]] = []
    if _has_gradnet_plant(mdl):
        par = cast(SpatialSaturatedSynchronousMachinePars, mdl.machine.par)
        g = export_gradnet(par.magnetic_map_fcn)
        variables += [("machine.n_p", par.n_p), ("machine.R_s", par.R_s)]
        variables += [("machine.k", g["k"])]
        variables += [(f"machine.current_map.{f}", g[f]) for f in GRADNET_FIELDS]
    else:
        par = cast(SynchronousMachinePars, mdl.machine.par)
        variables += [
            ("machine.n_p", par.n_p),
            ("machine.R_s", par.R_s),
            ("machine.L_d", par.L_d),
            ("machine.L_q", par.L_q),
            ("machine.psi_f", par.psi_f),
        ]
    return variables + mechanics_and_converter_variables(mdl)


def _machine_cscript_code() -> dict[str, str]:
    """Generate the code sections of the C-Script block of the GradNet machine."""
    declarations = (
        "/* Generated by motulator_export.plecs.sm. The magnetic model is in the\n"
        " * included C files. The parameters come from the initialization\n"
        " * commands. */\n"
        f'#include "{C_DIR}/common.c"\n'
        f'#include "{C_DIR}/gradnet.c"\n'
        f'#include "{C_DIR}/sm_machine.c"\n'
        "\n" + C_PARAMS + "\n" + C_GRADNET_PARAMS + "\n"
        "static SpatialSaturatedSynchronousMachinePars par;\n"
        "\n"
        "/* Stator flux linkage in rotor coordinates */\n"
        "#define PSI_D ContState(0)\n"
        "#define PSI_Q ContState(1)\n"
    )
    start = (
        "GradNet current_map;\n"
        "CHECK_GRADNET(3);\n"
        "READ_GRADNET(current_map, 3);\n"
        "par = spatial_saturated_synchronous_machine_pars(\n"
        "    P(0, 0), P(1, 0), &current_map, (int)P(2, 0));\n"
        "if (isnan(par.psi_f)) {\n"
        '    SetErrorMessage("The PM-flux linkage cannot be solved from the current "\n'
        '                    "map.");\n'
        "    return;\n"
        "}\n"
        "PSI_D = par.psi_f; /* Initial states as in motulator */\n"
        "PSI_Q = 0.0;\n"
    )
    output = (
        "/* Inputs: voltages -u_ac and -u_bc, rotor angle, and rotor speed */\n"
        "double theta_M = InputSignal(1, 0), w_M = InputSignal(2, 0);\n"
        "double complex i_s_dq;\n"
        "double tau_M;\n"
        "double theta_m = par.n_p * theta_M;\n"
        "machine_magnetic_map(&par, PSI_D + I * PSI_Q, theta_m, &i_s_dq, &tau_M);\n"
        "double i_s_abc[3];\n"
        "complex2abc(i_s_dq * cexp(I * theta_m), i_s_abc);\n"
        "/* Phase currents a and b to the current sources, phase c is the return */\n"
        "OutputSignal(0, 0) = i_s_abc[0];\n"
        "OutputSignal(0, 1) = i_s_abc[1];\n"
        "for (int k = 0; k < 3; k++) {\n"
        "    OutputSignal(1, k) = i_s_abc[k];\n"
        "}\n"
        "OutputSignal(2, 0) = tau_M;\n"
        "OutputSignal(3, 0) = w_M;\n"
        "OutputSignal(4, 0) = theta_M;\n"
    )
    derivative = (
        "/* Stator voltage from the line voltages u_ac and u_bc, which are the\n"
        " * negated voltages of the current sources (zero sequence is irrelevant) */\n"
        "double complex u_s_ab =\n"
        "    -2.0 / 3.0 * (InputSignal(0, 0) + cexp(I * 2.0 * M_PI / 3.0) * "
        "InputSignal(0, 1));\n"
        "double complex d_psi_s_dq = machine_flux_rhs(\n"
        "    &par, u_s_ab, PSI_D + I * PSI_Q, InputSignal(1, 0), InputSignal(2, 0));\n"
        "ContDeriv(0) = creal(d_psi_s_dq);\n"
        "ContDeriv(1) = cimag(d_psi_s_dq);\n"
    )
    return {
        "Declarations": declarations,
        "StartFcn": start,
        "OutputFcn": output,
        "DerivativeFcn": derivative,
    }


# Icon of the PLECS permanent-magnet synchronous machine, and the terminals (phases
# a, b, and c, and the rotational flange)
PMSM_ICON = (
    "Icon:circle(0, 0, 25)\n"
    "Icon:line({-25, -23}, {10, 10})\n"
    "Icon:line({-25, -23}, {-10, -10})\n"
    "Icon:line({-18, -25, -25, 25, 25, 18}, {18, 25, 35, 35, 25, 18})\n"
    "Icon:circle(-26.5, -13.5, .5)\n"
    "Icon:line({-5, -5, 5, 5, -5, -5, 5}, {-8, -14, -14, 14, 14, -8, -8})"
)


def _add_pmsm(sch: _Schematic) -> None:
    """Add the PLECS permanent-magnet synchronous machine (including the inertia)."""
    machine = {
        "configuration": "1",
        "R": "machine.R_s",
        "L": "[machine.L_d machine.L_q]",
        "phi": "machine.psi_f",
        "J": "mechanics.J",
        "F": "0",
        "p": "machine.n_p",
        "wm0": "0",
        "thm0": "0",
        "is0": "[0 0]",
    }
    sch.component(
        "Reference",
        "Machine",
        MACH,
        machine,
        direction="up",
        label="east",
        src_component="Components/Electrical/Machines/Perm.-Magnet SM",
        extra=MACHINE_FRAME,
        trailer=_terminals(MACHINE_TERMINALS),
    )


def _add_gradnet_machine(sch: _Schematic) -> None:
    """
    Add the machine with a GradNet current map as a subsystem.

    The subsystem looks like the PLECS permanent-magnet synchronous machine. Inside,
    a C-Script block computes the phase currents and the torque from the flux
    linkage (its state), the rotor angle, and the speed. The currents are injected
    with controlled current sources, and the torque acts on the rotational flange.

    """
    sub = _Schematic()
    for k, ph in enumerate("abc"):
        sub.component("Port", ph, (40, 60 + 40 * k), {"Index": str(k + 1)})
    # Current sources of phases a and b, returning via phase c
    for ph, x, y in (("b", 200, 100), ("a", 280, 60)):
        sub.component("CurrentSource", f"i_{ph}", (x, 140), SRC, direction="up")
        sub.wire((ph, 1), (f"i_{ph}", 1), [(x, y)])
    taps: list[Tap] = [([(140, 140), (140, 200), (200, 200)], [(("i_b", 2), [])])]
    sub.bus(("c", 1), "Wire", taps + [([(280, 200)], [(("i_a", 2), [])])])
    # Magnetic model: the phase voltages from the sources (probe), the rotor angle and
    # speed from the sensors; the currents to the sources, the torque to the flange
    parameters = ["machine.n_p", "machine.R_s", "machine.k"]
    parameters += [f"machine.current_map.{f}" for f in GRADNET_FIELDS]
    cscript = _cscript(
        _machine_cscript_code(),
        "[2 1 1]",
        "[2 3 1 1 1]",
        ", ".join(parameters),
        "0",
        num_cont_states=2,
        feedthrough="[0 1 1]",
    )
    sub.component(
        "CScript",
        "C-Script",
        (480, 160),
        cscript,
        direction="up",
        extra="      Frame         [-50, -40; 50, 40]\n",
    )
    probes = "".join(_probe(f"i_{ph}", ["Source voltage"], "Machine") for ph in "ab")
    sub.component("PlecsProbe", "u", (360, 150), extra=probes)
    sub.signal(("u", 1), ("C-Script", 1))
    sub.component("SignalDemux", "Demux", (500, 40), {"Width": "[1 1]"}, show=False)
    sub.signal(("C-Script", 4), ("Demux", 1), [(550, 140), (550, 40)])
    sub.signal(("Demux", 2), ("i_a", 3), [(305, 35)])
    sub.signal(("Demux", 3), ("i_b", 3), [(225, 45)])
    # Mechanics: the torque source and the sensors on the flange
    sub.component("RotationalPort", "flange", (780, 300), {"Index": "4"}, flipped=True)
    sub.component(
        "ControlledTorque",
        "Torque",
        (600, 230),
        {"SecondFlange": "2", "StateSpaceInlining": "2"},
        direction="left",
        label="north",
    )
    sub.component(
        "RotationalReference", "Frame", (605, 190), direction="left", show=False
    )
    sensor = {"SecondFlange": "1"}
    sub.component("RotationalSpeedSensor", "w_M", (660, 230), sensor, direction="left")
    sensor = {"SecondFlange": "1", "theta0": "0"}
    sub.component("AngleSensor", "theta_M", (720, 230), sensor, direction="left")
    taps = [([(720, 300)], [(("theta_M", 1), [])]), ([(660, 300)], [(("w_M", 1), [])])]
    sub.bus(("flange", 1), "Rotational", taps + [([(600, 300)], [(("Torque", 1), [])])])
    sub.connect(("Frame", 1), ("Torque", 3), "Rotational")
    sub.signal(("C-Script", 6), ("Torque", 2), [(560, 160), (560, 230)])
    sub.signal(
        ("theta_M", 2),
        ("C-Script", 2),
        [(690, 230), (690, 290), (395, 290), (395, 160)],
    )
    sub.signal(
        ("w_M", 2), ("C-Script", 3), [(630, 230), (630, 280), (405, 280), (405, 170)]
    )
    header = MACHINE_FRAME + _mask(
        "GradNet synchronous machine (motulator)",
        "Synchronous machine with a GradNet current map with spatial harmonics, "
        "corresponding to SpatialSaturatedSynchronousMachinePars in motulator. The "
        "stator flux linkage in rotor coordinates is the state variable. The "
        "parameters are in the variable 'machine' of the initialization commands.",
        PMSM_ICON,
    )
    probes_out = [
        ("Stator phase currents", "C-Script", "Output 2"),
        ("Rotational speed", "C-Script", "Output 4"),
        ("Rotor position", "C-Script", "Output 5"),
        ("Electrical torque", "C-Script", "Output 3"),
    ]
    trailer = (
        _terminals(MACHINE_TERMINALS)
        + _inner_schematic(sub, (820, 360))
        + _mask_probes(probes_out)
    )
    sch.component(
        "Subsystem",
        "Machine",
        MACH,
        direction="up",
        label="east",
        extra=header,
        trailer=trailer,
    )


def write_model(
    path: str | Path,
    mdl: Drive,
    ctrl: VectorControlSystem,
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    speed_ctrl_args: dict[str, float],
    outputs: bool = False,
    enable: StepSignal | float = 1.0,
) -> Path:
    """
    Write a PLECS model of the drive system.

    Parameters
    ----------
    path : str | Path
        Path of the model file (.plecs). The C sources are expected in the `c`
        directory of the package, referred to by a path relative to the model file.
    mdl : Drive
        Continuous-time system model.
    ctrl : VectorControlSystem
        Discrete-time control system.
    w_M_ref : StepSignal
        Speed reference (mechanical rad/s).
    tau_L : StepSignal
        External load torque (Nm).
    t_stop : float
        Simulation stop time (s).
    speed_ctrl_args : dict[str, float]
        Arguments of `SpeedController` used in `ctrl`, e.g., ``{"J": 0.015,
        "alpha_s": 25}``.
    outputs : bool, optional
        Add the output ports "mdl" and "ctrl" for `simulate`, defaults to False.
    enable : StepSignal | float, optional
        Input `enable` of the control system, defaults to 1 (enabled). While it is
        not positive, the duty ratios are 0.5 and the state of the control system is
        reset to its initial value.

    Returns
    -------
    Path
        Path of the written model file.

    """
    path = Path(path)
    _check_supported(mdl, ctrl)
    values, flux_map = export_values(ctrl, speed_ctrl_args)
    variables = export_plant_variables(mdl)
    if flux_map is not None:
        variables += [(f"est_flux_map.{f}", flux_map[f]) for f in GRADNET_FIELDS]
    init = init_script(variables, values, INIT_COMMENT)
    block, T_s = FVC_BLOCK, VALUES["T_s"]  # The model refers to cfg.T_s
    sch = _Schematic()
    gradnet_plant = _has_gradnet_plant(mdl)
    sources: list[tuple[str, StepSignal | float | str]] = [
        (ENABLE, enable),
        ("w_M_ref", w_M_ref),
        ("i_s_abc", _probe("Machine", MACHINE_PROBES[:1])),
        ("u_dc meas.", _probe("u_dc", ["Measured voltage"])),
        ("theta_M", _probe("Machine", MACHINE_PROBES[2:3])),
    ]
    _add_control_system(sch, block, sources, VALUES, init)
    _add_delay(sch, T_s, block)
    _add_pwm(sch, T_s)
    t_d = cast(CarrierComparison, mdl.pwm).t_d
    if t_d > 0:
        _add_blanking_time(sch)
    # The blanking time, the diode bridge, and its grid need space between the PWM and
    # the DC bus
    sch.dx = 320 if isinstance(mdl.converter, FrequencyConverter) else 0
    sch.dx += BLANKING_DX if t_d > 0 else 0
    sch.dx += DC_DX
    _add_converter(sch, t_d)
    _add_dc_bus(sch, mdl.converter)
    if gradnet_plant:
        _add_gradnet_machine(sch)
    else:
        _add_pmsm(sch)
    for k in range(3):
        sch.wire(("Converter", k + 1), ("Machine", k + 1))
    _add_mechanics(sch, tau_L, inertia=gradnet_plant)
    _add_drive_outputs(sch, block, outputs)
    size = (880 + sch.dx, 520)
    return _write_model(path, init, t_stop, T_s, sch, size, outputs)


def simulate(
    path: str | Path, t_eval: np.ndarray, model_vars: dict[str, float] | None = None
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the PLECS model of a synchronous machine drive.

    The model must have the output ports (`write_model` with ``outputs=True``).

    Parameters
    ----------
    path : str | Path
        Path of the model file.
    t_eval : ndarray
        Output times (s).
    model_vars : dict[str, float], optional
        Workspace variables to be overridden.

    Returns
    -------
    mdl : dict[str, ndarray]
        Machine signals (`MDL_OUTPUTS`) and the time "t".
    ctrl : dict[str, ndarray]
        Controller signals and the time "t".

    """
    return simulate_plecs(
        path,
        t_eval,
        model_vars,
        mdl_outputs=MDL_OUTPUTS,
        ctrl_outputs=FVC_BLOCK.signals,
    )
