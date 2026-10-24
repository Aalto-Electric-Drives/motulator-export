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
PLECS model: the system model, and the structs of the control system (vector_ctrl
with par and cfg, speed_ctrl, and pwm), as in the motulator API.

Currently supported:

- Synchronous machine: `SynchronousMachinePars` (the PLECS permanent-magnet
  synchronous machine is used) or `SpatialSaturatedSynchronousMachinePars` with a
  GradNet current map with spatial harmonics (a subsystem that looks like the PLECS
  machine: a C-Script block computes the currents and the torque, which are
  injected with controlled current and torque sources)
- Mechanical system (`MechanicalSystem` without friction), with PLECS blocks
- Converter: carrier comparison (`pwm=True` in `Drive`) with the Symmetrical PWM
  block and the IGBT converter of PLECS with ideal IGBTs and diodes. The DC bus is stiff
  (`VoltageSourceConverter` without a DC-bus model), capacitive (`DCBus`), or fed by
  a diode bridge with a DC-bus inductor and capacitor (`DiodeBridgeDCBus`). The
  DC-bus model feeds only the converter of the drive, without an external DC
  current, and the capacitor voltage starts from the state of the model. The
  computational delay of one sampling period is a PLECS Delay block. The duty
  ratios are updated twice per carrier period, or once (`single_update=True` with
  `delay=2`). The dead time and the threshold voltage (`t_d` and `u_th` with
  `sign=np.sign`) are modeled as in `motulator_export.plecs._im`.
- Flux-vector control (`FluxVectorController`) in the sensorless or sensored mode,
  with `SynchronousMachinePars` or `SaturatedSynchronousMachinePars` (GradNet flux
  map), and a speed controller (`SpeedController`) in `VectorControlSystem`, with
  the compensation of the dead time and the threshold voltage in the PWM as in
  `motulator_export.plecs._im`
- Gate blocking (`set_enable`): the input `enable` of the control system, which
  then runs in standby, and the gates of the IGBT converter blocked (not with the
  GradNet machine model, whose controlled current sources would toggle the diodes
  of a blocked leg)

The PLECS model can be simulated from Python via the XML-RPC interface of PLECS
Standalone (enable it in Preferences > General > RPC interface).

"""

from collections.abc import Callable
from pathlib import Path
from typing import Any, cast

import motulator.drive.gradnet as gn
import numpy as np
from motulator.common.model._converter import DiodeBridgeDCBus
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

from motulator_export._common import StepSignal, gate_enable
from motulator_export.plecs._common import (
    BLANKING_DX,
    C_DIR,
    C_GRADNET_PARAMS,
    C_PARAMS,
    DC_DX,
    ENABLE,
    GATE_DX,
    GRADNET_FIELDS,
    GRADNET_MAX_EMBED_DIM,
    GRADNET_MAX_IN_DIM,
    MACH,
    SRC,
    _add_blanking_time,
    _add_control_system,
    _add_converter,
    _add_dc_bus,
    _add_delay,
    _add_pwm,
    _add_reference,
    _write_model,
)
from motulator_export.plecs._control import INIT_COMMENT, T_S_EXPR, VALUES, pwm_values
from motulator_export.plecs._drive import (
    MACHINE_FRAME,
    MACHINE_PROBES,
    MACHINE_TERMINALS,
    MDL_OUTPUTS,
    _add_drive_outputs,
    _add_mechanics,
    _add_phase_lines,
    _check_supported_plant,
    _check_supported_speed_control,
    drive_references,
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
from motulator_export.plecs._sm_control import (
    CFG_FIELDS,
    FVC_BLOCK,
    PAR_FIELDS,
    init_script,
)

__all__ = ["simulate", "write_model"]


# %%
def fit_speed_dependent_gain(k: Callable[[float], float], name: str) -> list[float]:
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
    fcn = net_map.fcn  # The network as a numpy function (GradNetFunction)
    num_modules, embed_dim, in_dim = fcn.W.shape
    if num_modules != 1 or in_dim not in (2, 4):
        raise NotImplementedError("Only single-module GradNets with 2D or 4D inputs")
    if in_dim > GRADNET_MAX_IN_DIM or embed_dim > GRADNET_MAX_EMBED_DIM:
        raise NotImplementedError(
            f"GradNet too large for the C port: embed_dim={embed_dim} (at most "
            f"{GRADNET_MAX_EMBED_DIM}), in_dim={in_dim} (at most "
            f"{GRADNET_MAX_IN_DIM})"
        )
    activations = {"PNormGradient": 1, "Softmax": 2}  # GradNetActivation in C
    if fcn.activation not in activations:
        raise NotImplementedError(f"Activation not supported: {fcn.activation}")
    return {
        "in_dim": in_dim,
        "mu_dim": fcn.mu_log.size,
        "k": net_map.k if isinstance(net_map, gn.CurrentMapWithHarmonics) else 0,
        "W": fcn.W[0],  # (embed_dim, in_dim)
        "b": fcn.b[0],
        "mu_log": fcn.mu_log,
        "bias": fcn.bias,
        "activation": activations[fcn.activation],
        "beta_log": float(fcn.beta_log[0]),
        "p": fcn.p,
        "in_base": float(net_map.in_base),
        "out_base": float(net_map.out_base),
    }


def has_gradnet_plant(mdl: Drive) -> bool:
    """Whether the machine model is a GradNet current map with spatial harmonics."""
    return isinstance(mdl.machine.par, SpatialSaturatedSynchronousMachinePars)


def check_supported(mdl: Drive, ctrl: VectorControlSystem) -> None:
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
        # The controlled current sources of the machine model would toggle the
        # diodes of a blocked leg at the zero crossings of its current
        if gate_enable(ctrl) != 1:
            raise NotImplementedError(
                "Gate blocking (set_enable) not supported with the GradNet machine "
                "model"
            )
    if par.G_c != 0:
        raise NotImplementedError("Core losses not supported")
    _check_supported_plant(mdl, dead_time=True)
    _check_supported_control(ctrl)


def _check_supported_control(ctrl: VectorControlSystem) -> None:
    """Raise an error if the control system is not supported."""
    check_supported_fvc(ctrl.vector_ctrl)
    _check_supported_speed_control(ctrl, dead_time=True)


def check_supported_fvc(fvc: Any) -> None:
    """
    Raise an error if the flux-vector controller is not supported: the offline
    references, `k_f=None`, the observer gain `k_o` of the form k0 + k1*abs(w_m), and
    a machine model with constant inductances or a GradNet flux map.
    """
    if not isinstance(fvc, FluxVectorController):
        raise NotImplementedError("Only FluxVectorController supported")
    cfg = fvc.cfg
    if cfg.online_ref or cfg.k_f is not None:
        raise NotImplementedError("Only offline references and k_f=None supported")
    if cfg.k_o is not None:
        fit_speed_dependent_gain(cfg.k_o, "k_o")  # Raises if not supported
    if isinstance(fvc.par, SaturatedSynchronousMachinePars):
        if fvc.par.psi_s_dq_fcn is None:
            raise NotImplementedError("A flux map (psi_s_dq_fcn) is required")
        export_gradnet(fvc.par.psi_s_dq_fcn)  # Raises if not supported
    elif not isinstance(fvc.par, SynchronousMachinePars):
        raise NotImplementedError("Machine model of the control system not supported")


def export_values(
    ctrl: VectorControlSystem,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """
    Get the parameter values of the control system in the motulator API.

    Parameters
    ----------
    ctrl : VectorControlSystem
        Discrete-time control system.

    Returns
    -------
    values : dict[str, Any]
        Parameter values, None for the defaults (see `_sm_control.init_script`).
    flux_map : dict[str, Any] | None
        GradNet flux map of the control system, None if constant inductances are used.

    """
    values, flux_map = fvc_values(cast(FluxVectorController, ctrl.vector_ctrl))
    return values | speed_ctrl_values(ctrl) | pwm_values(ctrl.pwm), flux_map


def fvc_values(
    fvc: FluxVectorController,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """
    Get the parameter values of the flux-vector controller (see `export_values`):
    its machine model (`PAR_FIELDS` and `psi_s_dq_fcn`, the name of the struct of a
    flux map) and its configuration (`CFG_FIELDS`, with `k_o` as [k0, k1]), with the
    GradNet flux map.
    """
    cfg: FluxVectorControllerCfg = fvc.cfg
    flux_map = None
    if isinstance(fvc.par, SaturatedSynchronousMachinePars):
        flux_map = export_gradnet(fvc.par.psi_s_dq_fcn)
        values = {"n_p": fvc.par.n_p, "R_s": fvc.par.R_s, "L_d": None, "L_q": None}
        values |= {"psi_f": None, "psi_s_dq_fcn": "est_flux_map"}
    else:
        values = {n: getattr(fvc.par, n) for n in PAR_FIELDS}
        values |= {"psi_s_dq_fcn": None}
    values |= {n: getattr(cfg, n) for n in CFG_FIELDS}
    values["k_o"] = (
        None if cfg.k_o is None else fit_speed_dependent_gain(cfg.k_o, "k_o")
    )
    values["sensorless"] = int(cfg.sensorless)
    return values, flux_map


def export_plant_variables(mdl: Drive) -> list[tuple[str, Any]]:
    """Get the workspace variables of the system model."""
    variables: list[tuple[str, Any]] = []
    if has_gradnet_plant(mdl):
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


def machine_cscript_code() -> dict[str, str]:
    """Generate the code sections of the C-Script block of the GradNet machine."""
    declarations = (
        "/* Generated by motulator_export.plecs._sm. The magnetic model is in the\n"
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
        "configuration": "2",
        "R": "machine.R_s",
        "L": "[machine.L_d machine.L_q]",
        "phi": "machine.psi_f",
        "J": "mechanics.J",
        "F": "0",
        "p": "machine.n_p",
        "wm0": "mechanics.w_M0",
        "thm0": "mechanics.theta_M0",
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


# Name of the machine with a GradNet current map in the library
GRADNET_MACHINE = "SynchronousMachine (GradNet)"


def gradnet_machine() -> tuple[str, str]:
    """
    Mask and contents of the subsystem of the machine with a GradNet current map.

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
        machine_cscript_code(),
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
    sensor = {"SecondFlange": "1", "theta0": "mechanics.theta_M0"}
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
    return header, trailer


def _add_gradnet_machine(sch: _Schematic, library: bool) -> None:
    """Add the machine with a GradNet current map (`gradnet_machine`)."""
    if library:
        terminals = _terminals(MACHINE_TERMINALS)
        _add_reference(
            sch,
            GRADNET_MACHINE,
            "Machine",
            MACH,
            MACHINE_FRAME,
            terminals,
            label="east",
        )
        return
    header, trailer = gradnet_machine()
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
    t_stop: float,
    outputs: bool = False,
    library: bool = False,
) -> Path:
    """
    Write a PLECS model of the synchronous machine drive, see `plecs.write_model`.
    """
    path = Path(path)
    check_supported(mdl, ctrl)
    values, flux_map = export_values(ctrl)
    w_M_ref, tau_L = drive_references(mdl, ctrl, values["T_s"])
    variables = export_plant_variables(mdl)
    if flux_map is not None:
        variables += [(f"est_flux_map.{f}", flux_map[f]) for f in GRADNET_FIELDS]
    init = init_script(variables, values, INIT_COMMENT)
    block, T_s = FVC_BLOCK, T_S_EXPR
    sch = _Schematic()
    gradnet_plant = has_gradnet_plant(mdl)
    sources: list[tuple[str, StepSignal | float | str]] = [
        (ENABLE, gate_enable(ctrl, values["T_s"])),
        ("w_M_ref", w_M_ref),
        ("i_s_abc", _probe("Machine", MACHINE_PROBES[:1])),
        ("u_dc meas.", _probe("u_dc", ["Measured voltage"])),
        ("theta_M", _probe("Machine", MACHINE_PROBES[2:3])),
    ]
    y_sources = _add_control_system(sch, block, sources, VALUES, library)
    _add_delay(sch, T_s, block)
    _add_pwm(sch, T_s, mdl.single_update)
    t_d = cast(CarrierComparison, mdl.pwm).t_d
    if t_d > 0:
        _add_blanking_time(sch)
    # The blanking time, the diode bridge, and its grid need space between the PWM and
    # the DC bus
    sch.dx = 320 if isinstance(mdl.dc_bus, DiodeBridgeDCBus) else 0
    sch.dx += BLANKING_DX if t_d > 0 else 0
    sch.dx += DC_DX + GATE_DX
    _add_converter(sch, T_s, t_d, mdl.converter.u_th)
    _add_dc_bus(sch, mdl.dc_bus)
    if gradnet_plant:
        _add_gradnet_machine(sch, library)
    else:
        _add_pmsm(sch)
    _add_phase_lines(sch)
    _add_mechanics(sch, tau_L, inertia=gradnet_plant)
    y_probes = _add_drive_outputs(sch, block, outputs, y_sources)
    size = (880 + sch.dx, max(520, y_probes + 50))
    return _write_model(path, init, t_stop, T_s, sch, size, outputs)


def simulate(
    path: str | Path, t_eval: np.ndarray, mdl: Drive, ctrl: VectorControlSystem
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the PLECS model of a synchronous machine drive, see `plecs.simulate`.
    """
    return simulate_plecs(
        path, t_eval, mdl_outputs=MDL_OUTPUTS, ctrl_outputs=FVC_BLOCK.signals
    )
