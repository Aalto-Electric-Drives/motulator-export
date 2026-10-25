"""
MATLAB scripts that build the Simulink models of motulator systems.

A script builds the system with the MATLAB classes of motulator (the package
`+motulator` of this folder), as a motulator script does, and the Simulink model
from the blocks of the library of motulator (`Simulation.build`). The arguments are
written only where they differ from the defaults of the classes, which are those of
motulator, and the references are the constants or the step signals of the motulator
objects (the model samples the references of the control system at its sampling
instants).

"""

import dataclasses
import inspect
import os
from collections.abc import Sequence
from math import isinf
from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.control._pwm import PWM
from motulator.common.utils._utils import wrap
from motulator.drive.control import im as im_control
from motulator.drive.control import sm as sm_control
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.drive.control._common import SpeedController
from motulator.drive.model import Drive, ExternalRotorSpeed, MechanicalSystem
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
    SpatialSaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)
from motulator.grid import control as grid_control
from motulator.grid import model as grid_model
from motulator.grid.control import GridConverterControlSystem
from motulator.grid.model import GridConverterSystem, LCLFilter, ThreePhaseSource

from motulator_export._common import StepSignal, reference, system_type
from motulator_export.plecs import _grid, _im, _sm
from motulator_export.plecs._common import GRADNET_FIELDS, _fmt_mask
from motulator_export.plecs._control import assign
from motulator_export.plecs._drive import torque_control
from motulator_export.plecs._grid_control import GFL_CFG_FIELDS, GFM_CFG_FIELDS
from motulator_export.plecs._im_control import CFG_FIELDS as CVC_CFG_FIELDS
from motulator_export.plecs._im_control import PAR_FIELDS as IM_PAR_FIELDS
from motulator_export.plecs._im_fvc_control import CFG_FIELDS as IM_FVC_CFG_FIELDS
from motulator_export.plecs._im_vhz_control import CFG_FIELDS as VHZ_CFG_FIELDS
from motulator_export.plecs._sm_control import CFG_FIELDS as FVC_CFG_FIELDS
from motulator_export.plecs._sm_control import PAR_FIELDS as SM_PAR_FIELDS
from motulator_export.plecs._sm_cvc_control import CFG_FIELDS as SM_CVC_CFG_FIELDS
from motulator_export.plecs._sm_vhz_control import CFG_FIELDS as SM_VHZ_CFG_FIELDS

SIMULINK = Path(__file__).parent

# Packages of the classes of the scripts of the system types, as in motulator
IMPORTS = {
    "sm": ["drive.model", "drive.control.sm", "drive.utils"],
    "im": ["drive.model", "drive.control.im", "drive.utils"],
    "grid": ["grid.model", "grid.control", "grid.utils"],
}


# Positional arguments of VectorControlSystem in the scripts
VECTOR_ARGS = ["vector_ctrl", "speed_ctrl"]


# %%
# MATLAB code
def _value(v: Any) -> str:  # noqa: PLR0911
    """MATLAB literal of an argument."""
    if v is None:
        return "[]"
    if isinstance(v, (bool, np.bool_)):
        return "true" if v else "false"
    if isinstance(v, StepSignal):
        args = [f"time={_value(v.time)}", f"after={_value(v.after)}"]
        if v.before != 0:
            args.append(f"before={_value(v.before)}")
        return f"StepSignal({', '.join(args)})"
    if isinstance(v, str):
        return "'" + v.replace("'", "''") + "'"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, (complex, np.complexfloating)):
        if v.imag == 0:
            return _value(float(v.real))
        return f"complex({_value(float(v.real))}, {_value(float(v.imag))})"
    if isinstance(v, (float, np.floating)):
        v = float(v)
        if isinf(v):
            return "inf" if v > 0 else "-inf"
        if v.is_integer() and abs(v) < 1e15:
            return str(int(v))
        return _fmt_mask(v)
    if isinstance(v, (Sequence, np.ndarray)):
        return "[" + " ".join(_value(x) for x in np.ravel(np.asarray(v))) + "]"
    raise TypeError(f"No MATLAB literal of {v!r}")


def _equal(a: Any, b: Any) -> bool:
    """Whether an argument equals the default."""
    if a is None or b is None:
        return a is None and b is None
    if isinstance(a, (Sequence, np.ndarray)) or isinstance(b, (Sequence, np.ndarray)):
        return bool(np.array_equal(np.asarray(a), np.asarray(b)))
    return bool(a == b)


def _call(
    cls: str,
    positional: Sequence[str],
    named: Sequence[tuple[str, Any]],
    defaults: dict[str, Any] | None = None,
) -> str:
    """
    Call of the constructor of a class with the positional arguments (MATLAB code)
    and the name-value arguments, those equal to the defaults left out.
    """
    defaults = defaults or {}
    args = [*positional]
    args += [
        f"{n}={v if isinstance(v, _Code) else _value(v)}"
        for n, v in named
        if n not in defaults or not _equal(v, defaults[n])
    ]
    return f"{cls}({', '.join(args)})"


class _Code(str):
    """MATLAB code of an argument (e.g., a call), written as is."""


def _defaults(cls: Any) -> dict[str, Any]:
    """Defaults of the arguments of a motulator class (a dataclass or a class)."""
    if dataclasses.is_dataclass(cls):
        return {
            f.name: f.default
            for f in dataclasses.fields(cls)
            if f.default is not dataclasses.MISSING
        }
    return {
        n: p.default
        for n, p in inspect.signature(cls).parameters.items()
        if p.default is not inspect.Parameter.empty
    }


def _struct(name: str, fields: dict[str, Any]) -> list[str]:
    """Assignments of the fields of a struct."""
    return [assign(f"{name}.{f}", v).rstrip("\n") for f, v in fields.items()]


# %%
# Parts of the systems
def _pwm(values: dict[str, Any], default_overmodulation: str = "MPE") -> str | None:
    """
    PWM of the control system (the values of `pwm_values`), or None if it is the
    default PWM of the control system, with the overmodulation
    `default_overmodulation`.
    """
    t_d, u_th, i_0 = (values[f"pwm_{n}"] for n in ("t_d", "u_th", "i_0"))
    d_err = None
    if t_d != 0 or u_th != 0:
        d_err = _Code(
            _call(
                "InverterNonlinearity",
                [],
                [("t_d", t_d), ("u_th", u_th), ("i_0", i_0)],
                {"t_d": 0.0, "u_th": 0.0, "i_0": 0.0},
            )
        )
    named = [
        ("overmodulation", values["pwm_overmodulation"]),
        ("d_err", d_err),
        ("feedforward", bool(values["pwm_feedforward"])),
        ("d_min", values["pwm_d_min"]),
        ("single_update", bool(values["pwm_single_update"])),
    ]
    defaults = _defaults(PWM)
    if all(_equal(v, defaults[n]) for n, v in named[1:]) and _equal(
        named[0][1], default_overmodulation
    ):
        return None
    return _call("PWM", [], named, defaults)


def _speed_ctrl(values: dict[str, Any]) -> str:
    """Speed controller (the values of `speed_ctrl_values`)."""
    named = [
        (n, values[f"speed_{n}"]) for n in ("J", "alpha_s", "alpha_i", "tau_M_max")
    ]
    return _call("SpeedController", [], named, _defaults(SpeedController))


def _drive_model(mdl: Drive, machine: list[str]) -> list[str]:
    """The system model of a drive, after the lines of the machine."""
    if isinstance(mdl.mechanics, ExternalRotorSpeed):
        lines = [*machine, "mechanics = ExternalRotorSpeed();"]
    else:
        mech = cast(MechanicalSystem, mdl.mechanics)
        lines = [*machine, f"mechanics = MechanicalSystem(J={_value(mech.J)});"]
        w_M0 = float(mech.state.w_M.real)
        if w_M0 != 0:
            lines.append(f"mechanics.state.w_M = {_value(w_M0)};")
    theta_M0 = float(wrap(mdl.mechanics.state.theta_M.real))
    if theta_M0 != 0:
        lines.append(f"mechanics.state.theta_M = {_value(theta_M0)};")
    return [
        *lines,
        f"converter = VoltageSourceConverter(u_dc={_value(mdl.converter.u_dc)});",
        "mdl = Drive(machine, mechanics, converter);",
    ]


def _drive_references(
    mdl: Drive, ctrl: VectorControlSystem | VHzControlSystem
) -> list[str]:
    """
    The speed or the torque reference, and the load torque or the external rotor
    speed of a drive.
    """
    if torque_control(ctrl):
        tau_M_ref = reference(ctrl.ext_ref.tau_M, "torque reference")
        lines = [f"ctrl.set_torque_ref({_value(tau_M_ref)});"]
    else:
        w_M_ref = reference(ctrl.ext_ref.w_M, "speed reference")
        lines = [f"ctrl.set_speed_ref({_value(w_M_ref)});"]
    if isinstance(mdl.mechanics, ExternalRotorSpeed):
        w_M = reference(mdl.mechanics.w_M, "external rotor speed")
        if not _equal(w_M, 0.0):
            lines.append(f"mdl.mechanics.set_external_rotor_speed({_value(w_M)});")
        return lines
    tau_L = reference(cast(MechanicalSystem, mdl.mechanics).inp.tau_L, "load torque")
    if not _equal(tau_L, 0.0):
        lines.append(f"mdl.mechanics.set_external_load_torque({_value(tau_L)});")
    return lines


def _vector_control_system(
    ctrl: VectorControlSystem, est: str, values: dict[str, Any]
) -> list[str]:
    """
    The vector control system of the vector controller of the machine model `est`
    (with the initial speed estimate of its observer, if not zero), with the speed
    controller or in the torque-control mode.
    """
    lines = [f"vector_ctrl = {type(ctrl.vector_ctrl).__name__}({est}, cfg);"]
    if values["observer_w_M"] != 0:
        lines.append(f"vector_ctrl.observer.w_M = {_value(values['observer_w_M'])};")
    if torque_control(ctrl):
        return [*lines, _control_system("VectorControlSystem", VECTOR_ARGS[:1], values)]
    return [
        *lines,
        f"speed_ctrl = {_speed_ctrl(values)};",
        _control_system("VectorControlSystem", VECTOR_ARGS, values),
    ]


def _par_call(cls: str, par: Any, fields: Sequence[str]) -> str:
    """Machine model parameters (all of them)."""
    return _call(cls, [], [(f, getattr(par, f)) for f in fields])


def _sm_script(
    mdl: Drive, ctrl: VectorControlSystem | VHzControlSystem
) -> tuple[list[str], list[str]]:
    """The system model and the control system of a synchronous machine drive."""
    if isinstance(ctrl, VHzControlSystem):
        values, flux_map = _sm.export_vhz_values(ctrl)
        cfg_cls, cfg_fields = (
            sm_control.ObserverBasedVHzControllerCfg,
            SM_VHZ_CFG_FIELDS,
        )
    elif isinstance(ctrl.vector_ctrl, sm_control.CurrentVectorController):
        values, flux_map = _sm.export_values(ctrl)
        cfg_cls, cfg_fields = sm_control.CurrentVectorControllerCfg, SM_CVC_CFG_FIELDS
    else:
        values, flux_map = _sm.export_values(ctrl)
        cfg_cls, cfg_fields = sm_control.FluxVectorControllerCfg, FVC_CFG_FIELDS
    model: list[str] = []
    if _sm.has_gradnet_plant(mdl):
        par = cast(SpatialSaturatedSynchronousMachinePars, mdl.machine.par)
        g = _sm.export_gradnet(par.magnetic_map_fcn)
        fields = {f: g[f] for f in (*GRADNET_FIELDS, "k")} | {"psi_f": par.psi_f}
        model += [
            "% GradNet current map with spatial harmonics, with their order k and the",
            "% PM-flux linkage psi_f",
            *_struct("current_map", fields),
            f"par = SpatialSaturatedSynchronousMachinePars(n_p={_value(par.n_p)}, "
            f"R_s={_value(par.R_s)}, magnetic_map_fcn=current_map);",
        ]
        plant_values = None
    else:
        par = cast(SynchronousMachinePars, mdl.machine.par)
        model.append(
            f"par = {_par_call('SynchronousMachinePars', par, SM_PAR_FIELDS)};"
        )
        plant_values = [getattr(par, f) for f in SM_PAR_FIELDS]
    model = _drive_model(mdl, [*model, "machine = SynchronousMachine(par);"])
    control: list[str] = []
    if flux_map is not None:
        control += [
            "% GradNet flux map of the control system",
            *_struct("flux_map", {f: flux_map[f] for f in GRADNET_FIELDS}),
            f"est_par = SaturatedSynchronousMachinePars(n_p={_value(values['n_p'])}, "
            f"R_s={_value(values['R_s'])}, psi_s_dq_fcn=flux_map);",
        ]
        est = "est_par"
    elif [values[f] for f in SM_PAR_FIELDS] == plant_values:
        est = "par"
    else:
        control.append(
            "est_par = "
            + _call(
                "SynchronousMachinePars", [], [(f, values[f]) for f in SM_PAR_FIELDS]
            )
            + ";"
        )
        est = "est_par"
    cfg_values = [(f, values[f]) for f in cfg_fields]
    cfg = _call(cfg_cls.__name__, [], cfg_values, _defaults(cfg_cls))
    control.append(f"cfg = {cfg};")
    if isinstance(ctrl, VHzControlSystem):
        control += [
            f"vhz_ctrl = ObserverBasedVHzController({est}, cfg);",
            _vhz_control_system(values),
        ]
    else:
        control += _vector_control_system(ctrl, est, values)
    return model, [*control, *_drive_references(mdl, ctrl)]


def _im_script(
    mdl: Drive, ctrl: VectorControlSystem | VHzControlSystem
) -> tuple[list[str], list[str]]:
    """The system model and the control system of an induction machine drive."""
    par = mdl.machine.par
    if isinstance(par, InductionMachinePars):
        gamma = ["n_p", "R_s", "R_r", "L_ell", "L_s"]
        machine = f"par = {_par_call('InductionMachinePars', par, gamma)};"
        plant_values = None
    else:
        par = cast(InductionMachineInvGammaPars, par)
        machine = (
            f"par = {_par_call('InductionMachineInvGammaPars', par, IM_PAR_FIELDS)};"
        )
        plant_values = [getattr(par, f) for f in IM_PAR_FIELDS]
    model = _drive_model(mdl, [machine, "machine = InductionMachine(par);"])
    if isinstance(ctrl, VHzControlSystem):
        values = _im.export_vhz_values(ctrl)
        cls, fields = im_control.ObserverBasedVHzControllerCfg, VHZ_CFG_FIELDS
    elif isinstance(ctrl.vector_ctrl, im_control.FluxVectorController):
        values = _im.export_values(ctrl)
        cls, fields = im_control.FluxVectorControllerCfg, IM_FVC_CFG_FIELDS
    else:
        values = _im.export_values(ctrl)
        cls, fields = im_control.CurrentVectorControllerCfg, CVC_CFG_FIELDS
    control: list[str] = []
    if [values[f] for f in IM_PAR_FIELDS] == plant_values:
        est = "par"
    else:
        est_call = _call(
            "InductionMachineInvGammaPars", [], [(f, values[f]) for f in IM_PAR_FIELDS]
        )
        control.append(f"est_par = {est_call};")
        est = "est_par"
    cfg = _call(cls.__name__, [], [(f, values[f]) for f in fields], _defaults(cls))
    control.append(f"cfg = {cfg};")
    if isinstance(ctrl, VHzControlSystem):
        control += [
            f"vhz_ctrl = ObserverBasedVHzController({est}, cfg);",
            _vhz_control_system(values),
        ]
    else:
        control += _vector_control_system(ctrl, est, values)
    return model, [*control, *_drive_references(mdl, ctrl)]


def _grid_script(
    mdl: GridConverterSystem, ctrl: GridConverterControlSystem
) -> tuple[list[str], list[str]]:
    """The system model and the control system of a grid converter system."""
    f = mdl.ac_filter
    if isinstance(f, LCLFilter):
        args = [(n, getattr(f, n)) for n in ("L_fc", "L_fg", "C_f")]
        args.append(("u_f0_ab", complex(f.state.u_f_ab)))
        ac_filter = _call("LCLFilter", [], args, _defaults(grid_model.LCLFilter))
    else:
        args = [(n, getattr(f, n)) for n in ("L_f", "R_f", "L_g")]
        ac_filter = _call("LFilter", [], args, _defaults(grid_model.LFilter))
    src = cast(ThreePhaseSource, mdl.ac_source)
    source = _call(
        "ThreePhaseSource",
        [],
        [("w_g", src.w_g), ("e_g", src.e_g)],
        _defaults(ThreePhaseSource),
    )
    model = [
        f"converter = VoltageSourceConverter(u_dc={_value(mdl.converter.u_dc)});",
        f"ac_filter = {ac_filter};",
        f"ac_source = {source};",
        "mdl = GridConverterSystem(converter, ac_filter, ac_source);",
    ]
    values = _grid.export_values(ctrl)
    gfl = isinstance(ctrl.inner_ctrl, grid_control.CurrentVectorController)
    if gfl:
        cls, fields = grid_control.CurrentVectorControllerCfg, GFL_CFG_FIELDS
        inner = "CurrentVectorController"
    else:
        cls, fields = grid_control.ObserverBasedGridFormingControllerCfg, GFM_CFG_FIELDS
        inner = "ObserverBasedGridFormingController"
    ext = ctrl.ext_ref
    refs = [("set_power_ref", reference(ext.p_g, "active power reference"))]
    if gfl:
        refs.append(
            ("set_reactive_power_ref", reference(ext.q_g, "reactive power reference"))
        )
    else:
        refs.append(("set_ac_voltage_ref", reference(ext.v_c, "AC voltage reference")))
    cfg = _call(cls.__name__, [], [(n, values[n]) for n in fields], _defaults(cls))
    control = [
        f"cfg = {cfg};",
        f"inner_ctrl = {inner}(cfg);",
        _control_system("GridConverterControlSystem", ["inner_ctrl"], values),
        *(f"ctrl.{setter}({_value(r)});" for setter, r in refs),
    ]
    return model, control


def _control_system(
    cls: str,
    positional: list[str],
    values: dict[str, Any],
    named: Sequence[tuple[str, Any]] = (),
    defaults: dict[str, Any] | None = None,
    overmodulation: str = "MPE",
) -> str:
    """The control system, with its PWM if not the default."""
    pwm = _pwm(values, overmodulation)
    args = [*named, *([("pwm", _Code(pwm))] if pwm is not None else [])]
    return f"ctrl = {_call(cls, positional, args, defaults)};"


def _vhz_control_system(values: dict[str, Any]) -> str:
    """The V/Hz control system of the V/Hz controller vhz_ctrl."""
    return _control_system(
        "VHzControlSystem",
        ["vhz_ctrl"],
        values,
        [("slew_rate", values["slew_rate"])],
        _defaults(VHzControlSystem),
        "MME",
    )


# %%
def write_script(
    path: str | Path,
    mdl: Any,
    ctrl: Any,
    t_stop: float,
    code: str,
    control_block: str | None = None,
) -> Path:
    """
    Write the script `build_<model>.m` that builds the Simulink model `path` (.slx)
    of the system, in the folder of the model, and return its path. The control
    system is the block `control_block` of another library if given (see
    `Simulation.build`).
    """
    path = Path(path)
    kind = system_type(mdl, ctrl)
    writer = {"sm": _sm_script, "im": _im_script, "grid": _grid_script}[kind]
    model, control = writer(mdl, ctrl)
    try:
        src = Path(os.path.relpath(SIMULINK, path.resolve().parent))
        folder = "fullfile(here, " + ", ".join(_value(p) for p in src.parts) + ")"
    except ValueError:  # Different drives (Windows)
        folder = _value(str(SIMULINK))
    imports = "".join(f"import motulator.{p}.*\n" for p in IMPORTS[kind])
    blocks = (
        "the blocks of the control system are S-functions"
        if code == "c"
        else "the blocks of the control system are MATLAB Function blocks"
    )
    code_arg = "" if code == "c" else ", code='matlab'"
    block_arg = (
        "" if control_block is None else f", control_block={_value(control_block)}"
    )
    text = (
        f"% Build the Simulink model {path.stem}.slx in the folder of this script.\n"
        "% Written by motulator_export.simulink from motulator: the system is built\n"
        "% with the MATLAB classes of motulator, as in a motulator script, and the\n"
        "% model from the blocks of the library of motulator.\n"
        "\n"
        "here = fileparts(mfilename('fullpath'));\n"
        f"addpath({folder});\n" + imports + "\n"
        "%% System model\n" + "".join(f"{line}\n" for line in model) + "\n"
        "%% Control system ([] = None, the default)\n"
        + "".join(f"{line}\n" for line in control)
        + "\n"
        f"%% Simulink model ({blocks})\n"
        "simulation = Simulation(mdl, ctrl);\n"
        f"simulation.build('{path.stem}', t_stop={_value(t_stop)}, folder=here"
        f"{code_arg}{block_arg});\n"
        "% Simulate the model and plot the results:\n"
        "%   res = simulation.simulate();\n"
        "%   res.plot();\n"
    )
    script = path.with_name(f"build_{path.stem}.m")
    script.write_text(text, encoding="utf-8")
    return script
