"""
Export motulator grid converter systems to PLECS.

This module converts a grid converter system with grid-following current-vector
control or disturbance-observer-based grid-forming control into a PLECS Standalone
model, in the same way as `motulator_export.plecs._sm` does for drive systems. In
the control system, each class of motulator is a block with a mask of its arguments
(see `_grid_control`), with the C port in `c/gfl_current_vector.c` or
`c/gfm_observer.c`. The parameters are defined in the initialization commands of
the PLECS model as the structs cfg (`CurrentVectorControllerCfg` or
`ObserverBasedGridFormingControllerCfg`) and pwm (`PWM`). The system model consists
of PLECS blocks.

Currently supported:

- Grid-following control (`CurrentVectorController`) in the power-control mode, with
  the active and reactive power references (constants or step signals)
- Grid-forming control (`ObserverBasedGridFormingController`) in the power-control
  mode, with the active power reference and the converter voltage reference
  (constants or step signals)
- Converter: carrier comparison (`pwm=True`) with the Symmetrical PWM block and the
  IGBT converter of PLECS with ideal IGBTs and diodes, a stiff DC bus
  (`VoltageSourceConverter` without a DC-bus model), and the computational delay of
  one sampling period as a PLECS Delay block
- Gate blocking (`set_enable`), as in `motulator_export.plecs._sm`
- AC filter: LCL filter (`LCLFilter`) without resistances and grid impedance, or L
  filter (`LFilter`) with its series resistance and the grid inductance
- Grid: balanced three-phase voltage source (`ThreePhaseSource`) with constant
  magnitude and frequency

"""

from pathlib import Path
from typing import Any, cast

import numpy as np
from motulator.common.model._converter import VoltageSourceConverter
from motulator.common.model._pwm import CarrierComparison
from motulator.common.utils import complex2abc
from motulator.grid.control import (
    CurrentVectorController,
    GridConverterControlSystem,
    ObserverBasedGridFormingController,
)
from motulator.grid.model import (
    GridConverterSystem,
    LCLFilter,
    LFilter,
    ThreePhaseSource,
)

from motulator_export._common import (
    StepSignal,
    check_supported_pwm,
    gate_enable,
    reference,
)
from motulator_export.plecs._common import (
    CONV,
    DC_DX,
    ENABLE,
    GATE_DX,
    ControlBlock,
    _add_control_system,
    _add_converter,
    _add_ctrl_output,
    _add_dc_bus,
    _add_delay,
    _add_pwm,
    _check_delay,
    _check_initial_states,
    _write_model,
)
from motulator_export.plecs._control import INIT_COMMENT, T_S_EXPR, VALUES, pwm_values
from motulator_export.plecs._grid_control import (
    GFL_BLOCK,
    GFL_CFG_FIELDS,
    GFM_BLOCK,
    GFM_CFG_FIELDS,
    init_script,
)
from motulator_export.plecs._rpc import simulate_plecs
from motulator_export.plecs._schematic import Tap, _probe, _Schematic, _scope

__all__ = ["simulate", "write_model"]


# %%
def _check_supported_plant(mdl: GridConverterSystem) -> None:
    """Raise an error if the system model is not supported."""
    if type(mdl.converter) is not VoltageSourceConverter:
        raise NotImplementedError("Only VoltageSourceConverter supported")
    if mdl.dc_bus is not None:
        raise NotImplementedError("DC-bus model not supported, only a stiff DC bus")
    f = mdl.ac_filter
    sensors = (mdl.converter.u_dc_sensor, f.i_c_sensor, f.u_g_sensor)
    if any(sensor is not None for sensor in sensors):
        raise NotImplementedError("Sensor models not supported")
    if isinstance(f, LCLFilter):
        if f.R_fc or f.R_fg or f.L_g or f.R_g:
            raise NotImplementedError("Only LCLFilter without resistances and L_g")
    elif not isinstance(f, LFilter) or f.R_g:
        raise NotImplementedError("Only LFilter and LCLFilter without R_g supported")
    src = mdl.ac_source
    if type(src) is not ThreePhaseSource or callable(src.w_g) or callable(src.e_g):
        raise NotImplementedError("Only ThreePhaseSource with constant w_g and e_g")
    if src.phi != 0 or src.e_g_neg != 0:
        raise NotImplementedError("Phase shift and negative sequence not supported")
    if not isinstance(mdl.pwm, CarrierComparison):
        raise NotImplementedError("Only CarrierComparison (pwm=True) supported")
    if mdl.pwm.t_d != 0 or mdl.converter.u_th != 0:
        raise NotImplementedError(
            "Converter dead time and threshold voltage not supported"
        )
    _check_delay(mdl)
    # The capacitor voltage of the LCL filter is exported, the other states start
    # from their initial values
    states = [("ac_source.state.exp_j_theta_g", src.state.exp_j_theta_g, 1 + 0j)]
    states.append(("ac_filter.state.i_c_ab", f.state.i_c_ab, 0j))
    exported: list[tuple[str, complex]] = []
    if isinstance(f, LCLFilter):
        states.append(("ac_filter.state.i_g_ab", f.state.i_g_ab, 0j))
        exported.append(("ac_filter.state.u_f_ab", f.state.u_f_ab))
    _check_initial_states(states, exported)


def check_supported(mdl: GridConverterSystem, ctrl: GridConverterControlSystem) -> None:
    """Raise an error if the grid converter system is not supported."""
    _check_supported_plant(mdl)
    check_supported_pwm(ctrl)
    f = mdl.ac_filter
    inner = ctrl.inner_ctrl
    if not isinstance(
        inner, (CurrentVectorController, ObserverBasedGridFormingController)
    ):
        raise NotImplementedError("Only CurrentVectorController and DO-GFM supported")
    # The PCC voltage of grid-following control is measured after the LCL filter
    if isinstance(inner, CurrentVectorController) != isinstance(f, LCLFilter):
        raise NotImplementedError(
            "Only grid-following control with LCLFilter and grid-forming control "
            "with LFilter supported"
        )
    if ctrl.dc_bus_voltage_ctrl is not None:
        raise NotImplementedError("DC-bus voltage control not supported")


def export_plant_variables(mdl: GridConverterSystem) -> list[tuple[str, Any]]:
    """Workspace variables of the system model."""
    f = mdl.ac_filter
    src = cast(ThreePhaseSource, mdl.ac_source)
    variables: list[tuple[str, Any]] = [("converter.u_dc", mdl.converter.inp.u_dc)]
    if isinstance(f, LCLFilter):
        variables += [
            ("ac_filter.L_fc", f.L_fc),
            ("ac_filter.L_fg", f.L_fg),
            ("ac_filter.C_f", f.C_f),
            ("ac_filter.u_f0_abc", complex2abc(f.state.u_f_ab)),
        ]
    else:
        f = cast(LFilter, f)
        variables += [
            ("ac_filter.L_f", f.L_f),
            ("ac_filter.R_f", f.R_f),
            ("ac_filter.L_g", f.L_g),
        ]
    return variables + [("ac_source.e_g", src.e_g), ("ac_source.w_g", src.w_g)]


def grid_references(
    ctrl: GridConverterControlSystem, gfl: bool, T_s: float
) -> list[tuple[str, StepSignal | float]]:
    """
    References of the grid converter control system, read from it: the active power
    reference and the reactive power reference (grid-following control, `gfl`) or
    the AC voltage reference (grid-forming control), constants or `StepSignal`
    objects sampled to switch at the same sampling instant as in motulator.
    """
    ext_ref = ctrl.ext_ref
    refs = [("p_g_ref", reference(ext_ref.p_g, "active power reference", T_s))]
    if gfl:
        q_g_ref = reference(ext_ref.q_g, "reactive power reference", T_s)
        return [*refs, ("q_g_ref", q_g_ref)]
    return [*refs, ("v_c_ref", reference(ext_ref.v_c, "AC voltage reference", T_s))]


def control_block(ctrl: GridConverterControlSystem) -> ControlBlock:
    """Control-system block of grid-following or grid-forming control."""
    if isinstance(ctrl.inner_ctrl, CurrentVectorController):
        return GFL_BLOCK
    return GFM_BLOCK


def export_values(ctrl: GridConverterControlSystem) -> dict[str, Any]:
    """
    Get the parameter values of the control system in the motulator API, None for
    the defaults (see `_grid_control.init_script`).
    """
    inner = ctrl.inner_ctrl
    if isinstance(inner, CurrentVectorController):
        cfg, fields = inner.cfg, GFL_CFG_FIELDS
    else:
        cfg, fields = (
            cast(ObserverBasedGridFormingController, inner).cfg,
            GFM_CFG_FIELDS,
        )
    values = {n: getattr(cfg, n) for n in fields}
    return values | pwm_values(ctrl.pwm)


# %%
def _abc_probe(comp: str, signal: str) -> str:
    """Probe of the three-phase measurements comp + "a", comp + "b", comp + "c"."""
    return "".join(_probe(f"{comp}{ph}", [signal]) for ph in "abc")


def _grid_source(k: int) -> dict[str, str]:
    """Grid voltage e_g*cos(w_g*t - k*2*pi/3) of the phase k, exp_j_theta_g(0) = 1."""
    return {"V": "ac_source.e_g", "w": "ac_source.w_g", "phi": f"pi/2 - {k}*2*pi/3"}


def _add_converter_meters(sch: _Schematic, y_ph: list[int]) -> None:
    """Add the ammeters of the converter currents at the phase lines `y_ph`."""
    for k, ph in enumerate("abc"):
        sch.component(
            "Ammeter", f"i_c{ph}", (690, y_ph[k]), direction="left", show=False
        )
        jog = [] if k == 1 else [(660, CONV[1] - 10 + 10 * k), (660, y_ph[k])]
        sch.wire(("Converter", k + 1), (f"i_c{ph}", 1), jog)


# Line-to-line PCC voltages u_ab and u_bc (measured for the control system), with
# the voltmeters between the phase lines
PCC_METERS = [[("u_gab", 1)], [("u_gab", 2), ("u_gbc", 1)], [("u_gbc", 2)]]


def _add_pcc_meters(sch: _Schematic, x_v: int, y_ph: list[int]) -> None:
    """Add the voltmeters of the PCC voltages at `x_v` between the phase lines."""
    sch.component("Voltmeter", "u_gab", (x_v, y_ph[0] + 20), direction="up", show=False)
    sch.component("Voltmeter", "u_gbc", (x_v, y_ph[1] + 20), direction="up", show=False)


def _add_lcl_filter_and_grid(sch: _Schematic) -> None:
    """Add the LCL filter, the grid, and the PCC voltage measurements."""
    y_ph = [60, 100, 140]  # Phase lines a, b, and c
    x_cap = [780, 810, 840]  # Filter capacitors
    x_v = 940  # Line-to-line voltmeters between the phase lines
    _add_converter_meters(sch, y_ph)
    for k, ph in enumerate("abc"):
        y = y_ph[k]
        # Converter-side inductor, capacitor, and grid-side inductor
        inductor = {"i_init": "0", "L": "ac_filter.L_fc"}
        sch.component(
            "Inductor",
            f"L_fc{ph}",
            (740, y),
            inductor,
            direction="left",
            label="north",
            show=k == 0,
        )
        sch.wire((f"i_c{ph}", 2), (f"L_fc{ph}", 1))
        capacitor = {"C": "ac_filter.C_f", "v_init": f"ac_filter.u_f0_abc({k + 1})"}
        sch.component(
            "Capacitor",
            f"C_f{ph}",
            (x_cap[k], 175),
            capacitor,
            direction="up",
            label="east",
            show=k == 2,  # The label of the last capacitor does not overlap
        )
        inductor = {"i_init": "0", "L": "ac_filter.L_fg"}
        sch.component(
            "Inductor",
            f"L_fg{ph}",
            (890, y),
            inductor,
            direction="left",
            label="north",
            show=k == 0,
        )
        taps: list[Tap] = [([(x_cap[k], y)], [((f"C_f{ph}", 1), [])])]
        sch.bus((f"L_fc{ph}", 2), "Wire", taps + [([], [((f"L_fg{ph}", 1), [])])])
        sch.component(
            "ACVoltageSource",
            f"e_g{ph}",
            (990, y),
            _grid_source(k),
            direction="left",
            label="north",
            show=k == 0,
        )
    # Star point of the capacitors
    taps = [([(x_cap[0], 200), (x_cap[1], 200)], [(("C_fb", 2), [])])]
    sch.bus(("C_fa", 2), "Wire", taps + [([(x_cap[2], 200)], [(("C_fc", 2), [])])])
    _add_pcc_meters(sch, x_v, y_ph)
    for k, ph in enumerate("abc"):
        dsts = [(m, []) for m in PCC_METERS[k]]
        taps = [([(x_v, y_ph[k])], dsts), ([], [((f"e_g{ph}", 1), [])])]
        sch.bus((f"L_fg{ph}", 2), "Wire", taps)
    # Star point of the grid (not grounded, the circuit is floating as in motulator)
    taps = [([(1030, y_ph[0]), (1030, y_ph[1])], [(("e_gb", 2), [])])]
    sch.bus(("e_ga", 2), "Wire", taps + [([(1030, y_ph[2])], [(("e_gc", 2), [])])])


def _add_l_filter_and_grid(sch: _Schematic) -> None:
    """
    Add the L filter with its resistance, the grid inductance, and the grid, with the
    PCC voltmeters between the filter and the grid inductance.
    """
    y_ph = [60, 100, 140]  # Phase lines a, b, and c
    x_v = 815  # Line-to-line voltmeters between the phase lines
    _add_converter_meters(sch, y_ph)
    _add_pcc_meters(sch, x_v, y_ph)
    for k, ph in enumerate("abc"):
        elements = [
            ("Inductor", "L_f", {"i_init": "0", "L": "ac_filter.L_f"}),
            ("Resistor", "R_f", {"R": "ac_filter.R_f"}),
            ("Inductor", "L_g", {"i_init": "0", "L": "ac_filter.L_g"}),
            ("ACVoltageSource", "e_g", _grid_source(k)),
        ]
        prev = f"i_c{ph}"
        for n, (typ, name, params) in enumerate(elements):
            sch.component(
                typ,
                f"{name}{ph}",
                (740 + 50 * n, y_ph[k]),
                params,
                direction="left",
                label="north",
                show=k == 0,
            )
            if n == 2:  # The PCC voltmeters branch from the line
                dsts = [(m, []) for m in PCC_METERS[k]]
                taps = [([(x_v, y_ph[k])], dsts), ([], [((f"{name}{ph}", 1), [])])]
                sch.bus((prev, 2), "Wire", taps)
            else:
                sch.wire((prev, 2), (f"{name}{ph}", 1))
            prev = f"{name}{ph}"
    # Star point of the grid (not grounded, the circuit is floating as in motulator)
    taps: list[Tap] = [([(930, y_ph[0]), (930, y_ph[1])], [(("e_gb", 2), [])])]
    sch.bus(("e_ga", 2), "Wire", taps + [([(930, y_ph[2])], [(("e_gc", 2), [])])])


def mdl_outputs(mdl: GridConverterSystem) -> list[str]:
    """Signals of the output port "mdl": the converter (and grid) currents."""
    names = ["i_c_a", "i_c_b", "i_c_c"]
    if isinstance(mdl.ac_filter, LCLFilter):
        names += ["i_g_a", "i_g_b", "i_g_c"]
    return names


def _add_outputs(
    sch: _Schematic, block: ControlBlock, lcl: bool, outputs: bool
) -> None:
    """Add the scope and its probes, and optionally the output ports."""
    inductor = "L_fc" if lcl else "L_f"
    # Output ports for scripted simulations: currents and controller signals
    if outputs:
        probes = _abc_probe(inductor, "Inductor current")
        if lcl:
            probes += _abc_probe("L_fg", "Inductor current")
        sch.component("PlecsProbe", "Currents", (400, 300), extra=probes)
        sch.component("Output", "mdl", (480, 300), {"Index": "1", "Width": "-1"})
        sch.signal(("Currents", 1), ("mdl", 1))
        _add_ctrl_output(sch, block)
    # Scope: powers, converter currents, and the PCC voltages (LCL filter) or the
    # second monitored signal group of the control system
    groups = list(block.outputs)
    if lcl:
        voltages = _probe("u_gab", ["Measured voltage"])
        voltages += _probe("u_gbc", ["Measured voltage"])
        voltage_axis = ("Voltage", "Line-to-line voltage (V)")
    else:
        voltages = _probe(block.name, groups[1:2])
        voltage_axis = ("Voltage", "Voltage (V)")
    probes_scope = [
        ("Powers", _probe(block.name, groups[:1]), 300),
        ("Converter currents", _abc_probe(inductor, "Inductor current"), 340),
        ("Voltages", voltages, 380),
    ]
    for name, extra, y in probes_scope:
        sch.component("PlecsProbe", name, (720, y), extra=extra)
    axes = [("Power", "Power (W, VAr)"), ("Current", "Current (A)"), voltage_axis]
    sch.component("Scope", "Scope", (900, 320), extra=_scope(axes), direction="up")
    sch.signal(("Powers", 1), ("Scope", 1), [(760, 300), (760, 310)])
    sch.signal(("Converter currents", 1), ("Scope", 2), [(770, 340), (770, 320)])
    sch.signal(("Voltages", 1), ("Scope", 3), [(780, 380), (780, 330)])


# %%
def write_model(
    path: str | Path,
    mdl: GridConverterSystem,
    ctrl: GridConverterControlSystem,
    t_stop: float,
    outputs: bool = False,
    library: bool = False,
) -> Path:
    """
    Write a PLECS model of the grid converter system, see `plecs.write_model`.

    The references are the active power reference (`set_power_ref`) and, in
    grid-following control, the reactive power reference (`set_reactive_power_ref`)
    or, in grid-forming control, the AC voltage reference (`set_ac_voltage_ref`).

    """
    path = Path(path)
    check_supported(mdl, ctrl)
    block, values = control_block(ctrl), export_values(ctrl)
    init = init_script(
        export_plant_variables(mdl), values, INIT_COMMENT, gfl=block is GFL_BLOCK
    )
    T_s = T_S_EXPR
    lcl = isinstance(mdl.ac_filter, LCLFilter)
    sch = _Schematic()
    sources: list[tuple[str, StepSignal | float | str]] = [
        (ENABLE, gate_enable(ctrl, values["T_s"]))
    ]
    sources += list(grid_references(ctrl, block is GFL_BLOCK, values["T_s"]))
    sources += [("i_c_abc", _abc_probe("i_c", "Measured current"))]
    voltages = _probe("u_gab", ["Measured voltage"])
    voltages += _probe("u_gbc", ["Measured voltage"])
    sources += [("u_g_line", voltages)]
    sources += [("u_dc meas.", _probe("u_dc", ["Measured voltage"]))]
    _add_control_system(sch, block, sources, VALUES, library)
    _add_delay(sch, T_s, block)
    _add_pwm(sch, T_s, mdl.single_update)
    sch.dx = DC_DX + GATE_DX  # Space for the gate enable and the DC bus
    _add_converter(sch, T_s)
    _add_dc_bus(sch, None)
    if lcl:
        _add_lcl_filter_and_grid(sch)
    else:
        _add_l_filter_and_grid(sch)
    _add_outputs(sch, block, lcl, outputs)
    size = (1200 + sch.dx, 440)
    return _write_model(path, init, t_stop, T_s, sch, size, outputs)


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    mdl: GridConverterSystem,
    ctrl: GridConverterControlSystem,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Simulate the PLECS model of a grid converter system, see `plecs.simulate`."""
    return simulate_plecs(
        path,
        t_eval,
        mdl_outputs=mdl_outputs(mdl),
        ctrl_outputs=control_block(ctrl).signals,
    )
