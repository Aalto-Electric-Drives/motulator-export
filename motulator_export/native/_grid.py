"""
Native simulation of grid converter systems with grid-following or grid-forming
control.

The systems are simulated by the libraries `c/native_gfl.c` and `c/native_gfm.c`.
Supported:

- Grid converter system (`GridConverterSystem`) with an L filter (`LFilter`) or an
  LCL filter (`LCLFilter`) with the grid impedance, and a three-phase source
  (`ThreePhaseSource`), whose parameters are constants, `StepSignal` objects, or
  functions of time
- Converter (`VoltageSourceConverter`), its DC bus, and its PWM model, as in the
  drives (see `motulator_export.native._drive`)
- Grid-following current-vector control (`CurrentVectorController`) or
  disturbance-observer-based grid-forming control
  (`ObserverBasedGridFormingController`) in `GridConverterControlSystem` in the
  power-control mode, with the features of the C port (see
  `motulator_export.plecs.grid`), the gates enabled
- Ideal sensors

"""

from typing import Any, cast

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

from motulator_export.native._build import Signatures
from motulator_export.native._system import (
    PWM_API,
    NativeSystem,
    c_double,
    c_int,
    c_void_p,
    check_supported_control,
    check_supported_converter,
    value,
)
from motulator_export.plecs.grid import export_values

# Functions of the libraries setting the parameters of the model
GRID_API: Signatures = {
    "set_l_filter": (None, [c_void_p] + [c_double] * 4),
    "set_lcl_filter": (None, [c_void_p] + [c_double] * 7),
    "source_signal": (c_void_p, [c_void_p, c_int]),
}
# Functions of the libraries setting the parameters of the control systems
GFL_API: Signatures = {
    "set_controller": (None, [c_void_p] + [c_double] * 8),
    "power_reference": (c_void_p, [c_void_p]),
    "reactive_power_reference": (c_void_p, [c_void_p]),
    **PWM_API,
}
GFM_API: Signatures = {
    "set_controller": (None, [c_void_p] + [c_double] * 12),
    "power_reference": (c_void_p, [c_void_p]),
    "ac_voltage_reference": (c_void_p, [c_void_p]),
    **PWM_API,
}


class GridConverterNativeSystem(NativeSystem):
    """
    Grid converter system with grid-following or grid-forming control in the native
    library.

    Parameters
    ----------
    mdl : GridConverterSystem
        Continuous-time system model.
    ctrl : GridConverterControlSystem
        Discrete-time control system.

    """

    def __init__(self, mdl: Any, ctrl: Any) -> None:
        self.check_supported(mdl, ctrl)
        mdl = cast(GridConverterSystem, mdl)
        ctrl = cast(GridConverterControlSystem, ctrl)
        self.grid_forming = isinstance(
            ctrl.inner_ctrl, ObserverBasedGridFormingController
        )
        api = GFM_API if self.grid_forming else GFL_API
        super().__init__("gfm" if self.grid_forming else "gfl", {**GRID_API, **api})
        lib, h = self.lib, self.handle

        # System model
        f = mdl.ac_filter
        if isinstance(f, LCLFilter):
            lib.set_lcl_filter(h, f.L_fc, f.L_fg, f.C_f, f.R_fc, f.R_fg, f.L_g, f.R_g)
        else:
            lib.set_l_filter(h, f.L_f, f.R_f, f.L_g, f.R_g)
        self.set_converter(mdl)

        # Control system
        v = export_values(ctrl)
        if self.grid_forming:
            lib.set_controller(
                h,
                v["i_max"],
                v["L"],
                v["R"],
                value(v["R_a"]),
                value(v["k_v"]),
                v["alpha_o"],
                v["alpha_c"],
                v["u_nom"],
                v["w_nom"],
                v["T_s"],
                value(v["i_d_max"]),
                v["alpha_l"],
            )
        else:
            lib.set_controller(
                h,
                v["i_max"],
                v["L"],
                v["alpha_c"],
                value(v["alpha_i"]),
                v["u_nom"],
                v["w_nom"],
                v["alpha_pll"],
                v["T_s"],
            )
        self.set_pwm(ctrl.pwm)

    def set_signals(self, mdl: Any, ctrl: Any) -> None:
        """Set the signals of the source and the references."""
        lib, h = self.lib, self.handle
        src = cast(ThreePhaseSource, mdl.ac_source)
        for k, x in enumerate([src.w_g, src.e_g, src.phi, src.e_g_neg, src.phi_neg]):
            self.set_signal(lib.source_signal(h, k), x)
        ext_ref = ctrl.ext_ref
        if ext_ref.p_g is None:
            raise ValueError("Active power reference must be set")
        self.set_signal(lib.power_reference(h), ext_ref.p_g)
        if self.grid_forming:
            if ext_ref.v_c is None:
                raise ValueError("AC voltage reference must be set")
            self.set_signal(lib.ac_voltage_reference(h), ext_ref.v_c)
        else:
            if ext_ref.q_g is None:
                raise ValueError("Reactive power reference must be set")
            self.set_signal(lib.reactive_power_reference(h), ext_ref.q_g)

    @staticmethod
    def supports(mdl: Any, ctrl: Any) -> bool:
        """Whether the system is a grid converter system."""
        return isinstance(mdl, GridConverterSystem)

    @staticmethod
    def check_supported(mdl: Any, ctrl: Any) -> None:
        """Raise an error if the grid converter system is not supported."""
        if type(ctrl) is not GridConverterControlSystem:
            raise NotImplementedError("Only GridConverterControlSystem supported")
        inner = ctrl.inner_ctrl
        if type(inner) not in (
            CurrentVectorController,
            ObserverBasedGridFormingController,
        ):
            raise NotImplementedError(
                "Only CurrentVectorController and ObserverBasedGridFormingController "
                "supported"
            )
        if ctrl.dc_bus_voltage_ctrl is not None:
            raise NotImplementedError("DC-bus voltage control not supported")
        check_supported_control(ctrl, d_err=False)
        f = mdl.ac_filter
        if type(f) not in (LFilter, LCLFilter):
            raise NotImplementedError("Only LFilter and LCLFilter supported")
        if f.i_c_sensor is not None or f.u_g_sensor is not None:
            raise NotImplementedError("Sensor models not supported")
        if type(mdl.ac_source) is not ThreePhaseSource:
            raise NotImplementedError("Only ThreePhaseSource supported")
        check_supported_converter(mdl)
