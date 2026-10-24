"""
Systems of the tests: the builders of the system models and the control systems of
motulator, and the signals compared with those of the exported control systems.

"""

# %%
import importlib
from collections.abc import Callable
from math import inf, pi
from pathlib import Path
from typing import Any

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
import motulator.drive.gradnet as gn
import motulator.grid.control as grid_control
import motulator.grid.model as grid_model
import numpy as np
from motulator.drive import model, utils
from motulator.drive.control._base import VectorControlSystem, VHzControlSystem
from motulator.grid import utils as grid_utils


def blocked_drive(t: float) -> bool:
    """Gate enable of the drives with the gates blocked at the start and during the
    operation."""
    return 0.01 <= t < 0.15 or t >= 0.17


# Systems of the tests: the builders of the system model and the control system of
# motulator, the compared signals, and the inputs of the control-system blocks
Build = Callable[[], tuple[Any, Any]]
Signals = dict[str, Callable[[Any, Any], Any]]

SM_PAR = {"n_p": 3, "R_s": 3.6, "L_d": 0.036, "L_q": 0.051, "psi_f": 0.545}
SM_SPEED = {"J": 0.015, "alpha_s": 25.0}
IM_PAR = {"n_p": 2, "R_s": 3.7, "R_R": 2.1, "L_sgm": 0.021, "L_M": 0.224}
IM_SPEED = {"J": 0.015, "alpha_s": 2 * pi * 4}


def smooth_sign(x: np.ndarray) -> np.ndarray:
    """Smooth current-direction function of the dead-time error model."""
    return np.tanh(x / 0.2)


def sm_system(
    sensorless: bool, t_d: float, blocked: bool = False
) -> tuple[Build, Signals]:
    """
    Synchronous machine drive with flux-vector control, optionally with the gates
    blocked at the start and during the operation (`blocked`).
    """

    def build() -> tuple[model.Drive, VectorControlSystem]:
        par = model.SynchronousMachinePars(**SM_PAR)
        mdl = model.Drive(
            model.SynchronousMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 10.0)
        cfg = sm_control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=sensorless)
        pwm = sm_control.PWM()
        if t_d > 0:
            pwm = sm_control.PWM(
                d_err=sm_control.InverterNonlinearity(t_d, sign=smooth_sign)
            )
        ctrl = VectorControlSystem(
            sm_control.FluxVectorController(par, cfg),
            sm_control.SpeedController(**SM_SPEED),
            pwm,
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        if blocked:
            ctrl.set_enable(blocked_drive)
        return mdl, ctrl

    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_s_ref": lambda _, ref: ref.psi_s,
        "psi_s": lambda fbk, _: np.abs(fbk.psi_s),
    }
    return build, signals


# Minimum duty ratio of the tests of current-vector control of synchronous machines,
# above the smallest duty ratios at the speed reference
SM_CVC_D_MIN = 0.2


def sm_cvc_system(
    sensorless: bool,
    t_d: float,
    blocked: bool = False,
    discrete: bool = False,
    d_min: float = 0.0,
) -> tuple[Build, Signals]:
    """
    Synchronous machine drive of `sm_system` with current-vector control, optionally
    with the gates blocked at the start and during the operation (`blocked`), with
    the direct discrete-time current controller (`discrete`), and with the minimum
    duty ratio `d_min`.
    """

    def build() -> tuple[model.Drive, VectorControlSystem]:
        par = model.SynchronousMachinePars(**SM_PAR)
        mdl = model.Drive(
            model.SynchronousMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 10.0)
        cfg = sm_control.CurrentVectorControllerCfg(
            i_s_max=6.5, sensorless=sensorless, discrete=discrete
        )
        d_err = None
        if t_d > 0:
            d_err = sm_control.InverterNonlinearity(t_d, sign=smooth_sign)
        ctrl = VectorControlSystem(
            sm_control.CurrentVectorController(par, cfg),
            sm_control.SpeedController(**SM_SPEED),
            sm_control.PWM(d_err=d_err, d_min=d_min),
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        if blocked:
            ctrl.set_enable(blocked_drive)
        return mdl, ctrl

    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "i_d_ref": lambda _, ref: np.real(ref.i_s),
        "i_q_ref": lambda _, ref: np.imag(ref.i_s),
        "psi_s": lambda fbk, _: np.abs(fbk.psi_s),
    }
    return build, signals


def im_system(
    sensorless: bool,
    t_d: float,
    i_0: float,
    u_th: float = 0.0,
    single_update: bool = False,
    blocked: bool = False,
    discrete: bool = False,
    d_min: float = 0.0,
) -> tuple[Build, Signals]:
    """
    Induction machine drive with current-vector control, see `sm_system`, optionally
    with the direct discrete-time current controller (`discrete`) and the minimum
    duty ratio `d_min`.
    """
    par = model.InductionMachineInvGammaPars(**IM_PAR)

    def build() -> tuple[model.Drive, VectorControlSystem]:
        mdl = model.Drive(
            model.InductionMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d, u_th=u_th),
            delay=2 if single_update else 1,  # One sampling period
            single_update=single_update,
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 14.6)
        cfg = im_control.CurrentVectorControllerCfg(
            psi_s_nom=1.04, i_s_max=10.6, sensorless=sensorless, discrete=discrete
        )
        sign = np.sign if i_0 == 0 else lambda i: np.tanh(i / i_0)
        d_err = None
        if t_d > 0 or u_th > 0:
            d_err = im_control.InverterNonlinearity(t_d, u_th, sign)
        ctrl = VectorControlSystem(
            im_control.CurrentVectorController(par, cfg),
            im_control.SpeedController(**IM_SPEED),
            im_control.PWM(d_err=d_err, d_min=d_min, single_update=single_update),
        )
        ctrl.set_speed_ref(lambda t: (t > 0.05) * 2 * pi * 20)
        if blocked:
            ctrl.set_enable(blocked_drive)
        return mdl, ctrl

    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_R": lambda fbk, _: np.abs(fbk.psi_R),
    }
    return build, signals


def im_fvc_system(
    sensorless: bool,
    t_d: float = 0.0,
    blocked: bool = False,
    d_min: float = 0.0,
    alpha_i: float | None = None,
) -> tuple[Build, Signals]:
    """
    Induction machine drive of `im_system` with flux-vector control, optionally with
    the dead time and its compensation (`smooth_sign`), the gates blocked at the
    start and during the operation (`blocked`), the minimum duty ratio `d_min`, and
    the integral action (`alpha_i`, None for its default).
    """
    par = model.InductionMachineInvGammaPars(**IM_PAR)

    def build() -> tuple[model.Drive, VectorControlSystem]:
        mdl = model.Drive(
            model.InductionMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.2) * 14.6)
        cfg = im_control.FluxVectorControllerCfg(
            psi_s_nom=1.04, i_s_max=10.6, alpha_i=alpha_i, sensorless=sensorless
        )
        d_err = None
        if t_d > 0:
            d_err = im_control.InverterNonlinearity(t_d, sign=smooth_sign)
        ctrl = VectorControlSystem(
            im_control.FluxVectorController(par, cfg),
            im_control.SpeedController(**IM_SPEED),
            im_control.PWM(d_err=d_err, d_min=d_min),
        )
        # After the rotor flux has built up, see IM_FVC_RTOL
        ctrl.set_speed_ref(lambda t: (t > 0.1) * 2 * pi * 20)
        if blocked:
            ctrl.set_enable(blocked_drive)
        return mdl, ctrl

    signals = {
        "w_M": lambda fbk, _: fbk.w_M,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_s_ref": lambda _, ref: ref.psi_s,
        "psi_R": lambda fbk, _: np.abs(fbk.psi_R),
    }
    return build, signals


# Tolerances of flux-vector control of induction machines relative to the signal
# magnitudes (see `compare`) with the gates blocked: while the rotor flux builds up
# after the release, the current limit of the torque reference is the square root of
# a rounding error (as in V/Hz control, see VHZ_RTOL), which the voltage reference
# follows
IM_FVC_RTOL = {"tau_M_ref": 1e-6, "d_a": 1e-6, "d_b": 1e-6, "d_c": 1e-6}

IM_NOM = utils.NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
IM_BASE = utils.BaseValues.from_nominal(IM_NOM, n_p=2)
# Pure open-loop V/Hz control, as in plot_2kw_im_diode_vhz.py of motulator
OPEN_LOOP_PAR = {"n_p": 2, "R_s": 0, "R_R": 0, "L_sgm": 0, "L_M": inf}


def vhz_system(
    open_loop: bool, t_d: float, d_min: float, blocked: bool = False
) -> tuple[Build, Signals]:
    """
    Induction machine drive with observer-based or open-loop V/Hz control, see
    `sm_system`.
    """

    def build() -> tuple[model.Drive, VHzControlSystem]:
        mdl = model.Drive(
            model.InductionMachine(model.InductionMachineInvGammaPars(**IM_PAR)),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * 0.5 * IM_NOM.tau)
        if open_loop:
            par = im_control.InductionMachineInvGammaPars(**OPEN_LOOP_PAR)
            cfg = im_control.ObserverBasedVHzControllerCfg(
                psi_s_nom=IM_BASE.psi, i_s_max=inf, alpha_f=0, alpha_tau=0, alpha_psi=0
            )
        else:
            par = im_control.InductionMachineInvGammaPars(**IM_PAR)
            cfg = im_control.ObserverBasedVHzControllerCfg(
                psi_s_nom=IM_BASE.psi, i_s_max=1.5 * IM_BASE.i
            )
        d_err = None
        if t_d > 0:
            d_err = im_control.InverterNonlinearity(t_d, sign=smooth_sign)
        pwm = im_control.PWM(overmodulation="MME", d_err=d_err, d_min=d_min)
        ctrl = VHzControlSystem(
            im_control.ObserverBasedVHzController(par, cfg),
            slew_rate=2 * pi * 120,
            pwm=pwm,
        )
        ctrl.set_speed_ref(lambda t: (t > 0.1) * 0.8 * IM_BASE.w_M)
        if blocked:
            ctrl.set_enable(lambda t: 0.01 <= t < 0.7 or t >= 0.72)
        return mdl, ctrl

    signals = {
        "w_M_ref": lambda _, ref: ref.w_M,
        "w_s": lambda fbk, _: fbk.w_s,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_s_ref": lambda _, ref: ref.psi_s,
        "psi_s": lambda fbk, _: np.abs(fbk.psi_s),
    }
    return build, signals


SM_NOM = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
SM_BASE = utils.BaseValues.from_nominal(SM_NOM, n_p=3)


def sm_vhz_system(
    t_d: float, d_min: float, blocked: bool = False
) -> tuple[Build, Signals]:
    """
    Synchronous machine drive with observer-based V/Hz control (the 2.2-kW IPMSM of
    plot_2kw_ipmsm_ovhz.py of motulator), see `sm_system`.
    """

    def build() -> tuple[model.Drive, VHzControlSystem]:
        par = model.SynchronousMachinePars(**SM_PAR)
        mdl = model.Drive(
            model.SynchronousMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540, t_d=t_d),
        )
        mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * 0.5 * SM_NOM.tau)
        cfg = sm_control.ObserverBasedVHzControllerCfg(i_s_max=1.5 * SM_BASE.i)
        d_err = None
        if t_d > 0:
            d_err = sm_control.InverterNonlinearity(t_d, sign=smooth_sign)
        pwm = sm_control.PWM(overmodulation="MME", d_err=d_err, d_min=d_min)
        ctrl = VHzControlSystem(
            sm_control.ObserverBasedVHzController(par, cfg),
            slew_rate=2 * pi * 120,
            pwm=pwm,
        )
        ctrl.set_speed_ref(lambda t: (t > 0.1) * 0.8 * SM_BASE.w_M)
        if blocked:
            ctrl.set_enable(lambda t: 0.02 <= t < 0.7 or t >= 0.72)
        return mdl, ctrl

    signals = {
        "w_M_ref": lambda _, ref: ref.w_M,
        "w_c": lambda fbk, _: fbk.w_c,
        "tau_M_ref": lambda _, ref: ref.tau_M,
        "psi_s_ref": lambda _, ref: ref.psi_s,
        "psi_s": lambda fbk, _: np.abs(fbk.psi_s),
        "theta_m": lambda fbk, _: fbk.theta_m,
    }
    return build, signals


NOM = grid_utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
BASE = grid_utils.BaseValues.from_nominal(NOM)


def gfl_system(blocked: bool = False) -> tuple[Build, Signals]:
    """Grid converter system with grid-following control, see `sm_system`."""

    def build() -> tuple[Any, grid_control.GridConverterControlSystem]:
        ac_filter = grid_model.LCLFilter(
            L_fc=0.073 * BASE.L, L_fg=0.073 * BASE.L, C_f=0.043 * BASE.C, u_f0_ab=BASE.u
        )
        mdl = grid_model.GridConverterSystem(
            grid_model.VoltageSourceConverter(u_dc=650),
            ac_filter,
            grid_model.ThreePhaseSource(w_g=BASE.w, e_g=BASE.u),
        )
        cfg = grid_control.CurrentVectorControllerCfg(
            i_max=1.5 * BASE.i, L=0.073 * BASE.L, T_s=100e-6
        )
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.CurrentVectorController(cfg)
        )
        ctrl.set_power_ref(lambda t: (t > 0.02) * 5e3)
        ctrl.set_reactive_power_ref(lambda t: (t > 0.04) * 4e3)
        if blocked:
            ctrl.set_enable(lambda t: 0.005 <= t < 0.03 or t >= 0.035)
        return mdl, ctrl

    signals = {
        "p_g": lambda fbk, _: fbk.p_g,
        "q_g": lambda fbk, _: fbk.q_g,
        "w_g": lambda fbk, _: fbk.w_g,
        "i_c_d_ref": lambda _, ref: ref.i_c.real,
    }
    return build, signals


def gfm_system(power_limitation: bool, blocked: bool = False) -> tuple[Build, Signals]:
    """Grid converter system with grid-forming control, see `sm_system`."""

    def build() -> tuple[Any, grid_control.GridConverterControlSystem]:
        ac_filter = grid_model.LFilter(
            L_f=0.15 * BASE.L, R_f=0.05 * BASE.Z, L_g=0.74 * BASE.L
        )
        mdl = grid_model.GridConverterSystem(
            grid_model.VoltageSourceConverter(u_dc=650),
            ac_filter,
            grid_model.ThreePhaseSource(w_g=BASE.w, e_g=BASE.u),
        )
        cfg = grid_control.ObserverBasedGridFormingControllerCfg(
            i_max=1.3 * BASE.i,
            L=0.35 * BASE.L,
            R=0.05 * BASE.Z,
            R_a=0.2 * BASE.Z,
            u_nom=BASE.u,
            w_nom=BASE.w,
            i_d_max=0.85 * 1.3 * BASE.i if power_limitation else None,
        )
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.ObserverBasedGridFormingController(cfg)
        )
        ctrl.set_power_ref(lambda t: (t > 0.05) * NOM.P - (t > 0.15) * 2 * NOM.P)
        ctrl.set_ac_voltage_ref(BASE.u)
        if blocked:
            ctrl.set_enable(lambda t: 0.01 <= t < 0.1 or t >= 0.11)
        return mdl, ctrl

    signals = {
        "p_g": lambda fbk, _: fbk.p_g,
        "q_g": lambda fbk, _: fbk.q_g,
        "v_c": lambda fbk, _: np.abs(fbk.v_c),
        "theta_c": lambda fbk, _: fbk.theta_c,
    }
    return build, signals


# Cases of V/Hz control: open loop, the dead time, the minimum duty ratio, and the
# gates blocked
VHZ_CASES = [
    (False, 0.0, 0.0, False),
    (True, 0.0, 0.0, False),
    (False, 2e-6, 0.0, False),
    (True, 2e-6, 0.04, False),
    (False, 0.0, 0.0, True),
]
VHZ_IDS = [
    "ovhz",
    "open_loop",
    "ovhz_dead_time",
    "open_loop_dead_time_min_pulse",
    "ovhz_blocked",
]
# Cases of V/Hz control of synchronous machines: the dead time, the minimum duty ratio
# (above the smallest duty ratios at the speed reference), and the gates blocked. The
# dead time of the system model makes it sensitive to the rounding errors at the zero
# crossings of the currents, so it is not combined with the minimum pulses.
SM_VHZ_CASES = [
    (0.0, 0.0, False),
    (2e-6, 0.0, False),
    (0.0, 0.15, False),
    (0.0, 0.0, True),
]
SM_VHZ_IDS = ["smvhz", "smvhz_dead_time", "smvhz_min_pulse", "smvhz_blocked"]
# Tolerances of V/Hz control relative to the signal magnitudes (see `compare`): with
# the flux reference at its current limit (e.g., after the release of the gates), the
# current limit of the torque reference is the square root of a rounding error
VHZ_RTOL = {"tau_M_ref": 1e-6}


MODEL_DIR = Path(__file__).parents[1] / "examples" / "trained_models"
FLUX_MAP = MODEL_DIR / "baldor_fem_flux_map_pnorm_d12_sub20.npz"
CURRENT_MAP = MODEL_DIR / "baldor_fem_curr_map_harm_softmax_d48_sub10.npz"


def gradnet_control_system() -> VectorControlSystem:
    """Flux-vector control with a GradNet flux map."""
    flux_map = gn.FluxMap(gn.load_gradnet(FLUX_MAP))
    par = sm_control.SaturatedSynchronousMachinePars(
        n_p=2, R_s=0.63, psi_s_dq_fcn=flux_map
    )
    cfg = sm_control.FluxVectorControllerCfg(
        i_s_max=17.6, alpha_i=0, alpha_o=2 * pi * 8, J=0.05, sensorless=False
    )
    ctrl = VectorControlSystem(
        sm_control.FluxVectorController(par, cfg),
        sm_control.SpeedController(J=0.05, alpha_s=2 * pi * 4),
    )
    return ctrl


# %%
# Systems of the examples (the scripts of examples/)
EXAMPLES = Path(__file__).parents[1] / "examples"
EXAMPLE_SYSTEMS = [
    "ipmsm_2kw_fvc",
    "ipmsm_2kw_vhz",
    "ipmsm_2kw_cvc",
    "ipmsm_2kw_cvc_discrete",
    "pmsyrm_6kw_gn_fvc",
    "im_2kw_cvc",
    "im_2kw_fvc",
    "im_2kw_vhz",
    "im_2kw_vhz_open_loop",
    "gfl_10kva_lcl",
    "gfm_13kva_do",
]


def example_system(name: str) -> tuple[Any, Any, float]:
    """
    The system of an example with its references (as the script sets them), and its
    stop time. The folder `EXAMPLES` must be on the path, since the scripts import
    their helpers. The suffix _open_loop or _discrete of the name selects the option
    of the script (e.g., --open-loop), the argument of its `build_system`.
    """
    suffix = next((s for s in ("_open_loop", "_discrete") if name.endswith(s)), "")
    ex = importlib.import_module(name.removesuffix(suffix))
    mdl, ctrl = ex.build_system(True) if suffix else ex.build_system()
    if hasattr(ex, "W_M_REF"):
        ctrl.set_speed_ref(ex.W_M_REF)
        mdl.mechanics.set_external_load_torque(ex.TAU_L)
    elif hasattr(ex, "Q_G_REF"):
        ctrl.set_power_ref(ex.P_G_REF)
        ctrl.set_reactive_power_ref(ex.Q_G_REF)
    else:
        ctrl.set_ac_voltage_ref(ex.V_C_REF)
        ctrl.set_power_ref(ex.P_G_REF)
    return mdl, ctrl, ex.T_STOP
