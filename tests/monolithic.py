"""
Monolithic control systems: the reference of the tests of the control systems.

The control systems of the package are modular: each class of motulator is a block
(see `motulator_export.plecs._control`). Here, each control system is instead one
C-Script block, which runs the whole-system functions of the C port (e.g.,
`vector_control_system_compute_output` of `c/sm_flux_vector.c`) with the parameters
of the motulator API. Its S-function is compared with motulator in
`test_simulink.py`, and the modular control systems are compared with it exactly in
`test_control.py`.

"""

import re
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from motulator_export.plecs import (
    _grid_control,
    _im_control,
    _im_vhz_control,
    _sm_control,
)
from motulator_export.plecs._common import (
    C_DIR,
    C_GRADNET_PARAMS,
    C_PARAMS,
    C_U_DC_MIN,
    GRADNET_FIELDS,
    MaskParam,
    monitored_code,
)
from motulator_export.simulink._sfunction import SFunction


@dataclass
class MonolithicBlock:
    """Control system as one C-Script block with the parameters of the motulator API."""

    name: str
    inputs: list[str]
    input_widths: list[int]
    outputs: dict[str, list[str]]  # Monitored signals: probe name -> signal names
    mask_params: list[MaskParam]
    code: Callable[[], dict[str, str]]  # Code sections of the C-Script
    cscript_params: list[str] | None = None  # Defaults to the mask variables

    @property
    def signals(self) -> list[str]:
        """Names of the monitored signals."""
        return [n for v in self.outputs.values() for n in v]


# %%
# C code helpers
DUTY_RATIO_CODE = (
    "/* Duty ratios, delayed by the Delay block outside the subsystem */\n"
    "for (int k = 0; k < 3; k++) {\n"
    "    OutputSignal(0, k) = ctrl.ref.d_abc[k];\n"
    "}\n"
    "\n"
)


def enable_code(code: dict[str, str], outputs: dict[str, list[str]]) -> dict[str, str]:
    """
    Add the input `enable` (the first input) to the code sections of a C-Script.

    While the input is not positive, the control algorithm is not run: the duty ratios
    are 0.5 (zero voltage), the monitored signals are zero, and the state `ctrl` is
    reset to its initial value.

    """
    match = re.search(r"^static (\w+) ctrl;$", code["Declarations"], re.M)
    if match is None:
        raise RuntimeError("State of the control system (ctrl) not found")
    disabled = "if (!(InputSignal(0, 0) > 0.0)) {\n"
    zeros = "".join(
        f"    OutputSignal({i_out + 1}, {j}) = 0.0;\n"
        for i_out, names in enumerate(outputs.values())
        for j in range(len(names))
    )
    return code | {
        "Declarations": code["Declarations"]
        + "\n/* Initial state, restored while the control system is disabled */\n"
        f"static {match.group(1)} ctrl_init;\n",
        "StartFcn": code["StartFcn"] + "\nctrl_init = ctrl;\n",
        "OutputFcn": "/* Disabled: zero voltage, the control algorithm is not run */\n"
        + disabled
        + "    for (int k = 0; k < 3; k++) {\n"
        "        OutputSignal(0, k) = 0.5;\n"
        "    }\n" + zeros + "    return;\n"
        "}\n"
        "\n" + code["OutputFcn"],
        "UpdateFcn": "/* Disabled: reset the state */\n"
        + disabled
        + "    ctrl = ctrl_init;\n"
        "    return;\n"
        "}\n"
        "\n" + code["UpdateFcn"],
    }


def parameter_checks(mask_params: list[MaskParam]) -> str:
    """C code checking that the required mask parameters are scalars."""
    return "".join(
        f"if (PDIM({k}) != 1) {{\n"
        f'    SetErrorMessage("{m.variable} must be a scalar.");\n'
        "    return;\n"
        "}\n"
        for k, m in enumerate(mask_params)
        if m.required
    )


def cfg_assignments(mask_params: list[MaskParam]) -> str:
    """C code assigning the non-empty mask parameters to the configuration."""
    return "".join(
        f"if (PDIM({k}) > 0) {{\n    {m.target} = P({k}, 0);\n}}\n"
        for k, m in enumerate(mask_params)
        if m.target.startswith("cfg.")
    )


def _declarations(sources: list[str], state: str) -> str:
    """Declarations of a C-Script of a control system."""
    return (
        "/* Monolithic control system of the tests. */\n"
        + "".join(f'#include "{C_DIR}/{f}"\n' for f in sources)
        + "\n"
        + C_PARAMS
        + "\n"
        + C_U_DC_MIN
        + "\n"
        + (C_GRADNET_PARAMS + "\n" if "gradnet.c" in sources else "")
        + f"static {state} ctrl;\n"
    )


# %%
# Speed controller and PWM of the drives
TAB_SPEED = "Speed control (SpeedController)"
SPEED_MASK_PARAMS = [
    MaskParam("speed_J", "J: Total inertia (kgm²)", TAB_SPEED, "", True),
    MaskParam("speed_alpha_s", "alpha_s", TAB_SPEED, "", True),
    MaskParam("speed_alpha_i", "alpha_i", TAB_SPEED, ""),
    MaskParam("speed_tau_M_max", "tau_M_max", TAB_SPEED, ""),
]
TAB_PWM = "PWM (PWM)"
PWM_MASK_PARAMS = [
    MaskParam("pwm_t_d", "t_d", TAB_PWM, "", True),
    MaskParam("pwm_i_0", "i_0", TAB_PWM, "", True),
    MaskParam("pwm_feedforward", "feedforward", TAB_PWM, "", True),
    MaskParam("pwm_d_min", "d_min", TAB_PWM, "", True),
]


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


def pwm_code(i: dict[str, int]) -> str:
    """C code setting the duty-ratio error model and the minimum pulses of the PWM."""
    return (
        "/* Duty-ratio error model of the PWM (dead_time_error) */\n"
        f"pwm_set_dead_time(&ctrl.pwm, P({i['pwm_t_d']}, 0), P({i['T_s']}, 0),\n"
        f"                  P({i['pwm_i_0']}, 0), (int)P({i['pwm_feedforward']}, 0));\n"
        "/* Minimum duty ratio of a switching leg */\n"
        f"pwm_set_min_pulse(&ctrl.pwm, P({i['pwm_d_min']}, 0));\n"
    )


def _drive_output(meas_type: str, compute: str) -> str:
    """C code of the measurements and the control algorithm of a drive."""
    return (
        "/* Measurements */\n"
        "double w_M_ref = InputSignal(1, 0);\n"
        "double i_s_abc[3] = {InputSignal(2, 0), InputSignal(2, 1),\n"
        "                     InputSignal(2, 2)};\n"
        "double u_dc = fmax(InputSignal(3, 0), U_DC_MIN);\n"
        f"{meas_type} meas = {{abc2complex(i_s_abc), u_dc, InputSignal(4, 0)}};\n"
        "\n"
        f"{compute}(&ctrl, &meas, w_M_ref);\n"
        "\n" + DUTY_RATIO_CODE
    )


# %%
# Flux-vector control of synchronous machine drives
TAB = "Flux-vector control"
SM_MASK_PARAMS = [
    MaskParam("n_p", "n_p", TAB, "", True),
    MaskParam("R_s", "R_s", TAB, "", True),
    MaskParam("psi_s_dq_fcn", "psi_s_dq_fcn", TAB, ""),
    MaskParam("L_d", "L_d", TAB, ""),
    MaskParam("L_q", "L_q", TAB, ""),
    MaskParam("psi_f", "psi_f", TAB, ""),
    MaskParam("i_s_max", "i_s_max", TAB, "", True),
    MaskParam("alpha_tau", "alpha_tau", TAB, "cfg.alpha_tau"),
    MaskParam("alpha_psi", "alpha_psi", TAB, "cfg.alpha_psi"),
    MaskParam("alpha_i", "alpha_i", TAB, "cfg.alpha_i"),
    MaskParam("alpha_o", "alpha_o", TAB, "cfg.alpha_o"),
    MaskParam("k_o", "k_o", TAB, ""),
    MaskParam("psi_s_min", "psi_s_min", TAB, "cfg.psi_s_min"),
    MaskParam("psi_s_max", "psi_s_max", TAB, "cfg.psi_s_max"),
    MaskParam("k_u", "k_u", TAB, "cfg.k_u"),
    MaskParam("k_mtpv", "k_mtpv", TAB, "cfg.k_mtpv"),
    MaskParam("J", "J", TAB, "cfg.J"),
    MaskParam("sensorless", "sensorless", TAB, "cfg.sensorless"),
    MaskParam("T_s", "T_s", TAB, "cfg.T_s", True),
    *SPEED_MASK_PARAMS,
    *PWM_MASK_PARAMS,
]


def _sm_code() -> dict[str, str]:
    """Code sections of the C-Script of flux-vector control."""
    sources = ["common.c", "gradnet.c", "sm_parameters.c", "sm_control_loci.c"]
    declarations = _declarations([*sources, "sm_flux_vector.c"], "VectorControlSystem")
    i = {m.variable: k for k, m in enumerate(SM_MASK_PARAMS)}
    i_gn = len(SM_MASK_PARAMS)  # The GradNet parameters follow the mask parameters
    k_o = i["k_o"]
    L_d, L_q, psi_f = i["L_d"], i["L_q"], i["psi_f"]
    start = (
        parameter_checks(SM_MASK_PARAMS)
        + f"if (PDIM({k_o}) != 0 && PDIM({k_o}) != 2) {{\n"
        '    SetErrorMessage("k_o must be [] or [k0 k1].");\n'
        "    return;\n"
        "}\n"
        "SynchronousMachinePars par;\n"
        f"if (PDIM({i_gn}) > 0) {{\n"
        "    GradNet flux_map;\n"
        f"    CHECK_GRADNET({i_gn});\n"
        f"    READ_GRADNET(flux_map, {i_gn});\n"
        "    par = saturated_synchronous_machine_pars(\n"
        f"        P({i['n_p']}, 0), P({i['R_s']}, 0), &flux_map);\n"
        "} else {\n"
        f"    if (PDIM({L_d}) != 1 || PDIM({L_q}) != 1 || PDIM({psi_f}) != 1) {{\n"
        '        SetErrorMessage("L_d, L_q, and psi_f needed without a flux map.");\n'
        "        return;\n"
        "    }\n"
        "    par = synchronous_machine_pars(\n"
        f"        P({i['n_p']}, 0), P({i['R_s']}, 0), P({L_d}, 0), P({L_q}, 0),\n"
        f"        P({psi_f}, 0));\n"
        "}\n"
        "FluxVectorControllerCfg cfg = "
        f"flux_vector_controller_cfg(P({i['i_s_max']}, 0));\n"
        + cfg_assignments(SM_MASK_PARAMS)
        + f"if (PDIM({k_o}) == 2) {{\n"
        f"    cfg.k_o[0] = P({k_o}, 0);\n"
        f"    cfg.k_o[1] = P({k_o}, 1);\n"
        "}\n" + speed_controller_code(i) + "\n"
        "vector_control_system_init(&ctrl, par, &cfg, speed_ctrl);\n" + pwm_code(i)
    )
    monitored = {
        "w_M_ref": "ctrl.ref.w_M",
        "w_M": "ctrl.fbk.w_M",
        "tau_M_ref": "ctrl.ref.tau_M",
        "tau_M": "ctrl.fbk.tau_M",
        "psi_s_ref": "ctrl.ref.psi_s",
        "psi_s": "cabs(ctrl.fbk.psi_s)",
        "theta_m": "ctrl.fbk.theta_m",
        "i_d": "creal(ctrl.fbk.i_s)",
        "i_q": "cimag(ctrl.fbk.i_s)",
    }
    output = _drive_output(
        "Measurements", "vector_control_system_compute_output"
    ) + monitored_code(_sm_control.CTRL_OUTPUTS, monitored)
    code = {
        "Declarations": declarations,
        "StartFcn": start,
        "OutputFcn": output,
        "UpdateFcn": "vector_control_system_update(&ctrl);\n",
    }
    return enable_code(code, _sm_control.CTRL_OUTPUTS)


def _names(ports: list[Any]) -> list[str]:
    return [p.name for p in ports]


def _widths(ports: list[Any]) -> list[int]:
    return [p.width for p in ports]


FVC = MonolithicBlock(
    name="Flux-vector control",
    inputs=_names(_sm_control.CTRL_INPUTS),
    input_widths=_widths(_sm_control.CTRL_INPUTS),
    outputs=_sm_control.CTRL_OUTPUTS,
    mask_params=SM_MASK_PARAMS,
    code=_sm_code,
    # The flux map (a struct) is passed via the gn_* parameters
    cscript_params=[
        f"isempty({m.variable})" if m.variable == "psi_s_dq_fcn" else m.variable
        for m in SM_MASK_PARAMS
    ]
    + [f"gn_{f}" for f in GRADNET_FIELDS],
)


# %%
# Current-vector control of induction machine drives
TAB = "Current-vector control"
IM_MASK_PARAMS = [
    MaskParam("n_p", "n_p", TAB, "", True),
    MaskParam("R_s", "R_s", TAB, "", True),
    MaskParam("R_R", "R_R", TAB, "", True),
    MaskParam("L_sgm", "L_sgm", TAB, "", True),
    MaskParam("L_M", "L_M", TAB, "", True),
    MaskParam("psi_s_nom", "psi_s_nom", TAB, "", True),
    MaskParam("i_s_max", "i_s_max", TAB, "", True),
    MaskParam("alpha_c", "alpha_c", TAB, "cfg.alpha_c"),
    MaskParam("alpha_i", "alpha_i", TAB, "cfg.alpha_i"),
    MaskParam("alpha_o", "alpha_o", TAB, "cfg.alpha_o"),
    MaskParam("w_s_nom", "w_s_nom", TAB, "cfg.w_s_nom"),
    MaskParam("k_u", "k_u", TAB, "cfg.k_u"),
    MaskParam("k_fw", "k_fw", TAB, "cfg.k_fw"),
    MaskParam("J", "J", TAB, "cfg.J"),
    MaskParam("sensorless", "sensorless", TAB, "cfg.sensorless"),
    MaskParam("T_s", "T_s", TAB, "cfg.T_s", True),
    *SPEED_MASK_PARAMS,
    *PWM_MASK_PARAMS,
]


def _im_code() -> dict[str, str]:
    """Code sections of the C-Script of current-vector control."""
    declarations = _declarations(
        ["common.c", "im_current_vector.c"], "IMVectorControlSystem"
    )
    i = {m.variable: k for k, m in enumerate(IM_MASK_PARAMS)}
    start = (
        parameter_checks(IM_MASK_PARAMS) + "InductionMachineInvGammaPars par = {\n"
        f"    P({i['n_p']}, 0), P({i['R_s']}, 0), P({i['R_R']}, 0), "
        f"P({i['L_sgm']}, 0), P({i['L_M']}, 0)}};\n"
        "IMCurrentVectorControllerCfg cfg = im_current_vector_controller_cfg(\n"
        f"    P({i['psi_s_nom']}, 0), P({i['i_s_max']}, 0));\n"
        + cfg_assignments(IM_MASK_PARAMS)
        + speed_controller_code(i)
        + "im_vector_control_system_init(&ctrl, par, &cfg, speed_ctrl);\n"
        + pwm_code(i)
    )
    monitored = {
        "w_M_ref": "ctrl.ref.w_M",
        "w_M": "ctrl.fbk.w_M",
        "tau_M_ref": "ctrl.ref.tau_M",
        "tau_M": "ctrl.fbk.tau_M",
        "psi_s": "cabs(ctrl.fbk.psi_s)",
        "psi_R": "cabs(ctrl.fbk.psi_R)",
        "i_sd_ref": "creal(rot * ctrl.ref.i_s)",
        "i_sd": "creal(rot * ctrl.fbk.i_s)",
        "i_sq_ref": "cimag(rot * ctrl.ref.i_s)",
        "i_sq": "cimag(rot * ctrl.fbk.i_s)",
    }
    output = _drive_output(
        "IMMeasurements", "im_vector_control_system_compute_output"
    ) + monitored_code(
        _im_control.CTRL_OUTPUTS,
        monitored,
        prelude="double complex rot = cexp(-I * carg(ctrl.fbk.psi_R));\n",
    )
    code = {
        "Declarations": declarations,
        "StartFcn": start,
        "OutputFcn": output,
        "UpdateFcn": "im_vector_control_system_update(&ctrl);\n",
    }
    return enable_code(code, _im_control.CTRL_OUTPUTS)


CVC = MonolithicBlock(
    name="Current-vector control",
    inputs=_names(_im_control.CTRL_INPUTS),
    input_widths=_widths(_im_control.CTRL_INPUTS),
    outputs=_im_control.CTRL_OUTPUTS,
    mask_params=IM_MASK_PARAMS,
    code=_im_code,
)


# %%
# Observer-based V/Hz control of induction machine drives
TAB = "Observer-based VHz control"
VHZ_MASK_PARAMS = [
    MaskParam("n_p", "n_p", TAB, "", True),
    MaskParam("R_s", "R_s", TAB, "", True),
    MaskParam("R_R", "R_R", TAB, "", True),
    MaskParam("L_sgm", "L_sgm", TAB, "", True),
    MaskParam("L_M", "L_M", TAB, "", True),
    MaskParam("psi_s_nom", "psi_s_nom", TAB, "", True),
    MaskParam("i_s_max", "i_s_max", TAB, "", True),
    MaskParam("alpha_psi", "alpha_psi", TAB, "cfg.alpha_psi"),
    MaskParam("alpha_tau", "alpha_tau", TAB, "cfg.alpha_tau"),
    MaskParam("alpha_f", "alpha_f", TAB, "cfg.alpha_f"),
    MaskParam("k_u", "k_u", TAB, "cfg.k_u"),
    MaskParam("k_b", "k_b", TAB, "cfg.k_b"),
    MaskParam("T_s", "T_s", TAB, "cfg.T_s", True),
    MaskParam("slew_rate", "slew_rate", TAB, "", True),
    *PWM_MASK_PARAMS,
]


def _vhz_code() -> dict[str, str]:
    """Code sections of the C-Script of observer-based V/Hz control."""
    declarations = _declarations(
        ["common.c", "im_current_vector.c", "im_flux_vector.c"], "IMVHzControlSystem"
    )
    i = {m.variable: k for k, m in enumerate(VHZ_MASK_PARAMS)}
    start = (
        parameter_checks(VHZ_MASK_PARAMS) + "InductionMachineInvGammaPars par = {\n"
        f"    P({i['n_p']}, 0), P({i['R_s']}, 0), P({i['R_R']}, 0), "
        f"P({i['L_sgm']}, 0), P({i['L_M']}, 0)}};\n"
        "IMVHzControllerCfg cfg = im_vhz_controller_cfg(\n"
        f"    P({i['psi_s_nom']}, 0), P({i['i_s_max']}, 0));\n"
        + cfg_assignments(VHZ_MASK_PARAMS)
        + f"im_vhz_control_system_init(&ctrl, par, &cfg, P({i['slew_rate']}, 0));\n"
        + pwm_code(i)
    )
    monitored = {
        "w_M_ref": "ctrl.ref.w_M",
        "w_s": "ctrl.fbk.w_s",
        "tau_M_ref": "ctrl.ref.tau_M",
        "tau_M": "ctrl.fbk.tau_M",
        "psi_s_ref": "ctrl.ref.psi_s",
        "psi_s": "cabs(ctrl.fbk.psi_s)",
        "psi_R": "cabs(ctrl.fbk.psi_R)",
        "i_sd": "creal(rot * ctrl.fbk.i_s)",
        "i_sq": "cimag(rot * ctrl.fbk.i_s)",
    }
    output = (
        "double w_M_ref = InputSignal(1, 0);\n"
        "double i_s_abc[3] = {InputSignal(2, 0), InputSignal(2, 1),\n"
        "                     InputSignal(2, 2)};\n"
        "double u_dc = fmax(InputSignal(3, 0), U_DC_MIN);\n"
        "IMVHzMeasurements meas = {abc2complex(i_s_abc), u_dc};\n"
        "im_vhz_control_system_compute_output(&ctrl, &meas, w_M_ref);\n"
        + DUTY_RATIO_CODE
        + monitored_code(
            _im_vhz_control.CTRL_OUTPUTS,
            monitored,
            prelude="double complex rot = cexp(-I * carg(ctrl.fbk.psi_R));\n",
        )
    )
    code = {
        "Declarations": declarations,
        "StartFcn": start,
        "OutputFcn": output,
        "UpdateFcn": "im_vhz_control_system_update(&ctrl);\n",
    }
    return enable_code(code, _im_vhz_control.CTRL_OUTPUTS)


VHZ = MonolithicBlock(
    name="Observer-based VHz control",
    inputs=_names(_im_vhz_control.CTRL_INPUTS),
    input_widths=_widths(_im_vhz_control.CTRL_INPUTS),
    outputs=_im_vhz_control.CTRL_OUTPUTS,
    mask_params=VHZ_MASK_PARAMS,
    code=_vhz_code,
)


# %%
# Grid-following and grid-forming control
TAB = "Grid-following control"
GFL_MASK_PARAMS = [
    MaskParam("i_max", "i_max", TAB, "", True),
    MaskParam("L", "L", TAB, "", True),
    MaskParam("alpha_c", "alpha_c", TAB, "cfg.alpha_c"),
    MaskParam("alpha_i", "alpha_i", TAB, ""),
    MaskParam("u_nom", "u_nom", TAB, "cfg.u_nom"),
    MaskParam("w_nom", "w_nom", TAB, "cfg.w_nom"),
    MaskParam("alpha_pll", "alpha_pll", TAB, "cfg.alpha_pll"),
    MaskParam("T_s", "T_s", TAB, "cfg.T_s", True),
]
TAB = "Grid-forming control"
GFM_MASK_PARAMS = [
    MaskParam("i_max", "i_max", TAB, "", True),
    MaskParam("L", "L", TAB, "", True),
    MaskParam("R", "R", TAB, "cfg.R"),
    MaskParam("R_a", "R_a", TAB, "cfg.R_a"),
    MaskParam("k_v", "k_v", TAB, "cfg.k_v"),
    MaskParam("alpha_o", "alpha_o", TAB, "cfg.alpha_o"),
    MaskParam("alpha_c", "alpha_c", TAB, "cfg.alpha_c"),
    MaskParam("u_nom", "u_nom", TAB, "cfg.u_nom"),
    MaskParam("w_nom", "w_nom", TAB, "cfg.w_nom"),
    MaskParam("T_s", "T_s", TAB, "cfg.T_s", True),
    MaskParam("i_d_max", "i_d_max", TAB, "cfg.i_d_max"),
    MaskParam("alpha_l", "alpha_l", TAB, "cfg.alpha_l"),
]


def _gfl_code() -> dict[str, str]:
    """Code sections of the C-Script of grid-following control."""
    i = {m.variable: k for k, m in enumerate(GFL_MASK_PARAMS)}
    start = (
        parameter_checks(GFL_MASK_PARAMS)
        + f"GFLControllerCfg cfg = gfl_controller_cfg(P({i['i_max']}, 0), "
        f"P({i['L']}, 0));\n"
        + cfg_assignments(GFL_MASK_PARAMS)
        + f"cfg.alpha_i = PDIM({i['alpha_i']}) > 0 ? P({i['alpha_i']}, 0) : "
        "cfg.alpha_c;\n"
        "gfl_control_system_init(&ctrl, &cfg);\n"
    )
    output = (
        "double i_c_abc[3] = {InputSignal(3, 0), InputSignal(3, 1),\n"
        "                     InputSignal(3, 2)};\n"
        "double u_g_line[2] = {InputSignal(4, 0), InputSignal(4, 1)};\n"
        "double u_dc = fmax(InputSignal(5, 0), U_DC_MIN);\n"
        "GridMeasurements meas = {abc2complex(i_c_abc), line2complex(u_g_line),\n"
        "                         u_dc};\n"
        "gfl_control_system_compute_output(&ctrl, &meas, InputSignal(1, 0),\n"
        "                                  InputSignal(2, 0));\n" + DUTY_RATIO_CODE
    )
    monitored = {
        "p_g_ref": "ctrl.ref.p_g",
        "p_g": "ctrl.fbk.p_g",
        "q_g_ref": "ctrl.ref.q_g",
        "q_g": "ctrl.fbk.q_g",
        "i_c_d_ref": "creal(ctrl.ref.i_c)",
        "i_c_d": "creal(ctrl.fbk.i_c)",
        "i_c_q_ref": "cimag(ctrl.ref.i_c)",
        "i_c_q": "cimag(ctrl.fbk.i_c)",
        "u_g": "ctrl.fbk.u_g",
        "w_g": "ctrl.fbk.w_g",
        "theta_c": "ctrl.fbk.theta_c",
    }
    code = {
        "Declarations": _declarations(
            ["common.c", "gfl_current_vector.c"], "GFLControlSystem"
        ),
        "StartFcn": start,
        "OutputFcn": output + monitored_code(_grid_control.GFL_OUTPUTS, monitored),
        "UpdateFcn": "gfl_control_system_update(&ctrl);\n",
    }
    return enable_code(code, _grid_control.GFL_OUTPUTS)


def _gfm_code() -> dict[str, str]:
    """Code sections of the C-Script of grid-forming control."""
    i = {m.variable: k for k, m in enumerate(GFM_MASK_PARAMS)}
    start = (
        parameter_checks(GFM_MASK_PARAMS)
        + f"GFMControllerCfg cfg = gfm_controller_cfg(P({i['i_max']}, 0), "
        f"P({i['L']}, 0));\n"
        + cfg_assignments(GFM_MASK_PARAMS)
        + "gfm_control_system_init(&ctrl, &cfg);\n"
    )
    output = (
        "double i_c_abc[3] = {InputSignal(3, 0), InputSignal(3, 1),\n"
        "                     InputSignal(3, 2)};\n"
        "double u_dc = fmax(InputSignal(4, 0), U_DC_MIN);\n"
        "GFMMeasurements meas = {abc2complex(i_c_abc), u_dc};\n"
        "gfm_control_system_compute_output(&ctrl, &meas, InputSignal(1, 0),\n"
        "                                  InputSignal(2, 0));\n" + DUTY_RATIO_CODE
    )
    monitored = {
        "p_g_ref": "ctrl.ref.p_g",
        "p_g": "ctrl.fbk.p_g",
        "q_g": "ctrl.fbk.q_g",
        "v_c_ref": "ctrl.ref.v_c",
        "v_c": "cabs(ctrl.fbk.v_c)",
        "i_c_d_ref": "creal(ctrl.ref.i_c)",
        "i_c_d": "creal(ctrl.fbk.i_c)",
        "i_c_q_ref": "cimag(ctrl.ref.i_c)",
        "i_c_q": "cimag(ctrl.fbk.i_c)",
        "theta_c": "ctrl.fbk.theta_c",
    }
    code = {
        "Declarations": _declarations(
            ["common.c", "gfm_observer.c"], "GFMControlSystem"
        ),
        "StartFcn": start,
        "OutputFcn": output + monitored_code(_grid_control.GFM_OUTPUTS, monitored),
        "UpdateFcn": "gfm_control_system_update(&ctrl);\n",
    }
    return enable_code(code, _grid_control.GFM_OUTPUTS)


GFL = MonolithicBlock(
    name="Grid-following control",
    inputs=_names(_grid_control.GFL_INPUTS),
    input_widths=_widths(_grid_control.GFL_INPUTS),
    outputs=_grid_control.GFL_OUTPUTS,
    mask_params=GFL_MASK_PARAMS,
    code=_gfl_code,
)
GFM = MonolithicBlock(
    name="Grid-forming control",
    inputs=_names(_grid_control.GFM_INPUTS),
    input_widths=_widths(_grid_control.GFM_INPUTS),
    outputs=_grid_control.GFM_OUTPUTS,
    mask_params=GFM_MASK_PARAMS,
    code=_gfm_code,
)


# %%
def control_sfunction(block: MonolithicBlock) -> SFunction:
    """
    S-function of a monolithic control system.

    The inputs are those of the block, and the outputs are the duty ratios and the
    groups of the monitored signals. The parameters are the C-Script parameters,
    and the sample time is the parameter `T_s`.

    """
    params = block.cscript_params or [m.variable for m in block.mask_params]
    name = "sfun_" + re.sub(r"[^a-z0-9]+", "_", block.name.lower()).strip("_")
    return SFunction(
        name=name,
        code=block.code(),
        input_widths=block.input_widths,
        output_widths=[3, *(len(v) for v in block.outputs.values())],
        params=[f"double({p})" for p in params],
        sample_time=[m.variable for m in block.mask_params].index("T_s"),
    )


def sfunction_params(
    block: MonolithicBlock,
    values: dict[str, Any],
    flux_map: dict[str, Any] | None = None,
) -> list[Any]:
    """
    Values of the S-function parameters (the C-Script parameters) of a block.

    The values are those of the motulator API (e.g., `sm.export_values`), and the
    GradNet flux map gives the parameters gn_* (empty without a flux map).

    """
    params = block.cscript_params or [m.variable for m in block.mask_params]
    out = []
    for p in params:
        if p == "isempty(psi_s_dq_fcn)":
            out.append(float(flux_map is None))
        elif p.startswith("gn_"):
            out.append(None if flux_map is None else flux_map[p[3:]])
        else:
            out.append(values[p])
    return out
