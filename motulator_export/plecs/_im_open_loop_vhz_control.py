"""
Open-loop V/Hz control of induction machine drives: each class is a block.

The control system mirrors the classes of motulator as a hierarchy of blocks, each
with a mask of the arguments of its class (see `_control`):

- VHzControlSystem (the masked subsystem of the control system), containing the
  blocks of its methods (`get_measurement` and `modulate`), the monitored signals,
  the `RateLimiter` of the speed reference, the `PWM` (with the MME overmodulation by
  default), and the `OpenLoopVHzController`
- OpenLoopVHzController(n_p, u_s_ref, T_s), whose affine V/Hz curve u_s_ref is
  given by its coefficients (see `_im.vhz_curve`)

The mask of the VHzControlSystem has its arguments from the model initialization
(`init_script`): the V/Hz controller `vhz_ctrl` (the struct of its arguments), the
slew rate `slew_rate` of the speed reference, and the PWM `pwm`. The signal vectors
are defined in `im_open_loop_vhz.h`.

"""

from typing import Any

from motulator_export.plecs._common import ENABLE
from motulator_export.plecs._control import (
    SAMPLING,
    Signals,
    abc_input,
    assign,
    checks,
    complex_in,
    complex_out,
    control_block,
    get_measurement,
    init_header,
    mask_params,
    monitor,
    pwm_init,
    vhz_control_system,
)
from motulator_export.plecs._netlist import (
    COMPLEX,
    CBlock,
    Mask,
    MCode,
    Param,
    Port,
    Subsystem,
    required,
)

SIGNALS = Signals(
    ("common.c", "im_open_loop_vhz.c"),
    ("im_open_loop_vhz.h",),
    {
        "meas": (
            "IMOpenLoopVHzMeasurements",
            "im_open_loop_vhz_measurements",
            "OLVHZ_MEAS",
        ),
        "fbk": ("IMOpenLoopVHzFeedbacks", "im_open_loop_vhz_feedbacks", "OLVHZ_FBK"),
        "ref": ("IMOpenLoopVHzReferences", "im_open_loop_vhz_references", "OLVHZ_REF"),
    },
    "sfun_olvhz",
)

# Name of the control-system block, its inputs, and the monitored signals (mask
# probes)
NAME = "Open-loop VHz control"  # No slash, the path separator of Simulink
CTRL_INPUTS = [Port(ENABLE, 1), Port("w_M_ref", 1), Port("i_s_abc", 3), Port("u_dc", 1)]
CTRL_OUTPUTS = {
    "Speed (w_M_ref, w_s)": ["w_M_ref", "w_s"],
    "Voltage (u_s_ref, u_s)": ["u_s_ref", "u_s"],
    "Current (i_sd, i_sq)": ["i_sd", "i_sq"],
}

# The V/Hz curve u_s_ref as the parameters of the C-Script: the mask initialization
# takes its coefficients, each [real part, imaginary part]
CURVE = {"u_s_ref": ["u_s_ref_u_0", "u_s_ref_k"]}
CURVE_INIT = (
    "% Coefficients of the V/Hz curve u_s_ref for the C-Script\n"
    "u_s_ref_u_0 = u_s_ref.u_0;\n"
    "u_s_ref_k = u_s_ref.k;\n"
)


# %%
# Block of the class
def _vhz_controller() -> CBlock:
    """Open-loop V/Hz controller (OpenLoopVHzController)."""
    mask = Mask(
        "OpenLoopVHzController (motulator)",
        "Open-loop V/Hz controller (OpenLoopVHzController): the stator voltage "
        "reference is given by the V/Hz curve u_s_ref as a function of the stator "
        "angular frequency w_s = n_p*w_M, where w_M is the rate-limited speed "
        "reference, in coordinates rotating at w_s. The curve is affine, u_s_ref = "
        "u_0 + k*w_s, given by the voltage u_0 at zero frequency and the slope k. The "
        "output fbk.u_s is the converter voltage u_c_ab averaged over the previous and "
        "ongoing periods (DriveControlSystem.get_feedback).",
        [
            Param("n_p", "n_p: Number of pole pairs", "vhz_ctrl.n_p"),
            Param(
                "u_s_ref",
                "u_s_ref: V/Hz curve u_0 + k*w_s as a struct of the coefficients u_0 "
                "(V) and k (Vs), each [real part, imaginary part]",
                "vhz_ctrl.u_s_ref",
            ),
            Param("T_s", "T_s: Sampling period (s)", "vhz_ctrl.T_s"),
        ],
        init=CURVE_INIT,
    )
    params = mask_params(mask, CURVE)
    i = {p: k for k, p in enumerate(params)}
    u_0, k = i["u_s_ref_u_0"], i["u_s_ref_k"]
    start = (
        checks(params, ["n_p", "T_s"]) + f"if (PDIM({u_0}) != 2 || PDIM({k}) != 2) {{\n"
        '    SetErrorMessage("u_s_ref.u_0 and u_s_ref.k must be [real part, '
        'imaginary part].");\n'
        "    return;\n"
        "}\n"
        f"T_s = P({i['T_s']}, 0);\n"
        f"im_open_loop_vhz_ctrl_init(&self, P({i['n_p']}, 0),\n"
        f"                           complex_from(P({u_0}, 0), P({u_0}, 1)),\n"
        f"                           complex_from(P({k}, 0), P({k}, 1)), T_s);\n"
    )
    output = (
        SIGNALS.read(1, "meas")
        + "/* Feedback signals, the voltage of the ongoing period NAN if unknown */\n"
        f"im_open_loop_vhz_ctrl_get_feedback(&self, {complex_in(3)},\n"
        "                                   meas.i_c_ab, InputSignal(0, 0), &fbk);\n"
        "/* Voltage for the control laws (DriveControlSystem.get_feedback), or\n"
        " * u_s_zoh if unknown */\n"
        f"double complex u_c_ab = {complex_in(2)};\n"
        "fbk.u_s = isnan(creal(u_c_ab)) ? fbk.u_s_zoh\n"
        "                               : cexp(-I * fbk.theta_c) * u_c_ab;\n"
        + SIGNALS.write(0, "fbk", "fbk")
        + "/* Stator voltage reference (ref.u_s) */\n"
        "double complex u_s_ref = im_open_loop_vhz_ctrl_compute_output(&self, &fbk);\n"
        + complex_out(1, "u_s_ref")
    )
    return CBlock(
        "OpenLoopVHzController",
        [
            Port("w_M", 1),
            SIGNALS.port("meas"),
            Port("u_c_ab", 2),
            Port("u_c_zoh_ab", 2),
        ],
        [SIGNALS.port("fbk"), SIGNALS.port("ref")],
        SIGNALS.class_code(
            "IMOpenLoopVHzController",
            start,
            output,
            "im_open_loop_vhz_ctrl_update(&self, T_s, &fbk);\n",
            workspace=SAMPLING + "static IMOpenLoopVHzFeedbacks fbk;\n",
        ),
        params,
        SIGNALS.sfunction("vhz_controller"),
        mask=mask,
        mcode=MCode(
            start="self = im_open_loop_vhz_ctrl_init(p.n_p, ...\n"
            "    complex(p.u_s_ref_u_0(1), p.u_s_ref_u_0(2)), ...\n"
            "    complex(p.u_s_ref_k(1), p.u_s_ref_k(2)), p.T_s);\n",
            output="fbk = im_open_loop_vhz_ctrl_get_feedback(self, u_c_zoh_ab, "
            "meas.i_c_ab, w_M);\n"
            "% Voltage for the control laws (DriveControlSystem.get_feedback), or\n"
            "% u_s_zoh if unknown\n"
            "if isnan(real(u_c_ab))\n"
            "    fbk.u_s = fbk.u_s_zoh;\n"
            "else\n"
            "    fbk.u_s = exp(-1i*fbk.theta_c)*u_c_ab;\n"
            "end\n"
            "ref = struct('u_s', im_open_loop_vhz_ctrl_compute_output(self, fbk));\n",
            update="self = im_open_loop_vhz_ctrl_update(self, p.T_s, fbk);\n",
            inputs={
                "meas": SIGNALS.vector("meas"),
                "u_c_ab": COMPLEX,
                "u_c_zoh_ab": COMPLEX,
            },
            outputs={"fbk": SIGNALS.vector("fbk"), "ref": SIGNALS.vector("ref")},
            checks=(
                *required("n_p", "T_s"),
                (
                    "numel(p.u_s_ref_u_0) ~= 2 || numel(p.u_s_ref_k) ~= 2",
                    "u_s_ref.u_0 and u_s_ref.k must be [real part, imaginary part].",
                ),
            ),
        ),
    )


# %%
MONITORED = {
    "w_M_ref": "InputSignal(2, 0)",
    "w_s": "fbk.w_s",
    "u_s_ref": "cabs(ref.u_s)",
    "u_s": "cabs(fbk.u_s)",
    "i_sd": "creal(fbk.i_s)",
    "i_sq": "cimag(fbk.i_s)",
}


def control_system() -> Subsystem:
    """Contents of the control-system block (VHzControlSystem)."""
    measurement = get_measurement(
        SIGNALS,
        CTRL_INPUTS[2:],
        [
            ("i_c_ab", "abc2complex(i_s_abc)"),
            ("u_dc", "fmax(InputSignal(1, 0), U_DC_MIN)"),
        ],
        abc_input("i_s_abc", 0),
    )
    signals = monitor(SIGNALS, CTRL_OUTPUTS, MONITORED, [Port("w_M_ref", 1)])
    return vhz_control_system(
        SIGNALS, NAME, CTRL_INPUTS, measurement, _vhz_controller(), signals
    )


VHZ_BLOCK = control_block(
    NAME,
    "Open-loop V/Hz control of an induction machine drive (VHzControlSystem with "
    "OpenLoopVHzController). The speed reference is in mechanical rad/s, and the "
    "monitored current is in the coordinates of the controller.",
    CTRL_INPUTS,
    CTRL_OUTPUTS,
    control_system,
    "OpenLoopVHzController",
    cls="VHzControlSystem",
)


# %%
def init_script(
    variables: list[tuple[str, Any]], values: dict[str, Any], comment: str
) -> str:
    """
    Model initialization: the variables of the system model, and the structs of the
    control system (vhz_ctrl, slew_rate, and pwm) from the parameter values of the
    control system (`_im.export_vhz_values`).
    """
    return (
        init_header(variables, comment)
        + "% Open-loop V/Hz control (OpenLoopVHzController): the V/Hz curve\n"
        "% u_s_ref = u_0 + k*w_s by the voltage u_0 at zero frequency (V) and the\n"
        "% slope k (Vs), each [real part, imaginary part]\n"
        + assign("vhz_ctrl.n_p", values["n_p"])
        + assign("vhz_ctrl.u_s_ref.u_0", values["u_0"])
        + assign("vhz_ctrl.u_s_ref.k", values["k"])
        + assign("vhz_ctrl.T_s", values["T_s"])
        + "% Slew rate of the speed reference (VHzControlSystem), mechanical rad/s^2\n"
        + assign("slew_rate", values["slew_rate"])
        + pwm_init(values)
    )
