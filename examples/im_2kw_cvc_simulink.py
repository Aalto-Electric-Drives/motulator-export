"""
2.2-kW IM, sensorless CVC: motulator vs. Simulink
=================================================

This script builds the drive system of `im_2kw_cvc.py` in motulator and writes a
MATLAB script (simulink/build_im_2kw_cvc.m) that builds the Simulink model. If
the MATLAB Engine API for Python is installed, it runs the script, simulates the
model and motulator, and compares the results.

Run from the repository root:

    python examples/im_2kw_cvc_simulink.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%
import sys
from pathlib import Path
from typing import cast

import motulator.drive.control.im as control
import numpy as np
from im_2kw_cvc import SPEED_CTRL, T_STOP, TAU_L, W_M_REF, build_system
from motulator.common.model import SolverCfg
from motulator.drive import model

from motulator_export import sampled_step
from motulator_export.simulink import im

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    # Speed reference switching at the same sample as in motulator
    T_s = cast(control.CurrentVectorController, ctrl.vector_ctrl).cfg.T_s
    w_M_ref = sampled_step(W_M_REF, T_s)
    path = Path(__file__).parent / "simulink" / "im_2kw_cvc.slx"
    path.parent.mkdir(exist_ok=True)
    script = im.write_model(path, mdl, ctrl, w_M_ref, TAU_L, T_STOP, SPEED_CTRL)
    print(f"Wrote {script}")
    try:
        import matlab.engine  # noqa: F401, PLC0415  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("MATLAB Engine API for Python not available, skipping the comparison.")
        sys.exit()

    # Simulate in motulator with tight tolerances
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    sim = model.Simulation(mdl, ctrl, cfg=SolverCfg(rtol=1e-9, atol=1e-9))
    res = sim.simulate(t_stop=T_STOP)
    in_mdl = res.mdl.t <= T_STOP
    in_ctrl = res.ctrl.t + 0.5 * T_s <= T_STOP

    # Build and simulate in Simulink, with outputs at the motulator solver steps and
    # just after the sampling instants
    t_mdl = res.mdl.t[in_mdl]
    t_ctrl = res.ctrl.t[in_ctrl] + 0.5 * T_s
    t_eval = np.unique(np.concatenate((t_mdl, t_ctrl)))
    sl_mdl, sl_ctrl = im.simulate(path, t_eval)

    # Compare the machine signals at the motulator solver steps
    k_mdl = np.searchsorted(sl_mdl["t"], t_mdl)
    i_s_abc = np.array([sl_mdl[k][k_mdl] for k in ("i_a", "i_b", "i_c")])
    i_s_ab = (2 / 3) * (i_s_abc[0] - 0.5 * (i_s_abc[1] + i_s_abc[2])) + 1j * (
        i_s_abc[1] - i_s_abc[2]
    ) / np.sqrt(3)
    print("Maximum differences (Simulink - motulator):")
    for name, sl_value, value in [
        ("w_M", sl_mdl["w_M"][k_mdl], res.mdl.mechanics.w_M[in_mdl]),
        ("tau_M", sl_mdl["tau_M"][k_mdl], res.mdl.machine.tau_M[in_mdl]),
        ("i_s_ab", i_s_ab, res.mdl.machine.i_s_ab[in_mdl]),
    ]:
        print(f"  mdl.{name}: {np.max(np.abs(sl_value - value)):.3g}")

    # Compare the controller signals at the sampling instants
    k_ctrl = np.searchsorted(sl_ctrl["t"], t_ctrl)
    for name, value in [
        ("w_M", res.ctrl.fbk.w_M[in_ctrl]),
        ("tau_M", res.ctrl.fbk.tau_M[in_ctrl]),
        ("tau_M_ref", res.ctrl.ref.tau_M[in_ctrl]),
        ("psi_R", np.abs(res.ctrl.fbk.psi_R[in_ctrl])),
    ]:
        err = np.max(np.abs(sl_ctrl[name][k_ctrl] - value))
        print(f"  ctrl.{name}: {err:.3g}")
