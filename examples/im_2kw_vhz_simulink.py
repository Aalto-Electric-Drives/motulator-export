"""
2.2-kW IM, observer-based or open-loop V/Hz control: motulator vs. Simulink
===========================================================================

This script builds the drive system of `im_2kw_vhz.py` in motulator and writes the
MATLAB scripts (simulink/build_im_2kw_vhz.m and simulink/init_im_2kw_vhz.m) that
build and parametrize the Simulink model. If the MATLAB Engine API for Python is
installed, it runs the build script, simulates the model and motulator, and compares
the results. With --open-loop, the control system is pure open-loop V/Hz control
(simulink/build_im_2kw_vhz_open_loop.m), see `im_2kw_vhz.py`.

Run from the repository root:

    python examples/im_2kw_vhz_simulink.py [--open-loop]

Without the MATLAB Engine API, run the generated build script in MATLAB.

"""

# %%
import sys
from pathlib import Path
from typing import cast

import motulator.drive.control.im as control
import numpy as np
from im_2kw_vhz import T_STOP, TAU_L, W_M_REF, build_system
from motulator.common.model import SolverCfg
from motulator.drive import model

from motulator_export import sampled_step
from motulator_export.simulink import im

# %%
if __name__ == "__main__":
    open_loop = "--open-loop" in sys.argv  # Pure open-loop V/Hz control
    mdl, ctrl = build_system(open_loop)
    # Speed reference switching at the same sample as in motulator
    T_s = cast(control.ObserverBasedVHzController, ctrl.vhz_ctrl).cfg.T_s
    w_M_ref = sampled_step(W_M_REF, T_s)
    name = "im_2kw_vhz_open_loop" if open_loop else "im_2kw_vhz"
    path = Path(__file__).parent / "simulink" / f"{name}.slx"
    path.parent.mkdir(exist_ok=True)
    script = im.write_model(path, mdl, ctrl, w_M_ref, TAU_L, T_STOP)
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
    sl_mdl, sl_ctrl = im.simulate(path, t_eval, ctrl=ctrl)

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
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    for name, value in [
        ("w_s", fbk.w_s[in_ctrl]),
        ("tau_M", fbk.tau_M[in_ctrl]),
        ("tau_M_ref", ref.tau_M[in_ctrl]),
        ("psi_s", np.abs(fbk.psi_s[in_ctrl])),
    ]:
        err = np.max(np.abs(sl_ctrl[name][k_ctrl] - value))
        print(f"  ctrl.{name}: {err:.3g}")
