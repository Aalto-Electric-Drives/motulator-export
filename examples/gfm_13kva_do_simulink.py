"""
12.5-kVA grid converter, weak grid, DO-GFM control: motulator vs. Simulink
==========================================================================

This script builds the grid converter system of `gfm_13kva_do.py` in motulator and
writes a MATLAB script (simulink/build_gfm_13kva_do.m) that builds the Simulink
model. If the MATLAB Engine API for Python is installed, it runs the script,
simulates the model and motulator, and compares the results.

Run from the repository root:

    python examples/gfm_13kva_do_simulink.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%
import sys
from pathlib import Path
from typing import cast

import numpy as np
from gfm_13kva_do import P_G_REF, T_STOP, V_C_REF, build_system
from motulator.common.model import SolverCfg
from motulator.common.utils import abc2complex, complex2abc
from motulator.grid import control, model

from motulator_export import sampled_step
from motulator_export.simulink import grid

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    T_s = cast(control.ObserverBasedGridFormingController, ctrl.inner_ctrl).T_s
    path = Path(__file__).parent / "simulink" / "gfm_13kva_do.slx"
    path.parent.mkdir(exist_ok=True)
    script = grid.write_model(
        path, mdl, ctrl, T_STOP, p_g_ref=sampled_step(P_G_REF, T_s), v_c_ref=V_C_REF
    )
    print(f"Wrote {script}")
    try:
        import matlab.engine  # noqa: F401, PLC0415  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("MATLAB Engine API for Python not available, skipping the comparison.")
        sys.exit()

    # Simulate in motulator with tight tolerances
    ctrl.set_power_ref(P_G_REF)
    ctrl.set_ac_voltage_ref(V_C_REF)
    sim = model.Simulation(mdl, ctrl, cfg=SolverCfg(rtol=1e-9, atol=1e-9))
    res = sim.simulate(t_stop=T_STOP)
    in_mdl = res.mdl.t <= T_STOP
    in_ctrl = res.ctrl.t + 0.5 * T_s <= T_STOP

    # Build and simulate in Simulink, with outputs at the motulator solver steps and
    # just after the sampling instants
    t_mdl = res.mdl.t[in_mdl]
    t_ctrl = res.ctrl.t[in_ctrl] + 0.5 * T_s
    t_eval = np.unique(np.concatenate((t_mdl, t_ctrl)))
    sl_mdl, sl_ctrl = grid.simulate(path, t_eval, mdl, ctrl)

    k_mdl = np.searchsorted(sl_mdl["t"], t_mdl)
    i_c_ab = abc2complex(np.array([sl_mdl[f"i_c_{p}"][k_mdl] for p in "abc"]))
    print("Maximum differences (Simulink - motulator):")
    i_c_a = complex2abc(res.mdl.ac_filter.i_c_ab[in_mdl])[0]
    for name, sl_value, value in [
        ("i_c_ab", i_c_ab, res.mdl.ac_filter.i_c_ab[in_mdl]),
        ("i_c_a", sl_mdl["i_c_a"][k_mdl], i_c_a),
    ]:
        print(f"  mdl.{name}: {np.max(np.abs(sl_value - value)):.3g}")
    k_ctrl = np.searchsorted(sl_ctrl["t"], t_ctrl)
    fbk = res.ctrl.fbk
    for name, value in [
        ("p_g", fbk.p_g[in_ctrl]),
        ("q_g", fbk.q_g[in_ctrl]),
        ("v_c", np.abs(fbk.v_c[in_ctrl])),
        ("theta_c", fbk.theta_c[in_ctrl]),
    ]:
        err = np.max(np.abs(sl_ctrl[name][k_ctrl] - value))
        print(f"  ctrl.{name}: {err:.3g}")
