"""
10-kVA grid converter, LCL filter, GFL control: motulator vs. Simulink
======================================================================

This script builds the grid converter system of `gfl_10kva_lcl.py` in motulator and
writes a MATLAB script (simulink/build_gfl_10kva_lcl.m) that builds the Simulink
model. If the MATLAB Engine API for Python is installed, it runs the script,
simulates the model and motulator, and compares the results.

Run from the repository root:

    python examples/gfl_10kva_lcl_simulink.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%
import sys
from pathlib import Path

import numpy as np
from gfl_10kva_lcl import P_G_REF, Q_G_REF, T_STOP, build_system
from motulator.common.model import SolverCfg
from motulator.common.utils import abc2complex, complex2abc
from motulator.grid import model

from motulator_export import sampled_step
from motulator_export.simulink import grid

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    T_s = 100e-6
    path = Path(__file__).parent / "simulink" / "gfl_10kva_lcl.slx"
    path.parent.mkdir(exist_ok=True)
    script = grid.write_model(
        path,
        mdl,
        ctrl,
        T_STOP,
        p_g_ref=sampled_step(P_G_REF, T_s),
        q_g_ref=sampled_step(Q_G_REF, T_s),
    )
    print(f"Wrote {script}")
    try:
        import matlab.engine  # noqa: F401, PLC0415  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("MATLAB Engine API for Python not available, skipping the comparison.")
        sys.exit()

    # Simulate in motulator with tight tolerances
    ctrl.set_power_ref(P_G_REF)
    ctrl.set_reactive_power_ref(Q_G_REF)
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
    i_g_ab = abc2complex(np.array([sl_mdl[f"i_g_{p}"][k_mdl] for p in "abc"]))
    print("Maximum differences (Simulink - motulator):")
    i_c_a = complex2abc(res.mdl.ac_filter.i_c_ab[in_mdl])[0]
    for name, sl_value, value in [
        ("i_c_ab", i_c_ab, res.mdl.ac_filter.i_c_ab[in_mdl]),
        ("i_g_ab", i_g_ab, res.mdl.ac_filter.i_g_ab[in_mdl]),
        ("i_c_a", sl_mdl["i_c_a"][k_mdl], i_c_a),
    ]:
        print(f"  mdl.{name}: {np.max(np.abs(sl_value - value)):.3g}")
    k_ctrl = np.searchsorted(sl_ctrl["t"], t_ctrl)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    for name, value in [
        ("p_g", fbk.p_g[in_ctrl]),
        ("q_g", fbk.q_g[in_ctrl]),
        ("u_g", fbk.u_g[in_ctrl]),
        ("w_g", fbk.w_g[in_ctrl]),
        ("i_c_d_ref", ref.i_c.real[in_ctrl]),
    ]:
        err = np.max(np.abs(sl_ctrl[name][k_ctrl] - value))
        print(f"  ctrl.{name}: {err:.3g}")
