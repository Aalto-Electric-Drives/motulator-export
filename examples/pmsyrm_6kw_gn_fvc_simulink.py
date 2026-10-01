"""
5.6-kW PM-SyRM, GradNet models, sensored FVC: motulator vs. Simulink
====================================================================

This script builds the drive system of `pmsyrm_6kw_gn_fvc.py` in motulator and
writes a MATLAB script (simulink/build_pmsyrm_6kw_gn_fvc.m) that builds the
Simulink model. If the MATLAB Engine API for Python is installed, it runs the
script, simulates the model and motulator, and compares the results.

Run from the repository root:

    python examples/pmsyrm_6kw_gn_fvc_simulink.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%
import sys
import time
from pathlib import Path
from typing import cast

import motulator.drive.control.sm as control
import numpy as np
from motulator.common.model import SolverCfg
from motulator.drive import model
from pmsyrm_6kw_gn_fvc import SPEED_CTRL, T_STOP, TAU_L, W_M_REF, build_system

from motulator_export import sampled_step
from motulator_export.simulink import sm

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    # Speed reference switching at the same sample as in motulator
    T_s = cast(control.FluxVectorController, ctrl.vector_ctrl).cfg.T_s
    w_M_ref = sampled_step(W_M_REF, T_s)
    path = Path(__file__).parent / "simulink" / "pmsyrm_6kw_gn_fvc.slx"
    path.parent.mkdir(exist_ok=True)
    script = sm.write_model(path, mdl, ctrl, w_M_ref, TAU_L, T_STOP, SPEED_CTRL)
    print(f"Wrote {script}")
    try:
        import matlab.engine  # noqa: F401, PLC0415  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("MATLAB Engine API for Python not available, skipping the comparison.")
        sys.exit()

    # Simulate in motulator
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    sim = model.Simulation(mdl, ctrl, cfg=SolverCfg(rtol=1e-8, atol=1e-8))
    start = time.time()
    res = sim.simulate(t_stop=T_STOP)
    print(f"motulator: {time.time() - start:.1f} s")

    # Build and simulate in Simulink at the motulator solver steps
    keep = res.mdl.t <= T_STOP
    t, idx = np.unique(res.mdl.t[keep], return_index=True)
    start = time.time()
    sl_mdl, _ = sm.simulate(path, t)
    k = np.searchsorted(sl_mdl["t"], t)
    print(f"Simulink, including the start of MATLAB: {time.time() - start:.1f} s")
    i_s_abc = np.array([sl_mdl[p][k] for p in ("i_a", "i_b", "i_c")])
    i_s_ab = (2 / 3) * (i_s_abc[0] - 0.5 * (i_s_abc[1] + i_s_abc[2])) + 1j * (
        i_s_abc[1] - i_s_abc[2]
    ) / np.sqrt(3)
    print("Maximum differences (Simulink - motulator):")
    for name, sl_value, value in [
        ("w_M", sl_mdl["w_M"][k], res.mdl.mechanics.w_M[keep][idx]),
        ("tau_M", sl_mdl["tau_M"][k], res.mdl.machine.tau_M[keep][idx]),
        ("i_s_ab", i_s_ab, res.mdl.machine.i_s_ab[keep][idx]),
    ]:
        print(f"  mdl.{name}: {np.max(np.abs(sl_value - value)):.3g}")
