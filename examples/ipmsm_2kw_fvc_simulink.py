"""
2.2-kW IPMSM, sensorless FVC: motulator vs. Simulink
====================================================

This script builds the drive system of `ipmsm_2kw_fvc.py` in motulator and writes a
MATLAB script (simulink/build_ipmsm_2kw_fvc.m) that builds the Simulink model. If
the MATLAB Engine API for Python is installed, it runs the script, simulates the
model and motulator, and compares the results. With --matlab, the blocks of the
control system are MATLAB Function blocks instead of S-functions
(simulink/build_ipmsm_2kw_fvc_matlab.m).

Run from the repository root:

    python examples/ipmsm_2kw_fvc_simulink.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%
import sys
from pathlib import Path
from typing import cast

import motulator.drive.control.sm as control
import numpy as np
from ipmsm_2kw_fvc import T_STOP, TAU_L, W_M_REF, build_system
from motulator.common.model import SolverCfg
from motulator.drive import model

from motulator_export import simulink

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    code = "matlab" if "--matlab" in sys.argv else "c"  # MATLAB Function blocks
    name = "ipmsm_2kw_fvc_matlab" if code == "matlab" else "ipmsm_2kw_fvc"
    path = Path(__file__).parent / "simulink" / f"{name}.slx"
    path.parent.mkdir(exist_ok=True)
    script = simulink.write_model(path, mdl, ctrl, T_STOP, code=code)
    print(f"Wrote {script}")
    try:
        import matlab.engine  # noqa: F401, PLC0415  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("MATLAB Engine API for Python not available, skipping the comparison.")
        sys.exit()

    # Simulate in motulator with tight tolerances
    T_s = cast(control.FluxVectorController, ctrl.vector_ctrl).cfg.T_s
    sim = model.Simulation(mdl, ctrl, cfg=SolverCfg(rtol=1e-9, atol=1e-9))
    res = sim.simulate(t_stop=T_STOP)
    in_mdl = res.mdl.t <= T_STOP
    in_ctrl = res.ctrl.t + 0.5 * T_s <= T_STOP

    # Build and simulate in Simulink, with outputs at the motulator solver steps and
    # just after the sampling instants
    t_mdl = res.mdl.t[in_mdl]
    t_ctrl = res.ctrl.t[in_ctrl] + 0.5 * T_s
    t_eval = np.unique(np.concatenate((t_mdl, t_ctrl)))
    sl_mdl, sl_ctrl = simulink.simulate(path, t_eval, mdl, ctrl)

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
        ("psi_s_ref", res.ctrl.ref.psi_s[in_ctrl]),
    ]:
        err = np.max(np.abs(sl_ctrl[name][k_ctrl] - value))
        print(f"  ctrl.{name}: {err:.3g}")
