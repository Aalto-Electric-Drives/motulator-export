"""
5.6-kW PM-SyRM, GradNet models, sensored FVC: motulator vs. Simulink
====================================================================

This script builds the drive system of `pmsyrm_6kw_gn_fvc.py` in motulator and
writes a MATLAB script (simulink/build_pmsyrm_6kw_gn_fvc.m) that builds the Simulink
model. If the MATLAB Engine API for Python is installed, it runs the script,
simulates the model and motulator, and compares the results. With --matlab, the
blocks of the control system are MATLAB Function blocks instead of S-functions
(simulink/build_pmsyrm_6kw_gn_fvc_matlab.m).

Run from the repository root:

    python examples/pmsyrm_6kw_gn_fvc_simulink.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%
import time

from _compare import (
    Comparison,
    drive_signals,
    require_matlab_engine,
    simulate_motulator,
    write_simulink_model,
)
from pmsyrm_6kw_gn_fvc import T_STOP, TAU_L, W_M_REF, build_system

from motulator_export import simulink

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    path = write_simulink_model("pmsyrm_6kw_gn_fvc", mdl, ctrl, T_STOP)
    require_matlab_engine()

    # Simulate in motulator
    start = time.time()
    res = simulate_motulator(mdl, ctrl, T_STOP)
    print(f"motulator: {time.time() - start:.1f} s")

    # Build and simulate in Simulink, and compare at the motulator solver steps
    cmp = Comparison(res, T_STOP, control=False)
    start = time.time()
    sl_signals = simulink.simulate(path, cmp.t_eval, mdl, ctrl)
    print(f"Simulink, including the start of MATLAB: {time.time() - start:.1f} s")
    cmp.print_differences("Simulink", sl_signals, drive_signals(res))
