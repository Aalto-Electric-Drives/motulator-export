"""
12.5-kVA grid converter, weak grid, DO-GFM control: motulator vs. Simulink
==========================================================================

This script builds the grid converter system of `gfm_13kva_do.py` in motulator and
writes a MATLAB script (simulink/build_gfm_13kva_do.m) that builds the Simulink
model. If the MATLAB Engine API for Python is installed, it runs the script,
simulates the model and motulator, and compares the results. With --matlab, the
blocks of the control system are MATLAB Function blocks instead of S-functions
(simulink/build_gfm_13kva_do_matlab.m).

Run from the repository root:

    python examples/gfm_13kva_do_simulink.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%

import numpy as np
from _compare import (
    Comparison,
    require_matlab_engine,
    simulate_motulator,
    write_simulink_model,
)
from gfm_13kva_do import P_G_REF, T_STOP, V_C_REF, build_system
from motulator.common.utils import complex2abc

from motulator_export import simulink

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_power_ref(P_G_REF)
    ctrl.set_ac_voltage_ref(V_C_REF)
    path = write_simulink_model("gfm_13kva_do", mdl, ctrl, T_STOP)
    require_matlab_engine()

    # Simulate in motulator and in Simulink, and compare, the phase current too, since
    # the space vectors would hide a zero-sequence current
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    sl_signals = simulink.simulate(path, cmp.t_eval, mdl, ctrl)
    f, fbk, ref = res.mdl.ac_filter, res.ctrl.fbk, res.ctrl.ref
    mdl_signals = {"i_c_ab": f.i_c_ab, "i_c_a": complex2abc(f.i_c_ab)[0]}
    ctrl_signals = {
        "p_g": fbk.p_g,
        "q_g": fbk.q_g,
        "v_c": np.abs(fbk.v_c),
        "theta_c": fbk.theta_c,
    }
    cmp.print_differences("Simulink", sl_signals, mdl_signals, ctrl_signals)
