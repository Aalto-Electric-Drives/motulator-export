"""
10-kVA grid converter, LCL filter, GFL control: motulator vs. Simulink
======================================================================

This script builds the grid converter system of `gfl_10kva_lcl.py` in motulator and
writes a MATLAB script (simulink/build_gfl_10kva_lcl.m) that builds the Simulink
model. If the MATLAB Engine API for Python is installed, it runs the script,
simulates the model and motulator, and compares the results. With --matlab, the
blocks of the control system are MATLAB Function blocks instead of S-functions
(simulink/build_gfl_10kva_lcl_matlab.m).

Run from the repository root:

    python examples/gfl_10kva_lcl_simulink.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%

from _compare import (
    Comparison,
    require_matlab_engine,
    simulate_motulator,
    write_simulink_model,
)
from gfl_10kva_lcl import P_G_REF, Q_G_REF, T_STOP, build_system
from motulator.common.utils import complex2abc

from motulator_export import simulink

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_power_ref(P_G_REF)
    ctrl.set_reactive_power_ref(Q_G_REF)
    path = write_simulink_model("gfl_10kva_lcl", mdl, ctrl, T_STOP)
    require_matlab_engine()

    # Simulate in motulator and in Simulink, and compare, the phase current too, since
    # the space vectors would hide a zero-sequence current
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    sl_signals = simulink.simulate(path, cmp.t_eval, mdl, ctrl)
    f, fbk, ref = res.mdl.ac_filter, res.ctrl.fbk, res.ctrl.ref
    mdl_signals = {
        "i_c_ab": f.i_c_ab,
        "i_g_ab": f.i_g_ab,
        "i_c_a": complex2abc(f.i_c_ab)[0],
    }
    ctrl_signals = {
        "p_g": fbk.p_g,
        "q_g": fbk.q_g,
        "u_g": fbk.u_g,
        "w_g": fbk.w_g,
        "i_c_d_ref": ref.i_c.real,
    }
    cmp.print_differences("Simulink", sl_signals, mdl_signals, ctrl_signals)
