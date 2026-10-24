"""
2.2-kW IPMSM, sensorless CVC: motulator vs. Simulink
====================================================

This script builds the drive system of `ipmsm_2kw_cvc.py` in motulator and writes a
MATLAB script (simulink/build_ipmsm_2kw_cvc.m) that builds the Simulink model. If
the MATLAB Engine API for Python is installed, it runs the script, simulates the
model and motulator, and compares the results. With --matlab, the blocks of the
control system are MATLAB Function blocks instead of S-functions
(simulink/build_ipmsm_2kw_cvc_matlab.m).

Run from the repository root:

    python examples/ipmsm_2kw_cvc_simulink.py

Without the MATLAB Engine API, run the generated build script in MATLAB.

"""

# %%
from _compare import (
    Comparison,
    drive_signals,
    require_matlab_engine,
    simulate_motulator,
    write_simulink_model,
)
from ipmsm_2kw_cvc import T_STOP, TAU_L, W_M_REF, build_system

from motulator_export import simulink

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    path = write_simulink_model("ipmsm_2kw_cvc", mdl, ctrl, T_STOP)
    require_matlab_engine()

    # Simulate in motulator and in Simulink, and compare
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    sl_signals = simulink.simulate(path, cmp.t_eval, mdl, ctrl)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    ctrl_signals = {
        "w_M": fbk.w_M,
        "tau_M": fbk.tau_M,
        "tau_M_ref": ref.tau_M,
        "i_q_ref": ref.i_s.imag,
    }
    cmp.print_differences("Simulink", sl_signals, drive_signals(res), ctrl_signals)
