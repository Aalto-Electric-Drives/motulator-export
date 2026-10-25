"""
2.2-kW IM, discrete-time current control: motulator vs. Simulink
================================================================

This script builds the drive system of `im_2kw_cvc_discrete.py` in motulator and
writes a MATLAB script (simulink/build_im_2kw_cvc_discrete.m) that builds the
Simulink model. If the MATLAB Engine API for Python is installed, it runs the
script, simulates the model and motulator, and compares the results. With
--matlab, the blocks of the control system are MATLAB Function blocks instead of
S-functions (simulink/build_im_2kw_cvc_discrete_matlab.m).

Run from the repository root:

    python examples/im_2kw_cvc_discrete_simulink.py

Without the MATLAB Engine API, run the generated build script in MATLAB.

"""

# %%
import numpy as np
from _compare import (
    Comparison,
    drive_signals,
    require_matlab_engine,
    simulate_motulator,
    write_simulink_model,
)
from im_2kw_cvc_discrete import T_STOP, TAU_M_REF, W_M_EXT, build_system

from motulator_export import simulink

# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_torque_ref(TAU_M_REF)
    mdl.mechanics.set_external_rotor_speed(W_M_EXT)
    path = write_simulink_model("im_2kw_cvc_discrete", mdl, ctrl, T_STOP)
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
        "psi_R": np.abs(fbk.psi_R),
    }
    cmp.print_differences("Simulink", sl_signals, drive_signals(res), ctrl_signals)
