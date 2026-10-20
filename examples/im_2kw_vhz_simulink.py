"""
2.2-kW IM, observer-based or open-loop V/Hz control: motulator vs. Simulink
===========================================================================

This script builds the drive system of `im_2kw_vhz.py` in motulator and writes the
MATLAB scripts (simulink/build_im_2kw_vhz.m and simulink/init_im_2kw_vhz.m) that
build and parametrize the Simulink model. If the MATLAB Engine API for Python is
installed, it runs the build script, simulates the model and motulator, and compares
the results. With --open-loop, the control system is pure open-loop V/Hz control
(simulink/build_im_2kw_vhz_open_loop.m), see `im_2kw_vhz.py`. With --matlab, the
blocks of the control system are MATLAB Function blocks instead of S-functions
(simulink/build_im_2kw_vhz_matlab.m).

Run from the repository root:

    python examples/im_2kw_vhz_simulink.py [--open-loop]

Without the MATLAB Engine API, run the generated build script in MATLAB.

"""

# %%
import sys

import numpy as np
from _compare import (
    Comparison,
    drive_signals,
    require_matlab_engine,
    simulate_motulator,
    write_simulink_model,
)
from im_2kw_vhz import T_STOP, TAU_L, W_M_REF, build_system

from motulator_export import simulink

# %%
if __name__ == "__main__":
    open_loop = "--open-loop" in sys.argv  # Pure open-loop V/Hz control
    mdl, ctrl = build_system(open_loop)
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    path = write_simulink_model(
        "im_2kw_vhz_open_loop" if open_loop else "im_2kw_vhz", mdl, ctrl, T_STOP
    )
    require_matlab_engine()

    # Simulate in motulator and in Simulink, and compare
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    sl_signals = simulink.simulate(path, cmp.t_eval, mdl, ctrl)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    ctrl_signals = {
        "w_s": fbk.w_s,
        "tau_M": fbk.tau_M,
        "tau_M_ref": ref.tau_M,
        "psi_s": np.abs(fbk.psi_s),
    }
    cmp.print_differences("Simulink", sl_signals, drive_signals(res), ctrl_signals)
