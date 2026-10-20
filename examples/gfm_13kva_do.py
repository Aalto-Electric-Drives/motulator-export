"""
12.5-kVA converter, weak grid, DO-GFM: motulator vs. PLECS
==========================================================

This script builds the grid converter system of the example
examples/grid/grid_forming/plot_13kva_do_gfm.py in motulator, exports it to a
PLECS model (gfm_13kva_do.plecs), and, if PLECS
Standalone is running with the RPC interface enabled, simulates both and compares
the results.

Run from the repository root:

    python examples/gfm_13kva_do.py

"""

# %%
from pathlib import Path

import numpy as np
from _compare import (
    Comparison,
    plot_grid,
    simulate_motulator,
    simulate_plecs,
    write_plecs_model,
)
from motulator.common.model import CarrierComparison
from motulator.common.utils import complex2abc
from motulator.grid import control, model, utils

from motulator_export import StepSignal

T_STOP = 1.4
nom = utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
base = utils.BaseValues.from_nominal(nom)
# Active power reference (W): steps up to the nominal power, then to the rectifier mode
P_G_REF = StepSignal(
    time=[0.2, 0.5, 0.8, 1.2], after=[nom.P / 3, 2 * nom.P / 3, nom.P, -nom.P]
)
V_C_REF = base.u  # Converter voltage magnitude reference (V)


# %%
def build_system() -> tuple[
    model.GridConverterSystem, control.GridConverterControlSystem
]:
    """Build the grid converter system of the motulator example."""
    ac_filter = model.LFilter(L_f=0.15 * base.L, R_f=0.05 * base.Z, L_g=0.74 * base.L)
    ac_source = model.ThreePhaseSource(w_g=base.w, e_g=base.u)
    converter = model.VoltageSourceConverter(u_dc=650)
    mdl = model.GridConverterSystem(converter, ac_filter, ac_source, pwm=True)
    mdl.pwm = CarrierComparison(N=2**24)  # Fine quantization (not modeled in PLECS)
    cfg = control.ObserverBasedGridFormingControllerCfg(
        i_max=1.3 * base.i,
        L=0.35 * base.L,
        R=0.05 * base.Z,
        R_a=0.2 * base.Z,
        u_nom=base.u,
        w_nom=base.w,
    )
    ctrl = control.GridConverterControlSystem(
        control.ObserverBasedGridFormingController(cfg)
    )
    return mdl, ctrl


# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_ac_voltage_ref(V_C_REF)
    ctrl.set_power_ref(P_G_REF)
    path = Path(__file__).with_name("gfm_13kva_do.plecs")
    tmp = write_plecs_model(path, mdl, ctrl, T_STOP)

    # Simulate in motulator and in PLECS, and compare, the phase current too, since
    # the space vectors would hide a zero-sequence current
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    plecs_signals = simulate_plecs(tmp, cmp.t_eval, mdl, ctrl)
    f, fbk, ref = res.mdl.ac_filter, res.ctrl.fbk, res.ctrl.ref
    mdl_signals = {"i_c_ab": f.i_c_ab, "i_c_a": complex2abc(f.i_c_ab)[0]}
    ctrl_signals = {
        "p_g": fbk.p_g,
        "q_g": fbk.q_g,
        "v_c": np.abs(fbk.v_c),
        "theta_c": fbk.theta_c,
    }
    cmp.print_differences("PLECS", plecs_signals, mdl_signals, ctrl_signals)
    plot_grid(cmp, res, plecs_signals, "PLECS")
