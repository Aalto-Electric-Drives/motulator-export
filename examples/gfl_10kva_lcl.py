"""
10-kVA converter, LCL filter, grid-following control: motulator vs. PLECS
========================================================================

This script builds the grid converter system of the example
examples/grid/grid_following/plot_10kva_lcl_gfl.py in motulator, exports it to a
PLECS model (gfl_10kva_lcl.plecs), and, if PLECS Standalone is running with the RPC
interface enabled, simulates both and compares the results.

Run from the repository root:

    python examples/gfl_10kva_lcl.py

"""

# %%
from pathlib import Path

from _compare import (
    Comparison,
    plot_grid,
    simulate_motulator,
    simulate_plecs,
    write_plecs_model,
)
from motulator.common.utils import complex2abc
from motulator.grid import control, model, utils

from motulator_export import StepSignal

T_STOP = 0.08
P_G_REF = StepSignal(time=0.02, after=5e3)  # Active power reference (W)
Q_G_REF = StepSignal(time=0.04, after=4e3)  # Reactive power reference (VAr)
nom = utils.NominalValues(U=400, I=14.5, f=50, P=10e3)
base = utils.BaseValues.from_nominal(nom)


# %%
def build_system() -> tuple[
    model.GridConverterSystem, control.GridConverterControlSystem
]:
    """Build the grid converter system of the motulator example."""
    ac_filter = model.LCLFilter(
        L_fc=0.073 * base.L, L_fg=0.073 * base.L, C_f=0.043 * base.C, u_f0_ab=base.u
    )
    ac_source = model.ThreePhaseSource(w_g=base.w, e_g=base.u)
    # Fine quantization (not modeled in PLECS)
    modulator = model.CarrierComparison(N=2**24)
    converter = model.VoltageSourceConverter(u_dc=650, modulator=modulator)
    mdl = model.GridConverterSystem(converter, ac_filter, ac_source)
    cfg = control.CurrentVectorControllerCfg(
        i_max=1.5 * base.i, L=0.073 * base.L, T_s=100e-6
    )
    ctrl = control.GridConverterControlSystem(control.CurrentVectorController(cfg))
    return mdl, ctrl


# %%
if __name__ == "__main__":
    mdl, ctrl = build_system()
    ctrl.set_power_ref(P_G_REF)
    ctrl.set_reactive_power_ref(Q_G_REF)
    path = Path(__file__).with_name("gfl_10kva_lcl.plecs")
    tmp = write_plecs_model(path, mdl, ctrl, T_STOP)

    # Simulate in motulator and in PLECS, and compare, the phase current too, since
    # the space vectors would hide a zero-sequence current
    res = simulate_motulator(mdl, ctrl, T_STOP)
    cmp = Comparison(res, T_STOP)
    plecs_signals = simulate_plecs(tmp, cmp.t_eval, mdl, ctrl)
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
    cmp.print_differences("PLECS", plecs_signals, mdl_signals, ctrl_signals)
    plot_grid(cmp, res, plecs_signals, "PLECS")
