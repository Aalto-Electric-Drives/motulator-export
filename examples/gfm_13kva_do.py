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
import atexit
import sys
from pathlib import Path
from typing import cast

import matplotlib.pyplot as plt
import numpy as np
from motulator.common.model import CarrierComparison, SolverCfg
from motulator.common.utils import abc2complex, complex2abc
from motulator.grid import control, model, utils

from motulator_plecs import StepSignal, grid, sampled_step

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
    T_s = cast(control.ObserverBasedGridFormingController, ctrl.inner_ctrl).T_s
    p_g_ref = sampled_step(P_G_REF, T_s)
    path = grid.write_model(
        Path(__file__).with_name("gfm_13kva_do.plecs"),
        mdl,
        ctrl,
        T_STOP,
        p_g_ref=p_g_ref,
        v_c_ref=V_C_REF,
    )
    print(f"Wrote {path}")
    # Temporary copy with the output ports for the comparison, removed at exit
    tmp = grid.write_model(
        path.with_stem(path.stem + "_tmp"),
        mdl,
        ctrl,
        T_STOP,
        p_g_ref=p_g_ref,
        v_c_ref=V_C_REF,
        outputs=True,
    )
    atexit.register(tmp.unlink, missing_ok=True)

    # Simulate in motulator with tight tolerances
    ctrl.set_ac_voltage_ref(V_C_REF)
    ctrl.set_power_ref(P_G_REF)
    sim = model.Simulation(mdl, ctrl, cfg=SolverCfg(rtol=1e-9, atol=1e-9))
    res = sim.simulate(t_stop=T_STOP)
    in_mdl = res.mdl.t <= T_STOP
    in_ctrl = res.ctrl.t + 0.5 * T_s <= T_STOP

    # Simulate in PLECS at the motulator solver steps and just after the sampling
    # instants
    t_mdl = res.mdl.t[in_mdl]
    t_ctrl = res.ctrl.t[in_ctrl] + 0.5 * T_s
    t_eval = np.unique(np.concatenate((t_mdl, t_ctrl)))
    try:
        plecs_mdl, plecs_ctrl = grid.simulate(tmp, t_eval, mdl, ctrl)
    except ConnectionRefusedError:
        print("PLECS RPC interface not available, skipping the comparison.")
        sys.exit()

    k_mdl = np.searchsorted(t_eval, t_mdl)
    i_c_ab = abc2complex(np.array([plecs_mdl[f"i_c_{p}"][k_mdl] for p in "abc"]))
    print("Maximum differences (PLECS - motulator):")
    # The phase current is compared too, since the space vector would hide a
    # zero-sequence current
    i_c_a = complex2abc(res.mdl.ac_filter.i_c_ab[in_mdl])[0]
    for name, plecs_value, value in [
        ("i_c_ab", i_c_ab, res.mdl.ac_filter.i_c_ab[in_mdl]),
        ("i_c_a", plecs_mdl["i_c_a"][k_mdl], i_c_a),
    ]:
        print(f"  mdl.{name}: {np.max(np.abs(plecs_value - value)):.3g}")
    k_ctrl = np.searchsorted(t_eval, t_ctrl)
    fbk = res.ctrl.fbk
    for name, value in [
        ("p_g", fbk.p_g[in_ctrl]),
        ("q_g", fbk.q_g[in_ctrl]),
        ("v_c", np.abs(fbk.v_c[in_ctrl])),
        ("theta_c", fbk.theta_c[in_ctrl]),
    ]:
        err = np.max(np.abs(plecs_ctrl[name][k_ctrl] - value))
        print(f"  ctrl.{name}: {err:.3g}")

    # Plot
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    t = t_mdl
    axs[0].plot(res.ctrl.t, fbk.p_g, label=r"$p_\mathrm{g}$ (motulator)")
    axs[0].plot(t_eval, plecs_ctrl["p_g"], "--", label=r"$p_\mathrm{g}$ (PLECS)")
    axs[0].plot(res.ctrl.t, fbk.q_g, label=r"$q_\mathrm{g}$ (motulator)")
    axs[0].plot(t_eval, plecs_ctrl["q_g"], "--", label=r"$q_\mathrm{g}$ (PLECS)")
    axs[0].set_ylabel("Power (W, VAr)")
    axs[1].plot(
        t, res.mdl.ac_filter.i_c_ab[in_mdl].real, label=r"$i_{c\alpha}$ (motulator)"
    )
    axs[1].plot(t, i_c_ab.real, "--", label=r"$i_{c\alpha}$ (PLECS)")
    axs[1].set_ylabel("Current (A)")
    axs[1].set_xlabel("Time (s)")
    for ax in axs:
        ax.legend(loc="upper right")
        ax.grid(True)
    fig.tight_layout()
    plt.show()
