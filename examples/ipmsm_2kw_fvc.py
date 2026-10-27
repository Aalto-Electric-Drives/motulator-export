"""
2.2-kW IPMSM, sensorless FVC: motulator vs. PLECS
=================================================

This script builds the drive system of the README example in motulator, exports it
to a PLECS model (ipmsm_2kw_fvc.plecs), and, if PLECS Standalone is running with
the RPC interface enabled, simulates both and compares the results.

Run from the repository root:

    python examples/ipmsm_2kw_fvc.py [--diode] [--blocked]

With --diode, the DC bus is a capacitor fed from a diode bridge, which is supplied
from the grid through AC chokes, as in
examples/drive/grid_connection/plot_2kw_ipmsm_diode_cvc.py
(ipmsm_2kw_fvc_diode.plecs). The diode bridge and the drive are a composite model.

With --blocked, the gates of the converter are blocked before the start and during
the operation, as in examples/drive/current_vector/plot_2kw_ipmsm_blocked_cvc.py
(ipmsm_2kw_fvc_blocked.plecs). While the gates are blocked, the control system runs
in standby: its observer runs on the open-circuit model of the machine, and its flux
and torque controller tracks the estimated flux and torque, from which the control
continues when the gates are released. Under the load, the stator
current decays through the diodes within a millisecond, after which the machine
decelerates at open circuit, since its voltage is below the DC-bus voltage.

"""

# %%
import sys
from math import pi, sqrt
from pathlib import Path
from typing import Any

import motulator.drive.control.sm as control
from _compare import (
    Comparison,
    drive_signals,
    plot_drive,
    simulate_motulator,
    simulate_plecs,
    write_plecs_model,
)
from motulator.common.model import (
    CarrierComparison,
    CompositeModel,
    DCBus,
    DiodeBridge,
    ThreePhaseSource,
)
from motulator.drive import model
from motulator.grid.model import GridConverterSystem, LFilter

from motulator_export import StepSignal

T_STOP = 1.2
W_M_REF = StepSignal(time=0.1, after=50.0)  # Speed reference (rad/s)
TAU_L = StepSignal(time=0.7, after=10.0)  # Load torque (Nm)
# Gate enable with --blocked: released at 0.05 s, blocked under the load at 0.8 s,
# and released at 0.83 s
ENABLE = StepSignal(time=[0.05, 0.8, 0.83], after=[1.0, 0.0, 1.0], before=0.0)


# %%
def build_system(
    diode: bool = False,
) -> tuple[model.Drive, control.VectorControlSystem]:
    """
    Build the drive system of the README example, with `diode` fed from a DC-bus
    capacitor (see `with_diode_bridge`).
    """
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=None if diode else 540),
        pwm=True,
        dc_bus=DCBus(C_dc=235e-6, u_dc=sqrt(2) * 400) if diode else None,
    )
    mdl.pwm = CarrierComparison(N=2**24)  # Fine quantization (not modeled in PLECS)
    cfg = control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=True)
    ctrl = control.VectorControlSystem(
        control.FluxVectorController(par, cfg),
        control.SpeedController(J=0.015, alpha_s=25.0),
    )
    return mdl, ctrl


def with_diode_bridge(mdl: model.Drive) -> CompositeModel:
    """
    The drive with its DC bus fed from a diode bridge, which is supplied from the
    grid through AC chokes, as a composite model.
    """
    grid = GridConverterSystem(
        DiodeBridge(),
        LFilter(L_f=0.5e-3),
        ThreePhaseSource(w_g=2 * pi * 50, e_g=sqrt(2 / 3) * 400),
        dc_bus=mdl.dc_bus,
    )
    return CompositeModel({"grid": grid, "drive": mdl})


# %%
if __name__ == "__main__":
    # Export the PLECS model (before simulating, since simulation changes the states)
    diode = "--diode" in sys.argv  # DC-bus capacitor fed from a diode bridge
    blocked = "--blocked" in sys.argv  # Gates blocked at the start and later
    mdl, ctrl = build_system(diode)
    ctrl.set_speed_ref(W_M_REF)
    mdl.mechanics.set_external_load_torque(TAU_L)
    if blocked:
        # The gate enable of the control system, exported as its input enable
        ctrl.set_enable(ENABLE)
    suffix = ("_diode" if diode else "") + ("_blocked" if blocked else "")
    path = Path(__file__).with_name(f"ipmsm_2kw_fvc{suffix}.plecs")
    system: tuple[Any, Any] = (mdl, ctrl)
    if diode:
        # A composite model, whose control systems are given by the model names
        system = (with_diode_bridge(mdl), {"drive": ctrl})
    tmp = write_plecs_model(path, *system, T_STOP)

    # Simulate in motulator and in PLECS, and compare
    res = simulate_motulator(*system, T_STOP)
    if diode:
        res = model.SimulationResults(res.mdl.drive, res.ctrl.drive)
    cmp = Comparison(res, T_STOP)
    plecs_signals = simulate_plecs(tmp, cmp.t_eval, *system)
    fbk, ref = res.ctrl.fbk, res.ctrl.ref
    ctrl_signals = {
        "w_M": fbk.w_M,
        "tau_M": fbk.tau_M,
        "tau_M_ref": ref.tau_M,
        "psi_s_ref": ref.psi_s,
    }
    cmp.print_differences("PLECS", plecs_signals, drive_signals(res), ctrl_signals)
    plot_drive(cmp, res, plecs_signals[0], "PLECS")
