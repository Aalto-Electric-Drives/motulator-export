"""
2.2-kW IPMSM, FVC, parameter sweep
==================================

This example studies the sensitivity of sensorless flux-vector control (FVC) of a
2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive to an error in the
stator resistance estimate at low speeds. The drive of
:doc:`plot_2kw_ipmsm_fvc` is simulated natively for several estimates, each simulation
taking a fraction of a second.

"""

# %%
import time
from math import pi

import matplotlib.pyplot as plt
import motulator.drive.control.sm as control
from motulator.common.utils import set_style
from motulator.drive import model, utils

from motulator_export import native

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)


# %%
# Configure the system. The control system uses the stator resistance estimate
# `k_R*R_s`. The speed reference is 0.1 p.u., and half the rated load torque is
# applied at 0.6 s.


def build_system(k_R: float) -> tuple[model.Drive, control.VectorControlSystem]:
    """Build the system with the stator resistance estimate `k_R*R_s`."""
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
    )
    est_par = model.SynchronousMachinePars(
        n_p=3, R_s=k_R * 3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    cfg = control.FluxVectorControllerCfg(i_s_max=1.5 * base.i, sensorless=True)
    vector_ctrl = control.FluxVectorController(est_par, cfg)
    speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
    ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)
    ctrl.set_speed_ref(lambda t: (t > 0.1) * 0.1 * base.w_M)
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * 0.5 * nom.tau)
    return mdl, ctrl


# %%
# Simulate natively with the different estimates and plot the rotor speeds. With the
# overestimated resistance (`k_R = 1.2`), the drive loses its stability under the
# load.

set_style()
fig, ax = plt.subplots()
k_Rs = [0.8, 0.9, 1.0, 1.1, 1.2]
start = time.perf_counter()
for i, k_R in enumerate(k_Rs):
    sim = native.Simulation(*build_system(k_R), show_progress=False)
    res = sim.simulate(t_stop=1.2)
    color = plt.get_cmap("viridis")(i / (len(k_Rs) - 1))
    ax.plot(res.mdl.t, res.mdl.mechanics.w_M / base.w_M, color=color, label=k_R)
print(f"Run time of the {len(k_Rs)} simulations: {time.perf_counter() - start:.2f} s")
ax.set_xlim(0, 1.2)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Rotor speed (p.u.)")
ax.legend(title=r"$k_R$", loc="center left", bbox_to_anchor=(1, 0.5))
fig.tight_layout()
plt.show()
