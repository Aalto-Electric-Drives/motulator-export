"""Simulation of PLECS models via the XML-RPC interface of PLECS Standalone."""

import xmlrpc.client
from pathlib import Path
from typing import Any

import numpy as np


def simulate_plecs(
    path: str | Path,
    t_eval: np.ndarray,
    model_vars: dict[str, float] | None = None,
    url: str = "http://localhost:1080/RPC2",
    mdl_outputs: list[str] | None = None,
    ctrl_outputs: list[str] | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate the PLECS model via the XML-RPC interface of PLECS Standalone.

    Parameters
    ----------
    path : str | Path
        Path of the model file.
    t_eval : ndarray
        Output times (s).
    model_vars : dict[str, float], optional
        Workspace variables to be overridden.
    url : str, optional
        URL of the RPC interface, defaults to "http://localhost:1080/RPC2".
    mdl_outputs, ctrl_outputs : list[str]
        Names of the signals in the output ports "mdl" and "ctrl".

    Returns
    -------
    mdl : dict[str, ndarray]
        System-model signals and the time "t".
    ctrl : dict[str, ndarray]
        Controller signals and the time "t".

    """
    path = Path(path).resolve()
    server = xmlrpc.client.ServerProxy(url)
    # Close the model first, since loading a model that is already open (e.g., in
    # the PLECS window) does not reload it from the file
    try:
        server.plecs.close(path.stem)
    except xmlrpc.client.Fault:
        pass
    server.plecs.load(str(path))
    opts: dict[str, Any] = {"SolverOpts": {"OutputTimes": [float(t) for t in t_eval]}}
    if model_vars:
        opts["ModelVars"] = model_vars
    try:
        res: Any = server.plecs.simulate(path.stem, opts)
    finally:
        server.plecs.close(path.stem)
    t = np.array(res["Time"])
    values = np.array(res["Values"])
    mdl_names, ctrl_names = mdl_outputs or [], ctrl_outputs or []
    n_mdl = len(mdl_names)
    if values.shape[0] != n_mdl + len(ctrl_names):
        raise RuntimeError(f"Unexpected number of output signals: {values.shape}")
    mdl = {"t": t, **dict(zip(mdl_names, values[:n_mdl], strict=True))}
    ctrl = {"t": t, **dict(zip(ctrl_names, values[n_mdl:], strict=True))}
    return mdl, ctrl
