"""Order of the examples in the gallery of the documentation."""

from pathlib import Path

EXAMPLES = [
    "plot_2kw_ipmsm_fvc.py",
    "plot_2kw_ipmsm_fvc_sweep.py",
    "plot_6kw_pmsyrm_gn_fvc.py",
    "plot_2kw_ipmsm_ovhz.py",
    "plot_2kw_im_diode_vhz.py",
    "plot_10kva_lcl_gfl.py",
    "plot_13kva_do_gfm_sag.py",
]


def example_order(filename: str) -> int:
    """Sort key of the examples, those not listed last."""
    name = Path(filename).name
    return EXAMPLES.index(name) if name in EXAMPLES else len(EXAMPLES)
