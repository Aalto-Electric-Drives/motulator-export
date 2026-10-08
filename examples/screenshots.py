"""
Screenshots of the models for the README
========================================

This script prints the Simulink model of `pmsyrm_6kw_gn_fvc_simulink.py`
(simulink/pmsyrm_6kw_gn_fvc.slx, built by the script) to
pmsyrm_6kw_gn_fvc_simulink.png with the MATLAB Engine API for Python, and writes the
dark versions (`*_black.png`) of the screenshots for the dark mode of GitHub.

The dark version of an image inverts its lightness and keeps its hues: white becomes
black, black becomes white, and, e.g., the green signals of PLECS stay green. The
PLECS screenshot (pmsyrm_6kw_gn_fvc.png, exported from PLECS with File > Export >
PNG at 300 dpi) is not made here, but its dark version is.

Run from the repository root:

    python examples/screenshots.py [--dark-only]

With --dark-only, only the dark versions are written (MATLAB is not needed).

"""

# %%
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

HERE = Path(__file__).parent
MODEL = HERE / "simulink" / "pmsyrm_6kw_gn_fvc.slx"
SCREENSHOTS = [HERE / "pmsyrm_6kw_gn_fvc.png", HERE / "pmsyrm_6kw_gn_fvc_simulink.png"]
DPI = 300


def print_simulink(model: Path, png: Path) -> None:
    """Print the root level of a Simulink model to a PNG file."""
    import matlab.engine  # noqa: PLC0415  # pyright: ignore[reportMissingImports]

    eng: Any = matlab.engine.start_matlab()
    try:
        eng.addpath(str(model.parent), nargout=0)
        eng.load_system(str(model), nargout=0)  # Runs the initialization script
        # Subsystems as plain blocks, without the previews of their contents
        eng.eval(
            "cellfun(@(b) set_param(b, 'ContentPreviewEnabled', 'off'), "
            f"find_system('{model.stem}', 'SearchDepth', 1, 'BlockType', "
            "'SubSystem'));",
            nargout=0,
        )
        eng.print(f"-s{model.stem}", "-dpng", f"-r{DPI}", str(png), nargout=0)
        eng.close_system(model.stem, 0, nargout=0)
    finally:
        eng.quit()


def dark_version(png: Path) -> Path:
    """
    Write the dark version of an image (`<name>_black.png`), whose lightness (of the
    HSL color model) is inverted and whose hues and chromas are kept.
    """
    im = Image.open(png)
    rgba = np.asarray(im.convert("RGBA"), dtype=float) / 255
    rgb = rgba[..., :3]
    # Shifting all the channels by the same amount keeps the hue and the chroma, and
    # the shift 1 - (max + min) maps the lightness (max + min)/2 to 1 - lightness
    shift = 1 - rgb.max(axis=-1, keepdims=True) - rgb.min(axis=-1, keepdims=True)
    rgba[..., :3] = np.clip(rgb + shift, 0, 1)
    out = Image.fromarray(np.round(rgba * 255).astype(np.uint8), "RGBA")
    if im.mode != "RGBA":
        out = out.convert(im.mode)
    path = png.with_name(f"{png.stem}_black.png")
    out.save(path, dpi=im.info.get("dpi", (DPI, DPI)))
    return path


# %%
if __name__ == "__main__":
    if "--dark-only" not in sys.argv:
        print_simulink(MODEL, SCREENSHOTS[1])
        print(f"Wrote {SCREENSHOTS[1]}")
    for png in SCREENSHOTS:
        print(f"Wrote {dark_version(png)}")
