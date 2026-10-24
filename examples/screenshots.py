"""
Screenshots of the models for the documentation
===============================================

This script prints the Simulink model of `pmsyrm_6kw_gn_fvc_simulink.py`
(simulink/pmsyrm_6kw_gn_fvc.slx, built by the script) to
docs/source/images/simulink_drive.png with the MATLAB Engine API for Python. The
PLECS screenshots in the same folder are not made here: they are exported from PLECS,
see examples/README.md.

With --dark, the script writes instead the dark versions (`*_black.png`) of all the
screenshots for the dark mode of the documentation and GitHub (MATLAB is not
needed). The dark version of an image inverts its lightness and keeps its hues:
white becomes black, black becomes white, and, e.g., the green signals stay green.

Run from the repository root:

    python examples/screenshots.py [--dark]

"""

# %%
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

HERE = Path(__file__).parent
SIMULINK = HERE.parent / "motulator_export" / "simulink"  # The package +motulator
MODEL = HERE / "simulink" / "pmsyrm_6kw_gn_fvc.slx"
IMAGES = HERE.parent / "docs" / "source" / "images"
SCREENSHOT = IMAGES / "simulink_drive.png"
DPI = 300


def print_simulink(model: Path, png: Path) -> None:
    """Print the root level of a Simulink model to a PNG file."""
    import matlab.engine  # noqa: PLC0415  # pyright: ignore[reportMissingImports]

    eng: Any = matlab.engine.start_matlab()
    try:
        # The package of motulator and its library on the path
        eng.addpath(str(SIMULINK), nargout=0)
        eng.eval("motulator.setup();", nargout=0)
        eng.load_system(str(model), nargout=0)
        # Subsystems as plain blocks, without the previews of their contents (the
        # blocks of the library are masked, and the links keep their settings)
        eng.eval(
            f"b = find_system('{model.stem}', 'SearchDepth', 1, 'BlockType', "
            "'SubSystem'); for k = 1:numel(b), try, set_param(b{k}, "
            "'ContentPreviewEnabled', 'off'); catch, end, end",
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
    if "--dark" in sys.argv:
        for png in sorted(IMAGES.glob("*.png")):
            if not png.stem.endswith("_black"):
                print(f"Wrote {dark_version(png)}")
    else:
        print_simulink(MODEL, SCREENSHOT)
        print(f"Wrote {SCREENSHOT}")
