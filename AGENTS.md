# AGENTS.md

This file provides guidance to AI coding agents working with code in this repository.

## Commands

Development uses a virtual environment in `.venv` (pyright is configured to use it). Install with `pip install -e .[dev]`.

- Lint and format: `pre-commit run --all-files`
- Type check: `pyright`
- Run the tests (require gcc): `pytest`
- Run a comparison with PLECS (requires PLECS Standalone with the RPC interface on port 1080): `python examples/ipmsm_2kw_fvc.py`

## Architecture

- `motulator_plecs/sm.py`, `im.py`, and `grid.py` write PLECS models of synchronous machine drives, induction machine drives, and grid converter systems. Each provides `write_model` and `simulate`. The shared building blocks are in `_common.py` (the schematic writer `_Schematic`, the control-system block `ControlBlock`, the converter, the DC bus, and the simulation via the RPC interface) and `_drive.py` (the mechanics, the speed controller, and the scope).
- `motulator_plecs/c/` is the C port of the motulator control algorithms, run in C-Script blocks. It follows the Python code of motulator closely, including the order of the state updates, and the tests in `tests/` compare it with motulator step by step. A change in the control algorithms of motulator must be mirrored here.
- `examples/` contains the comparison scripts and the generated models. Regenerate the models by running the scripts after changing the writers, and check that the differences stay at the previous level.

## Conventions

- Follow the conventions of motulator: peak-valued complex space vectors, SI units, NumPy-style docstrings, and line length 88.
- The schematic coordinates are chosen so that wires do not cross and the labels are below the blocks. Only the signals fed back to the control system are measured with meters; the other signals use PLECS probes.
