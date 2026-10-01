# AGENTS.md

This file provides guidance to AI coding agents working with code in this repository.

## Commands

Development uses a virtual environment in `.venv` (pyright is configured to use it). Install with `pip install -e .[dev]`.

- Lint and format: `pre-commit run --all-files`
- Type check: `pyright`
- Run the tests (require gcc): `pytest`
- Run a comparison with PLECS (requires PLECS Standalone with the RPC interface on port 1080): `python examples/ipmsm_2kw_fvc.py`
- Run a comparison with Simulink (requires MATLAB with a MinGW-w64, gcc, or clang compiler for `mex` and the MATLAB Engine API for Python matching the MATLAB release): `python examples/ipmsm_2kw_fvc_simulink.py`

## Architecture

- `motulator_export/plecs/sm.py`, `im.py`, and `grid.py` write PLECS models of synchronous machine drives, induction machine drives, and grid converter systems. Each provides `write_model` and `simulate`. The shared building blocks are in `_schematic.py` (the PLECS file format: the schematic writer `_Schematic`, probes, scopes, masks, and C-Script parameters), `_common.py` (the step signals, the control-system block `ControlBlock` and its C-Script code helpers, the converter, the DC bus, and the model file), `_drive.py` (the mechanics, the speed controller, and the scope), and `_rpc.py` (the simulation via the RPC interface). The package `motulator_export` exports `StepSignal` and `sampled_step`, which both exports use.
- `motulator_export/c/` is the C port of the motulator control algorithms, run in the C-Script blocks of PLECS and the S-functions of Simulink. It follows the Python code of motulator closely, including the order of the state updates, and the tests in `tests/` compare it with motulator step by step (the C test APIs are compiled with the helpers in `tests/c_port.py`). A change in the control algorithms of motulator must be mirrored here. The tests pass NaN for the optional parameters so that the default values are resolved in C, as in motulator.
- `motulator_export/simulink/` writes Simulink models (`sm.py`, `im.py`, `grid.py`). The control system is the C-Script code of the PLECS model wrapped in a generated C MEX S-function (`_sfunction.py`), and the models are built in MATLAB by `build_drive.m` and `build_grid.m` with the common parts in the package `+blocks`. `tests/test_simulink.py` tests the generated S-functions against motulator with a mock of the Simulink API (`tests/simulink_mock`). On Windows, the tests can use the MinGW-w64 gcc of MATLAB (on the PATH).
- `examples/` contains the comparison scripts and the generated models (the Simulink build scripts and S-function sources in `examples/simulink/`). Regenerate the models by running the scripts after changing the writers, and check that the differences stay at the level recorded in `examples/README.md` and `motulator_export/simulink/README.md` (update the tables if they change).

## Conventions

- Follow the conventions of motulator: peak-valued complex space vectors, SI units, NumPy-style docstrings, and line length 88.
- PLECS: the schematic coordinates are chosen so that wires do not cross and the labels are below the blocks. Only the signals fed back to the control system are measured with meters; the other signals use PLECS probes.
- Simulink: the blocks are aligned with the ports they connect to (`blocks.align`), so that the lines are straight, and the feedback lines are drawn through given corners (`blocks.route`).
