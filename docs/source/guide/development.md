# Development

Install the package from the cloned repository (see [](getting_started.md#installation)) with the development tools:

```bash
pip install -e ".[dev]"
```

- **Tests**: `pytest` compiles the C port with gcc and compares it, and the blocks of the exported control systems, with *motulator* step by step. Neither PLECS nor MATLAB is needed. The tests in MATLAB (`tests/test_matlab_function.py`: the MATLAB Function blocks, the libraries, and the models of the examples compared with the native simulation) need MATLAB and are skipped without it; run them before a release.
- **Documentation**: `pip install -e ".[doc]"` and `sphinx-build -b html docs/source docs/build/html`. The build runs the examples of the native simulation.
- **Comparisons**: the example scripts compare the PLECS and Simulink models with *motulator* (see [](getting_started.md#a-first-model)). Rerun them after changing the writers or the C port.
- **Libraries**: after changing the blocks of the control systems, the C or MATLAB port, or the MATLAB classes, rewrite the library files of the package with `python -m motulator_export.plecs._library` and `python -m motulator_export.simulink._library`; `tests/test_library.py` checks them.

The implementation is described in the developer notes of the repository:

- [examples/README.md](https://github.com/Aalto-Electric-Drives/motulator-export/blob/main/examples/README.md): the example scripts and the screenshots
- [motulator_export/plecs/README.md](https://github.com/Aalto-Electric-Drives/motulator-export/blob/main/motulator_export/plecs/README.md): the structure of the PLECS models and their agreement with *motulator*
- [motulator_export/simulink/README.md](https://github.com/Aalto-Electric-Drives/motulator-export/blob/main/motulator_export/simulink/README.md): the structure of the Simulink models and their agreement with *motulator*
- [motulator_export/c/README.md](https://github.com/Aalto-Electric-Drives/motulator-export/blob/main/motulator_export/c/README.md): the C port of the control algorithms and the C models and solver of the native simulation
- [motulator_export/matlab/README.md](https://github.com/Aalto-Electric-Drives/motulator-export/blob/main/motulator_export/matlab/README.md): the MATLAB port
