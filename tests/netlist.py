"""
Execution of the control systems, whose C blocks are S-functions.

The C blocks of the netlist are generated as S-functions and compiled with gcc
against the mock of the Simulink API (`test_simulink.compile_sfunction`), and the
netlist is executed as Simulink executes the model: the outputs of the blocks in
the order of their direct feedthrough, then the state updates. The interface is
that of the S-function of a whole control system (`sfun_step`), so that `Netlist`
can be used in place of it in the closed-loop simulations of `test_simulink`.

"""

import ctypes
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

from motulator_export.plecs._common import MONITOR
from motulator_export.plecs._netlist import (
    CBlock,
    Mux,
    Selector,
    Subsystem,
    Tag,
    leaves,
    port_ref,
)
from motulator_export.simulink._sfunction import block_sfunction
from tests.c_port import arr

Expr = tuple[Any, ...]


class Netlist:
    """
    Execution of a netlist of S-functions (as `test_simulink.SFunctionControlSystem`
    runs the S-function of a whole control system, `sfun_step`).
    """

    def __init__(
        self,
        top: Subsystem,
        out_dir: Path,
        compile_sfunction: Callable[[Any, Path], ctypes.CDLL],
    ) -> None:
        self.top = top
        self.blocks = dict(leaves(top))

        def build(item: tuple[str, CBlock]) -> tuple[str, ctypes.CDLL]:
            path, block = item
            folder = out_dir / path.replace("/", "_").replace(" ", "_")
            folder.mkdir()
            return path, compile_sfunction(block_sfunction(block), folder)

        with ThreadPoolExecutor() as pool:
            self.dlls = dict(pool.map(build, self.blocks.items()))
        # Sources of the inputs of the C blocks and of the outputs of the top level
        self.inputs = {
            path: [self._dst([(top, "")], path, p.name) for p in b.inputs]
            for path, b in self.blocks.items()
        }
        self.outputs = [self._dst([(top, "")], "", p.name) for p in top.outputs]

    # The sources as expressions: ("in", name), ("leaf", path, k), ("sel", indices,
    # expr), and ("mux", [expr, ...]). A chain is the list of the subsystems from the
    # top level, with their paths.
    def _dst(self, chain: list[tuple[Subsystem, str]], path: str, port: str) -> Expr:
        """Source of an input of a C block (or of an output of the top level)."""
        sub = chain[-1][0]
        names = path.split("/") if path else []
        for name in names[:-1]:
            block = sub.block(name)
            assert isinstance(block, Subsystem)
            chain = [*chain, (block, f"{chain[-1][1]}/{name}".strip("/"))]
            sub = block
        return self._source(chain, f"{names[-1]}:{port}" if names else port)

    def _source(self, chain: list[tuple[Subsystem, str]], dst: str) -> Expr:
        """Source of a port of the subsystem of the chain."""
        sub = chain[-1][0]
        src = next(
            port_ref(sub, s) for s, d in sub.connections if port_ref(sub, d) == dst
        )
        if ":" not in src:  # An input of the subsystem
            if len(chain) == 1:
                return ("in", src)
            return self._source(chain[:-1], f"{sub.name}:{src}")
        return self._output(chain, *src.split(":", 1))

    def _output(self, chain: list[tuple[Subsystem, str]], name: str, port: str) -> Expr:
        """Expression of an output of a block of the subsystem of the chain."""
        sub, base = chain[-1]
        b = sub.block(name)
        path = f"{base}/{name}".strip("/")
        if isinstance(b, CBlock):
            return ("leaf", path, [p.name for p in b.outputs].index(port))
        if isinstance(b, Selector):
            return ("sel", b.indices, self._source(chain, f"{name}:u"))
        if isinstance(b, Mux):
            return ("mux", [self._source(chain, f"{name}:{p.name}") for p in b.inputs])
        if isinstance(b, Tag):  # The source of the Goto block of the tag
            gotos = [t for t in sub.blocks if isinstance(t, Tag) and t.goto]
            goto = next(t for t in gotos if t.tag == b.tag)
            return self._source(chain, f"{goto.name}:u")
        assert isinstance(b, Subsystem)
        return self._source([*chain, (b, path)], port)

    def start(
        self,
        params: dict[str, list[Any]],
        start: Callable[[ctypes.CDLL, list[Any]], str | None],
    ) -> str | None:
        """Start the S-functions with their parameters, returning an error message."""
        for path in self.blocks:
            err = start(self.dlls[path], params[path])
            if err is not None:
                return f"{path}: {err}"
        return None

    def sfun_step(self, u: Any, y: Any) -> None:
        """One sampling period: the outputs of the top level, then the updates."""
        values = list(u)
        self.u: dict[str, np.ndarray] = {}
        for p in self.top.inputs:
            self.u[p.name] = np.array(values[: p.width])
            values = values[p.width :]
        self.memo: dict[str, list[np.ndarray]] = {}
        self.busy: set[str] = set()
        out = [self._eval(e) for e in self.outputs]
        for path in self.blocks:  # Every block computes its outputs
            self._leaf(path)
        for path in self.blocks:
            inputs = np.concatenate([self._eval(e) for e in self.inputs[path]])
            self.dlls[path].sfun_update(arr(inputs))
        signals = self.memo[MONITOR]
        flat = np.concatenate(out + signals)
        for k, v in enumerate(flat):
            y[k] = v

    def _eval(self, e: Expr) -> np.ndarray:
        if e[0] == "in":
            return self.u[e[1]]
        if e[0] == "leaf":
            return self._leaf(e[1])[e[2]]
        if e[0] == "sel":
            return self._eval(e[2])[e[1]]
        return np.concatenate([self._eval(x) for x in e[1]])

    def _leaf(self, path: str) -> list[np.ndarray]:
        if path in self.memo:
            return self.memo[path]
        if path in self.busy:
            raise RuntimeError(f"Algebraic loop at {path}")
        self.busy.add(path)
        b = self.blocks[path]
        feedthrough = b.feedthrough or [1] * len(b.inputs)
        # The inputs without direct feedthrough are not available (NaN)
        u = np.concatenate(
            [
                self._eval(e) if ft else np.full(p.width, np.nan)
                for e, ft, p in zip(
                    self.inputs[path], feedthrough, b.inputs, strict=True
                )
            ]
        )
        widths = [p.width for p in b.outputs]
        y = (ctypes.c_double * sum(widths))()
        self.dlls[path].sfun_outputs(arr(u), y)
        values = np.array(y)
        self.memo[path] = np.split(values, np.cumsum(widths)[:-1])
        self.busy.remove(path)
        return self.memo[path]

    def sfun_terminate(self) -> None:
        for dll in self.dlls.values():
            dll.sfun_terminate()
