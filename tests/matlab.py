"""
Interpreter of the subset of MATLAB used in the model initializations and the mask
initializations of the control systems, for the tests without MATLAB and PLECS.

The statements are assignments (also to the fields of structs) and if-elseif-else
blocks, and the expressions are numbers, matrices, variables, fields, the
arithmetic, comparison, and logical operators, and the functions isempty,
isfield, isinf, and strcmp. The variables are resolved through the workspaces of
the masks, as in Simulink, or only in the workspace of the innermost mask, as in
PLECS (`block_params`).

"""

import re
from collections import ChainMap
from collections.abc import MutableMapping
from math import inf, nan, pi
from typing import Any

import numpy as np

from motulator_export.plecs._common import workspace_variables
from motulator_export.plecs._netlist import CBlock, Subsystem


class Struct(dict):
    """MATLAB struct."""

    def __getattr__(self, name: str) -> Any:
        return self[name]


TOKEN = re.compile(
    r"(?P<num>(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)|(?P<id>[A-Za-z_]\w*)"
    r"|(?P<str>'[^']*')|(?P<op>&&|\|\||==|~=|<=|>=|[-+*/^~<>()\[\];,.=])|(?P<ws>\s+)"
)
CONSTANTS = {"pi": pi, "NaN": nan, "nan": nan, "Inf": inf, "inf": inf}
CONSTANTS |= {"true": 1.0, "false": 0.0}
BINARY = {"||": 1, "&&": 2, "==": 3, "~=": 3, "<": 3, ">": 3, "<=": 3, ">=": 3}
BINARY |= {"+": 4, "-": 4, "*": 5, "/": 5, "^": 7}


def _tokens(text: str) -> list[tuple[str, str]]:
    """Tokens (kind, text), with the whitespace inside the brackets."""
    out: list[tuple[str, str]] = []
    depth = 0
    pos = 0
    while pos < len(text):
        m = TOKEN.match(text, pos)
        if m is None:
            raise SyntaxError(f"Unknown token at {text[pos:]!r}")
        pos = m.end()
        kind = m.lastgroup or ""
        if kind == "ws":
            if depth > 0:
                out.append(("ws", " "))
            continue
        tok = m.group()
        depth += {"[": 1, "]": -1}.get(tok, 0)
        out.append((kind, tok))
    return out


class _Parser:
    """Evaluation of an expression by precedence climbing."""

    def __init__(self, tokens: list[tuple[str, str]], ws: MutableMapping) -> None:
        self.t = tokens
        self.k = 0
        self.ws = ws
        self.brackets = 0
        self.skip = 0  # Parsing without evaluation (short-circuited)

    def peek(self, skip_ws: bool = True) -> tuple[str, str]:
        k = self.k
        while skip_ws and k < len(self.t) and self.t[k][0] == "ws":
            k += 1
        return self.t[k] if k < len(self.t) else ("end", "")

    def take(self) -> tuple[str, str]:
        while self.k < len(self.t) and self.t[self.k][0] == "ws":
            self.k += 1
        tok = self.t[self.k]
        self.k += 1
        return tok

    def expect(self, text: str) -> None:
        tok = self.take()
        if tok[1] != text:
            raise SyntaxError(f"Expected {text}, got {tok[1]}")

    def _separator(self) -> bool:
        """Whether whitespace in a matrix separates the elements here."""
        if self.brackets == 0 or self.k >= len(self.t) or self.t[self.k][0] != "ws":
            return False
        nxt = self.peek()
        if nxt[1] in ("+", "-"):
            # [a -b] has two elements, [a - b] one
            after = self.t[self.k + 2] if self.k + 2 < len(self.t) else ("end", "")
            return after[0] != "ws"
        return nxt[1] not in BINARY and nxt[1] not in ("]", ";", ",")

    def expr(self, min_prec: int = 1) -> Any:
        lhs = self.unary()
        while True:
            if self._separator():
                return lhs
            op = self.peek()[1]
            prec = BINARY.get(op)
            if prec is None or prec < min_prec:
                return lhs
            self.take()
            # The logical operators short-circuit: the right side is only parsed
            short = (op == "&&" and not _true(lhs)) or (op == "||" and _true(lhs))
            self.skip += short
            rhs = self.expr(prec + (op != "^"))
            self.skip -= short
            lhs = float(_true(lhs)) if short else _binary(op, lhs, rhs)

    def unary(self) -> Any:
        op = self.peek()[1]
        if op in ("-", "+", "~"):
            self.take()
            value = self.expr(6)
            if op == "-":
                return -value
            if op == "~":
                return float(not _true(value))
            return value
        return self.postfix()

    def postfix(self) -> Any:
        kind, tok = self.take()
        if kind == "num":
            value: Any = float(tok)
        elif kind == "str":
            value = tok[1:-1]
        elif tok == "(":
            brackets, self.brackets = self.brackets, 0
            value = self.expr()
            self.brackets = brackets
            self.expect(")")
        elif tok == "[":
            value = self.matrix()
        elif kind == "id":
            if self.peek(skip_ws=False)[1] == "(":
                self.take()
                brackets, self.brackets = self.brackets, 0
                args = [self.expr()]
                while self.peek()[1] == ",":
                    self.take()
                    args.append(self.expr())
                self.brackets = brackets
                self.expect(")")
                return 0.0 if self.skip else _call(tok, args)
            if self.skip:
                value = 0.0
            else:
                value = self.ws[tok] if tok in self.ws else CONSTANTS[tok]
        else:
            raise SyntaxError(f"Unexpected {tok}")
        while self.peek(skip_ws=False)[1] == ".":
            self.take()
            name = self.take()[1]
            value = 0.0 if self.skip else value[name]
        return value

    def matrix(self) -> Any:
        self.brackets += 1
        rows: list[list[Any]] = [[]]
        while True:
            tok = self.peek()[1]
            if tok == "]":
                self.take()
                break
            if tok == ";":
                self.take()
                rows.append([])
                continue
            if tok == ",":
                self.take()
                continue
            rows[-1].append(self.expr())
        self.brackets -= 1
        rows = [r for r in rows if r]
        if not rows:
            return np.zeros((0, 0))
        if len(rows) == 1:
            return np.hstack([np.atleast_1d(v) for v in rows[0]]).astype(float)
        return np.array(rows, dtype=float)


def _true(value: Any) -> bool:
    return bool(np.size(value) > 0 and np.all(value))


def _binary(op: str, a: Any, b: Any) -> Any:
    if op == "||":
        return float(_true(a) or _true(b))
    if op == "&&":
        return float(_true(a) and _true(b))
    ops = {
        "+": lambda: a + b,
        "-": lambda: a - b,
        "*": lambda: a * b,
        "/": lambda: a / b,
        "^": lambda: a**b,
        "==": lambda: a == b,
        "~=": lambda: a != b,
        "<": lambda: a < b,
        ">": lambda: a > b,
        "<=": lambda: a <= b,
        ">=": lambda: a >= b,
    }
    value = ops[op]()
    return value.astype(float) if isinstance(value, np.ndarray) else float(value)


def _call(name: str, args: list[Any]) -> Any:
    if name == "isempty":
        return float(not isinstance(args[0], dict) and np.size(args[0]) == 0)
    if name == "isfield":
        return float(isinstance(args[0], dict) and args[1] in args[0])
    if name == "isinf":
        return float(np.all(np.isinf(args[0])))
    if name == "strcmp":
        return float(isinstance(args[0], str) and args[0] == args[1])
    raise NameError(name)


def evaluate(text: str, ws: MutableMapping) -> Any:
    """Value of an expression in a workspace."""
    parser = _Parser(_tokens(text), ws)
    value = parser.expr()
    if parser.peek()[0] != "end":
        raise SyntaxError(f"Unexpected {parser.peek()[1]} in {text!r}")
    return value


def run(code: str, ws: MutableMapping) -> None:
    """Run the statements in a workspace."""
    lines = [line.split("%", 1)[0].strip() for line in code.splitlines()]
    lines = [line for line in lines if line]
    _block(lines, 0, ws, True)


def _block(lines: list[str], k: int, ws: MutableMapping, active: bool) -> int:
    """Run the statements from line k until an unmatched end/else/elseif."""
    while k < len(lines):
        line = lines[k]
        word = line.split()[0].rstrip(";")
        if word in ("end", "else", "elseif"):
            return k
        if word == "if":
            k = _if(lines, k, ws, active)
            continue
        if active:
            lhs, rhs = line.rstrip(";").split("=", 1)
            _assign(lhs.strip(), evaluate(rhs.strip(), ws), ws)
        k += 1
    return k


def _if(lines: list[str], k: int, ws: MutableMapping, active: bool) -> int:
    """Run an if block from its line k, returning the line after its end."""
    done = False
    cond = lines[k][2:]
    while True:
        take = active and not done and _true(evaluate(cond, ws))
        done = done or take
        k = _block(lines, k + 1, ws, take)
        word = lines[k].split()[0].rstrip(";")
        if word == "end":
            return k + 1
        cond = lines[k][6:] if word == "elseif" else "1"


def _assign(lhs: str, value: Any, ws: MutableMapping) -> None:
    names = lhs.split(".")
    if len(names) == 1:
        ws[names[0]] = value
        return
    if names[0] not in ws:
        ws[names[0]] = Struct()
    s = ws[names[0]]
    for name in names[1:-1]:
        s = s.setdefault(name, Struct())
    s[names[-1]] = value


# %%
def block_params(
    top: Subsystem, init: str, values: dict[str, str], plecs: bool = False
) -> dict[str, list[Any]]:
    """
    Parameters of the C blocks of a control system, by their paths, as the masks of
    the model compute them: the model initialization `init` is run, the values of
    the mask of the control-system block (`values`) are evaluated, and so on to the
    blocks, the mask initializations defining the variables of the masks.

    In Simulink, a mask sees the variables of the masks around it and of the model
    initialization. In PLECS (`plecs`), it sees only its own variables, the mask of
    the control-system block also passing the variables of the control system in
    the model initialization (`workspace_variables`).
    """
    base: dict[str, Any] = {}
    run(init, base)
    top_vars = {n: evaluate(v, base) for n, v in values.items()}
    if plecs:
        ws = ChainMap(top_vars | {n: base[n] for n in workspace_variables(init)})
    else:
        ws = ChainMap(top_vars, base)
    out: dict[str, list[Any]] = {}

    def visit(sub: Subsystem, ws: ChainMap, path: str) -> None:
        for b in sub.blocks:
            if not isinstance(b, (CBlock, Subsystem)):
                continue
            local = ws
            if b.mask is not None:
                params = {p.variable: evaluate(p.value, ws) for p in b.mask.params}
                local = ChainMap(params) if plecs else ws.new_child(params)
                run(b.mask.init, local)
            name = f"{path}/{b.name}" if path else b.name
            if isinstance(b, CBlock):
                out[name] = [evaluate(p, local) for p in b.params]
            else:
                visit(b, local, name)

    visit(top, ws, "")
    return out
