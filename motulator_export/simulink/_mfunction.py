"""
MATLAB functions of the blocks of the control systems, run in MATLAB Function blocks.

The MATLAB code of a block (`MCode` of a C block, see `plecs._netlist`) is wrapped in
a MATLAB function with the inputs and outputs of the block and its parameters as the
struct `p`, with the state in persistent variables. The functions of the MATLAB port
of motulator (`motulator_export/matlab`) used by the code are appended as local
functions, so that the file of the function is self-contained. The function runs
the code sections as the S-function runs the C-Script code: the outputs are
computed from the inputs of the same sampling instant, and then the state is
updated. The file is in the folder of the library, and the MATLAB Function block
only calls the function (see `motulator.internal.add_mfunction`), so that the code of
the block is in the file, as that of an S-function is in its C source. Each MATLAB
Function block has its own copy of the persistent variables.

A MATLAB Function block has direct feedthrough from all its inputs, unlike the
S-functions, whose inputs used only in the update of the state are not direct
feedthrough (e.g., the limited torque reference of the speed controller, which
closes a loop through the vector controller). Such inputs come through unit delays,
and the update is deferred to the start of the next call, which gives the same
result: the update of the sampling instant k with the inputs of k is done before
the outputs of k + 1.

The parameters of the block are expressions in the workspace of its mask, or of the
masks of the subsystems containing it. The mask initialization of the block collects
them into the struct p (`mask_init`), an empty parameter ([], None in motulator)
being NaN, since a MATLAB Function block does not accept empty parameters, and checks
them.

"""

import re
import textwrap
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import cache
from pathlib import Path

from motulator_export.plecs._netlist import (
    COMPLEX,
    CBlock,
    Mask,
    MCode,
    Param,
    SignalVector,
    Subsystem,
    m_variable,
)

# Directory of the MATLAB port of motulator, and its files in the order of the local
# functions
M_SOURCES = Path(__file__).parents[1] / "matlab"
M_FILES = (
    "common.m",
    "gradnet.m",
    "sm_parameters.m",
    "sm_control_loci.m",
    "sm_flux_vector.m",
    "sm_current_vector.m",
    "im_current_vector.m",
    "im_flux_vector.m",
    "gfl_current_vector.m",
    "gfm_observer.m",
    "sm_machine.m",
)
# Type of the masks of the blocks without a mask in the C-Script code
PARAMS_MASK = "Parameters (motulator_export)"


@dataclass
class MFunction:
    """
    MATLAB function of a block: its name and source, the widths of its outputs, the
    parameters (the fields of p and their expressions), the sample time (the field
    of p, or None for an inherited sample time), the inputs through unit delays
    (zero-based indices), and the argument of the parameters (`MCode.param`).
    """

    name: str
    source: str
    output_widths: list[int]
    params: list[tuple[str, str]]
    sample_time: str | None
    delayed: list[int] = field(default_factory=list)
    param: str = "p"

    def write(self, folder: str | Path) -> Path:
        """Write the source of the function in the folder."""
        path = Path(folder) / f"{self.name}.m"
        path.write_text(self.source, encoding="utf-8")
        return path


# %%
# MATLAB code
def functions(text: str) -> dict[str, str]:
    """
    Functions of MATLAB code by their names, each from a line starting with
    "function" to the next one (or to the end), without the comments before the
    first function.
    """
    out: dict[str, str] = {}
    pattern = r"^function\s+(?:(?:\[[^\]]*\]|\w+)\s*=\s*)?(\w+)"
    starts = list(re.finditer(pattern, text, re.M))
    for m, end in zip(starts, [*starts[1:], None][: len(starts)], strict=True):
        source = text[m.start() : None if end is None else end.start()]
        out[m[1]] = source.rstrip() + "\n"
    return out


@cache
def library() -> dict[str, str]:
    """Functions of the MATLAB port of motulator by their names."""
    out: dict[str, str] = {}
    for f in M_FILES:
        out |= functions((M_SOURCES / f).read_text(encoding="utf-8"))
    return out


def code_only(code: str) -> str:
    """MATLAB code without the comments and the character vectors."""
    lines = []
    for line in code.splitlines():
        kept = []
        prev = ""  # The previous character that is not a space
        in_str = False
        k = 0
        while k < len(line):
            c = line[k]
            if in_str:
                if c == "'" and line[k + 1 : k + 2] == "'":
                    k += 2
                    continue
                in_str = c != "'"
            elif c == "%":
                break
            elif c == "'" and not (prev.isalnum() or prev in "_)]}.'"):
                in_str = True  # Else a transpose
            else:
                kept.append(c)
                prev = c if not c.isspace() else prev
            k += 1
        lines.append("".join(kept))
    return "\n".join(lines)


def identifiers(code: str) -> set[str]:
    """Identifiers in MATLAB code, excluding the fields of structs."""
    return set(re.findall(r"(?<![\w.])([A-Za-z]\w*)", code_only(code)))


def local_functions(code: str, available: dict[str, str]) -> list[str]:
    """
    Sources of the functions of `available` called by the code, directly or via
    other functions, in the order of `available`.
    """
    used: set[str] = set()
    todo = [code]
    while todo:
        for name in identifiers(todo.pop()) & set(available) - used:
            used.add(name)
            todo.append(available[name])
    return [src for name, src in available.items() if name in used]


def _indent(code: str, n: int = 1) -> str:
    """Code indented by n levels."""
    pad = "    " * n
    return "".join(f"{pad}{line}\n" if line else "\n" for line in code.splitlines())


# %%
# Signal vectors and conversions
def vector_functions(v: SignalVector) -> dict[str, str]:
    """Pack and unpack functions of a signal vector."""
    k = 1
    fields, values = [], []
    for name, width in v.fields:
        if width == 2:
            fields.append(f"    '{name}', complex(u({k}), u({k + 1}))")
            values += [f"real(s.{name})", f"imag(s.{name})"]
        else:
            fields.append(f"    '{name}', u({k})")
            values.append(f"s.{name}")
        k += width
    unpack, pack = f"{v.prefix}_unpack", f"{v.prefix}_pack"
    rows = ", ...\n".join(fields)
    items = ";\n    ".join(values)
    return {
        unpack: f"function s = {unpack}(u)\n"
        f"%{unpack.upper()} Struct of the signal vector u.\n"
        f"s = struct( ...\n{rows});\n"
        "end\n",
        pack: f"function y = {pack}(s)\n"
        f"%{pack.upper()} Signal vector of the struct s.\n"
        f"y = [ ...\n    {items}];\n"
        "end\n",
    }


def _from_signal(kind: SignalVector | str, arg: str) -> str:
    """Expression converting a signal to its variable."""
    if kind == COMPLEX:
        return f"complex_in({arg})"
    assert isinstance(kind, SignalVector)
    return f"{kind.prefix}_unpack({arg})"


def _to_signal(kind: SignalVector | str, var: str) -> str:
    """Expression converting a variable to its signal."""
    if kind == COMPLEX:
        return f"complex_out({var})"
    assert isinstance(kind, SignalVector)
    return f"{kind.prefix}_pack({var})"


# %%
# Blocks
def function_name(block: CBlock) -> str:
    """Name of the MATLAB function of a block, e.g., mfun_fvc_speed_controller."""
    return re.sub(r"^sfun_", "mfun_", block.sfunction)


def param_field(expr: str) -> str:
    """Field of p of a parameter, e.g., T_s for cfg.T_s."""
    name = expr.rpartition(".")[2]
    if not name.isidentifier():
        raise ValueError(f"No field for the parameter {expr}")
    return name


def block_params(block: CBlock) -> list[tuple[str, str]]:
    """Parameters of a block as the fields of p and their expressions."""
    params = [(param_field(e), e) for e in block.params]
    if len({f for f, _ in params}) != len(params):
        raise ValueError(f"{block.name}: duplicate fields in {block.params}")
    return params


def delayed_inputs(block: CBlock) -> list[int]:
    """Inputs without direct feedthrough, which come through unit delays."""
    feedthrough = block.feedthrough or [1] * len(block.inputs)
    return [k for k, ft in enumerate(feedthrough) if not ft]


def block_mfunction(block: CBlock) -> MFunction:
    """MATLAB function of a block of a control system (see the module docstring)."""
    m = block.mcode
    if m is None:
        raise ValueError(f"{block.name}: no MATLAB code")
    name = function_name(block)
    delayed = delayed_inputs(block)
    if delayed and block.sample_time is None:
        raise ValueError(f"{block.name}: delayed inputs need a sample time")
    params = block_params(block)
    args = [_arg(p.name, m.inputs, "in") for p in block.inputs]
    outs = [_arg(p.name, m.outputs, "out") for p in block.outputs]
    current = [k for k in range(len(block.inputs)) if k not in delayed]
    body = m.header + _state(m, delayed)
    if delayed:
        names = ", ".join(block.inputs[k].name for k in delayed)
        body += (
            "% Update of the previous sampling period with its inputs (the inputs\n"
            f"% {names} through unit delays)\n"
            "if pending\n"
            + _indent(_conversions(block, args, delayed) + m.update)
            + "end\n"
            "pending = true;\n"
        )
    body += m.prologue
    body += _conversions(block, args, current) + m.output
    body += "".join(
        f"{outs[k]} = {_to_signal(m.outputs[p.name], m_variable(p.name))};\n"
        for k, p in enumerate(block.outputs)
        if p.name in m.outputs
    )
    if not delayed:
        body += m.update
    arguments = ", ".join(args + ([m.param] if params else []))
    signature = f"function {_outputs(outs)}{name}({arguments})"
    main = f"{signature}\n{_header(block, name, delayed)}{body}end\n"
    available = functions(m.functions)
    for kind in (*m.inputs.values(), *m.outputs.values()):
        if isinstance(kind, SignalVector):
            available |= vector_functions(kind)
    available |= {k: v for k, v in library().items() if k not in available}
    return MFunction(
        name,
        "\n".join([main, *local_functions(main, available)]),
        [p.width for p in block.outputs],
        params,
        None if block.sample_time is None else param_field(block.sample_time),
        delayed,
        m.param,
    )


def _arg(port: str, kinds: dict[str, SignalVector | str], suffix: str) -> str:
    """Argument of a port: its variable, with the suffix if it is converted."""
    var = m_variable(port)
    return f"{var}_{suffix}" if port in kinds else var


def _conversions(block: CBlock, args: list[str], ports: list[int]) -> str:
    """Conversions of the inputs `ports` from their signals to their variables."""
    m = block.mcode
    assert m is not None
    return "".join(
        f"{m_variable(p.name)} = {_from_signal(m.inputs[p.name], args[k])};\n"
        for k, p in enumerate(block.inputs)
        if k in ports and p.name in m.inputs
    )


def _state(m: MCode, delayed: list[int]) -> str:
    """Persistent variables of the state and their initialization."""
    if not m.start:
        return ""
    state = ["self"]
    init = m.start
    if delayed:
        state.append("pending")
        init += "pending = false;\n"
    return f"persistent {' '.join(state)}\nif isempty(self)\n{_indent(init)}end\n"


def _outputs(outs: list[str]) -> str:
    """Outputs of a function signature."""
    if not outs:
        return ""
    if len(outs) == 1:
        return f"{outs[0]} = "
    return f"[{', '.join(outs)}] = "


def _header(block: CBlock, name: str, delayed: list[int]) -> str:
    """Help text of the function of a block."""
    text = (
        "" if block.mask is None else f"{block.mask.type.split(' (')[0]} of motulator. "
    )
    param = "p" if block.mcode is None else block.mcode.param
    text += (
        "Generated by motulator_export from the MATLAB port of motulator and called "
        "by the MATLAB Function block of the library of motulator, whose folder "
        "motulator.setup adds to the MATLAB path. The parameters are the fields of "
        f"the struct {param} (from the mask, [] as NaN)."
    )
    if block.mcode is not None and block.mcode.start:
        text += (
            " The state is in persistent variables, of which each MATLAB Function "
            "block has its own copy."
        )
    if delayed:
        text += (
            " The inputs without direct feedthrough come through unit delays, so the "
            "update of the state with them is done at the next call."
        )
    lines = textwrap.wrap(text, 84)
    return f"%{name.upper()} Block {block.name} of the control system.\n" + "".join(
        f"%   {line}\n" for line in lines
    )


def gradnet_machine_mfunction() -> MFunction:
    """
    MATLAB function of the synchronous machine with a GradNet current map with
    spatial harmonics (the MATLAB port of the C-Script code of the PLECS model).

    The inputs are the stator voltage in stationary coordinates, the stator flux
    linkage in rotor coordinates (the state of an integrator, whose initial value is
    machine.psi_f), the rotor speed, and the rotor angle, and the outputs are the
    derivative of the flux linkage, the phase currents, and the torque. The
    parameters are in the struct `machine` of the initialization script.

    """
    name = "mfun_gradnet_machine"
    main = (
        f"function [d_psi_s_dq, i_s_abc, tau_M] = {name}(u_s_ab, psi_s_dq, w_M, "
        "theta_M, machine)\n"
        f"%{name.upper()} Synchronous machine with a GradNet current map with\n"
        "%   spatial harmonics (SpatialSaturatedSynchronousMachinePars) in rotor\n"
        "%   coordinates, as SynchronousMachine of motulator. Generated by\n"
        "%   motulator_export from the MATLAB port of motulator and called by the\n"
        "%   MATLAB Function block of the machine model of the library of motulator,\n"
        "%   whose folder motulator.setup adds to the MATLAB path.\n"
        "persistent par\n"
        "if isempty(par)\n"
        "    g = machine.current_map;\n"
        "    current_map = gradnet(g.in_dim, g.mu_dim, g.W, g.b, g.mu_log, ...\n"
        "        g.bias, g.activation, g.beta_log, g.p, g.in_base, g.out_base);\n"
        "    par = struct('n_p', machine.n_p, 'R_s', machine.R_s, ...\n"
        "        'current_map', current_map, 'k', machine.k, ...\n"
        "        'psi_f', machine.psi_f);\n"
        "end\n"
        "psi = complex(psi_s_dq(1), psi_s_dq(2));\n"
        "theta_m = par.n_p*theta_M;\n"
        "[i_s_dq, tau_M] = machine_magnetic_map(par, psi, theta_m);\n"
        "% Phase currents\n"
        "i_s_abc = complex2abc(i_s_dq*exp(1i*theta_m));\n"
        "% Voltage equation in rotor coordinates\n"
        "d_psi = machine_flux_rhs(par, complex(u_s_ab(1), u_s_ab(2)), psi, theta_M, "
        "w_M);\n"
        "d_psi_s_dq = [real(d_psi); imag(d_psi)];\n"
        "end\n"
    )
    source = "\n".join([main, *local_functions(main, library())])
    return MFunction(name, source, [2, 3, 1], [], None)


def reference_luts_mfunction() -> MFunction:
    """
    MATLAB function `sm_reference_luts.m` of the lookup tables of the reference
    generator of synchronous machines (ReferenceGenerator), called by the mask
    initialization of the block in Simulink (`Mask.simulink_init`), so that the
    block does not compute the loci at its start (which matters on real-time
    targets). The arguments are the machine model `par` of the model initialization
    and the maximum current.
    """
    name = "sm_reference_luts"
    main = (
        f"function luts = {name}(par, i_s_max)\n"
        f"%{name.upper()} Lookup tables of the flux and torque reference generator\n"
        "%   (ReferenceGenerator of motulator) of the machine model par at the\n"
        "%   maximum current i_s_max (A): a 16-by-6 matrix whose columns are the\n"
        "%   sample points and the values of the MTPA flux magnitude vs. the torque,\n"
        "%   the current-limit torque vs. the flux magnitude, and the MTPV torque vs.\n"
        "%   the flux magnitude. The machine model is the struct of the model\n"
        "%   initialization: the fields n_p, R_s, L_d, L_q, and psi_f of\n"
        "%   SynchronousMachinePars, or n_p, R_s, and the GradNet flux map\n"
        "%   psi_s_dq_fcn of SaturatedSynchronousMachinePars. Called by the mask\n"
        "%   initialization of the block ReferenceGenerator, so this file must be on\n"
        "%   the MATLAB path (in the folder of the library, see motulator.setup).\n"
        "%   Generated by motulator_export from the MATLAB port of motulator.\n"
        "if isfield(par, 'psi_s_dq_fcn') && ~isempty(par.psi_s_dq_fcn)\n"
        "    g = par.psi_s_dq_fcn;\n"
        "    flux_map = gradnet(g.in_dim, g.mu_dim, g.W, g.b, g.mu_log, g.bias, ...\n"
        "        g.activation, g.beta_log, g.p, g.in_base, g.out_base);\n"
        "    par = saturated_synchronous_machine_pars(par.n_p, par.R_s, flux_map);\n"
        "else\n"
        "    par = synchronous_machine_pars(par.n_p, par.R_s, par.L_d, par.L_q, ...\n"
        "        par.psi_f, gradnet_placeholder());\n"
        "end\n"
        "luts = compute_reference_luts(par, i_s_max);\n"
        "% Error of ReferenceGenerator of motulator (the MTPA locus is NaN without a\n"
        "% solution, see compute_mtpa_current_angle)\n"
        "if any(isnan(luts(:, 1:2)), 'all')\n"
        "    error('No MTPA solution found. Check the alignment of the d axis.');\n"
        "end\n"
        "end\n"
    )
    source = "\n".join([main, *local_functions(main, library())])
    return MFunction(name, source, [], [], None)


# Functions of the m-files of the library, which the mask initializations of
# Simulink (`Mask.simulink_init`) may call
PATH_FUNCTIONS: dict[str, Callable[[], MFunction]] = {
    "sm_reference_luts": reference_luts_mfunction
}


def path_functions(top: Subsystem) -> list[MFunction]:
    """
    The functions of `PATH_FUNCTIONS` called by the mask initializations of Simulink
    of the subsystem and the blocks in it, each once.
    """
    names: set[str] = set()

    def visit(sub: Subsystem) -> None:
        for b in sub.blocks:
            if isinstance(b, (CBlock, Subsystem)) and b.mask is not None:
                names.update(identifiers(b.mask.simulink_init) & set(PATH_FUNCTIONS))
            if isinstance(b, Subsystem):
                visit(b)

    visit(top)
    return [PATH_FUNCTIONS[n]() for n in PATH_FUNCTIONS if n in names]


# %%
# Masks
def mask_init(block: CBlock) -> str:
    """
    Initialization commands collecting the parameters of a block into the struct p,
    [] (None) as NaN, and checking them.
    """
    m = block.mcode
    assert m is not None
    params = block_params(block)
    text = (
        "% Parameters of the MATLAB function: the struct p, [] (None) as NaN\n"
        "p = struct();\n"
        + "".join(f"p.{f} = {f};\n" for f, _ in params)
        + "for f_ = fieldnames(p)'\n"
        "  if isempty(p.(f_{1}))\n"
        "    p.(f_{1}) = NaN;\n"
        "  else\n"
        "    p.(f_{1}) = double(p.(f_{1}));\n"
        "  end\n"
        "end\n"
    )
    text += "".join(f"if {cond}\n  error('{msg}');\nend\n" for cond, msg in m.checks)
    if m.param != "p":
        text += f"% The argument of the parameters of the function\n{m.param} = p;\n"
    return text


def block_mask(block: CBlock) -> Mask:
    """
    Mask of the MATLAB Function block of a block: its mask, or the parameters of
    the block, with the initialization collecting the parameters into p.
    """
    init = mask_init(block)
    if block.mask is not None:
        prefix = block.mask.init + block.mask.simulink_init
        prefix += "\n" if prefix else ""
        return Mask(
            block.mask.type, block.mask.description, block.mask.params, prefix + init
        )
    params = [Param(f, f"{f}: {e}", e) for f, e in block_params(block)]
    return Mask(
        PARAMS_MASK,
        f"Parameters of the block {block.name}, collected into the struct p of its "
        "MATLAB function.",
        params,
        init,
    )
