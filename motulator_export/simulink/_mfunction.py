"""
MATLAB functions of the blocks of the control systems, run in MATLAB Function blocks.

The MATLAB code of a block (`MCode` of a C block, see `plecs._netlist`) is wrapped in
a MATLAB function with the inputs and outputs of the block and its parameters as the
struct `p`, with the state in persistent variables. The functions of the MATLAB port
of motulator (`motulator_export/matlab`) used by the code are appended as local
functions, so that the file of the function is self-contained. The function runs
the code sections as the S-function runs the C-Script code: the outputs are
computed from the inputs of the same sampling instant, and then the state is
updated. The file is in the folder `library` of the package, and the MATLAB Function
block contains a copy of its code (see `motulator.internal.add_mfunction`), so that a
model does not need the file. Each MATLAB Function block has its own copy of the
persistent variables.

A MATLAB Function block has direct feedthrough from all its inputs, unlike the
S-functions, whose inputs used only in the update of the state are not direct
feedthrough (e.g., the limited torque reference of the speed controller, which
closes a loop through the vector controller). Such inputs come through unit delays,
and the update is deferred to the start of the next call, which gives the same
result: the update of the sampling instant k with the inputs of k is done before
the outputs of k + 1.

The blocks have no masks. The parameters of a block are the fields of its struct in
the struct `ctrl_par` of the parameters of the blocks of the control system, nested as
the blocks (e.g., `ctrl_par.FluxVectorController.ReferenceGenerator`), to which the
MATLAB Function block refers. The struct is computed by the function of the
parameters of the control system (`params_mfunction`, e.g., `fvc_params.m`) from the
arguments of the top-level class of motulator; it is generated from the masks of the
netlist, so that it computes the parameters as the masks of the S-functions do.

"""

import re
import textwrap
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import cache
from pathlib import Path

from motulator_export.plecs._common import ControlBlock
from motulator_export.plecs._netlist import (
    COMPLEX,
    CBlock,
    MCode,
    SignalVector,
    Subsystem,
    leaves,
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


@dataclass
class MFunction:
    """
    MATLAB function of a block: its name and source, the widths of its outputs, the
    parameters (the fields of p and their expressions), the sample time (the field
    of p, or None for an inherited sample time), the inputs through unit delays
    (zero-based indices).
    """

    name: str
    source: str
    output_widths: list[int]
    params: list[tuple[str, str]]
    sample_time: str | None
    delayed: list[int] = field(default_factory=list)

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
    body = _state(m, delayed)
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
    body += _conversions(block, args, current) + m.output
    body += "".join(
        f"{outs[k]} = {_to_signal(m.outputs[p.name], m_variable(p.name))};\n"
        for k, p in enumerate(block.outputs)
        if p.name in m.outputs
    )
    if not delayed:
        body += m.update
    arguments = ", ".join(args + (["p"] if params else []))
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
    text += (
        "Generated by motulator_export from the MATLAB port of motulator; the MATLAB "
        "Function block of a model contains a copy of this file. The parameters are "
        "the fields of "
        "the struct p, the struct of the block in the struct ctrl_par of the "
        "parameters of the blocks of its control system (e.g., "
        "ctrl_par.SpeedController, see the function of the parameters of the control "
        "system, e.g., fvc_params; [] as NaN)."
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
        "%   motulator_export from the MATLAB port of motulator; the MATLAB Function\n"
        "%   block of the machine model of a model contains a copy of this file.\n"
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
    initialization of the block in Simulink (`Mask.simulink_init`) and by the
    functions of the parameters of the control systems (`params_mfunction`), so that
    the block does not compute the loci at its start (which matters on real-time
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
        "%   initialization of the block ReferenceGenerator (S-functions) and by the\n"
        "%   functions of the parameters of the control systems (MATLAB Function\n"
        "%   blocks, e.g., fvc_params), so this file must be on the MATLAB path (in\n"
        "%   the folder of the library, see motulator.setup).\n"
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
# Parameters of the blocks
# Variables of the generated function of the parameters, which the variables of the
# masks must not shadow
RESERVED = ("ctrl_par", "s", "p")

NUMERIC_FIELDS = (
    "function p = numeric_fields(p)\n"
    "%NUMERIC_FIELDS Fields of the struct p as doubles, [] (None) as NaN.\n"
    "names = fieldnames(p);\n"
    "for k = 1:numel(names)\n"
    "    if isempty(p.(names{k}))\n"
    "        p.(names{k}) = NaN;\n"
    "    else\n"
    "        p.(names{k}) = double(p.(names{k}));\n"
    "    end\n"
    "end\n"
    "end\n"
)


def struct_field(name: str) -> str:
    """Field of the struct of the parameters of a block, e.g., Monitored_signals."""
    return re.sub(r"\W+", "_", name).strip("_")


def struct_path(path: str) -> str:
    """
    Struct of the parameters of the block at a path of the control system, e.g.,
    ctrl_par.FluxVectorController.ReferenceGenerator.
    """
    return ".".join(["ctrl_par", *map(struct_field, path.split("/"))])


def params_name(block: ControlBlock) -> str:
    """
    Name of the function of the parameters of a control system, e.g., fvc_params,
    or fvc_torque_params in the torque-control mode.
    """
    names = {b.name: b for _, b in leaves(block.netlist())}
    prefix = function_name(next(iter(names.values()))).split("_")[1]
    return f"{prefix}{'_torque' if 'torque_reference' in names else ''}_params"


def _block_struct(block: CBlock, var: str) -> str:
    """Code collecting the parameters of a block into the struct `var`."""
    m = block.mcode
    assert m is not None
    text = "p = struct();\n" + "".join(
        f"p.{f} = {e};\n" for f, e in block_params(block)
    )
    text += "p = numeric_fields(p);\n"
    text += "".join(
        f"if {cond}\n    error('{block.name}: {msg}');\nend\n" for cond, msg in m.checks
    )
    return text + (f"{var} = p;\n" if var != "p" else "")


def _mask_code(init: str) -> str:
    """Initialization commands of a mask, indented as the generated code."""
    lines = init.splitlines()
    return "".join(
        " " * (2 * (len(x) - len(x.lstrip(" ")))) + x.lstrip(" ") + "\n" for x in lines
    )


def _scope(sub: Subsystem, var: str, path: str, funcs: list[str]) -> str:
    """
    Code setting the fields of the struct `var` of a subsystem: the structs of the
    parameters of its blocks. A block with a mask gets a local function of the
    arguments of its mask (appended to `funcs`), since the variables of a mask are
    those of its class.
    """
    text = f"{var} = struct();\n"
    for b in sub.blocks:
        if not isinstance(b, (CBlock, Subsystem)):
            continue
        field = f"{var}.{struct_field(b.name)}"
        child = f"{path}_{struct_field(b.name)}" if path else struct_field(b.name)
        if b.mask is not None:
            variables = [p.variable for p in b.mask.params]
            if set(variables) & set(RESERVED):
                raise ValueError(f"{b.name}: reserved variables in {variables}")
            k = len(funcs)  # Before the functions of the blocks in it
            funcs.append("")
            init = _mask_code(b.mask.init + b.mask.simulink_init)
            if isinstance(b, CBlock):
                body = init + _block_struct(b, "p")
                out = "p"
            else:
                body = init + _scope(b, "s", child, funcs)
                out = "s"
            cls = b.mask.type.split(" (")[0]
            help_text = textwrap.wrap(
                f"{child.upper()} Parameters of the block {b.name} ({cls} of "
                "motulator) from the arguments of its class.",
                84,
                subsequent_indent="   ",
            )
            signature = _wrap_call(f"function {out} = {child}(", ", ".join(variables))
            funcs[k] = (
                signature.replace(");\n", ")\n")
                + "".join(f"%{line}\n" for line in help_text)
                + f"{body}end\n"
            )
            values = ", ".join(p.value for p in b.mask.params)
            text += _wrap_call(f"{field} = {child}(", values)
        elif isinstance(b, CBlock):
            text += _block_struct(b, field)
        else:
            raise ValueError(f"{b.name}: a subsystem without a mask")
    return text


def _wrap_call(start: str, args: str) -> str:
    """A function call wrapped into lines with continuations."""
    lines = textwrap.wrap(f"{start}{args});", 80, break_long_words=False)
    return " ...\n    ".join(lines) + "\n"


def params_mfunction(block: ControlBlock) -> MFunction:
    """
    MATLAB function of the parameters of the blocks of a control system, e.g.,
    `ctrl_par = fvc_params(vector_ctrl, speed_ctrl, pwm)`.

    The arguments are those of the top-level class of motulator as structs (the
    variables of the mask of the control-system block), and the output is a struct
    with a struct of the parameters for each block, nested as the blocks (e.g.,
    `ctrl_par.FluxVectorController.ReferenceGenerator`). The function is generated from
    the masks of the netlist: each block with a mask is a local function of the
    arguments of its class, which runs the initialization commands of the mask and
    collects the parameters of the block (or of the blocks in it), an empty
    parameter ([], None in motulator) being NaN, since a MATLAB Function block does
    not accept empty parameters, and checks them.
    """
    name = params_name(block)
    args = [m.variable for m in block.mask_params]
    if set(args) & set(RESERVED):
        raise ValueError(f"{block.name}: reserved variables in {args}")
    funcs: list[str] = []
    scope = _scope(block.netlist(), "ctrl_par", "", funcs)
    period = "ctrl_par = struct();\n% Sampling period\nctrl_par.T_s = T_s;\n"
    body = _mask_code(block.mask_init) + scope.replace(
        "ctrl_par = struct();\n", period, 1
    )
    help_text = textwrap.wrap(
        f"CTRL_PAR = {name.upper()}({', '.join(a.upper() for a in args)}) gives the "
        "struct CTRL_PAR with a struct of the parameters for each block of the control "
        "system, nested as the blocks, and the sampling period T_s, from the "
        "arguments of the top-level class of "
        "motulator as structs (see the method params of the MATLAB classes of "
        "motulator). The MATLAB Function blocks of the control system refer to the "
        "variable ctrl_par of the model, e.g., to ctrl_par.PWM.compute_output. An "
        "empty parameter ([], None in motulator) is NaN in the structs of the blocks. "
        "Generated by motulator_export from the masks of the blocks of the control "
        "system.",
        80,
    )
    main = (
        f"function ctrl_par = {name}({', '.join(args)})\n"
        f"%{name.upper()} Parameters of the blocks of the control system\n"
        f"%   {block.name}.\n"
        + "".join(f"%   {line}\n" for line in help_text)
        + f"{body}end\n"
    )
    return MFunction(name, "\n".join([main, *funcs, NUMERIC_FIELDS]), [], [], None)
