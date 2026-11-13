"""Authoring helpers for SensorProfile messages (runs on the laptop, not on the ESP32).

The firmware only ever sees the RPN program; this module compiles readable infix formulas
into it, and allocates named registers. Registers r0..r7 are the published outputs, so name
the outputs first.
"""
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
GEN = ROOT / "tools" / "_gen"
REG_COUNT = 32


def load_pb():
    """Return the generated aai16_pb2 module, regenerating it when the .proto is newer."""
    proto = ROOT / "protocol" / "aai16.proto"
    out = GEN / "aai16_pb2.py"
    if not out.exists() or out.stat().st_mtime < proto.stat().st_mtime:
        GEN.mkdir(exist_ok=True)
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{proto.parent}",
                        f"--python_out={GEN}", str(proto)], check=True)
    if str(GEN) not in sys.path:
        sys.path.insert(0, str(GEN))
    import aai16_pb2
    return aai16_pb2


class Registers:
    """Name -> register index. Outputs are allocated first so they land on r0..r7."""

    def __init__(self, outputs=()):
        self.names = {}
        for n in outputs:
            self.alloc(n)
        self.output_count = len(outputs)

    def alloc(self, name):
        if name in self.names:
            raise ValueError(f"register name used twice: {name}")
        if len(self.names) >= REG_COUNT:
            raise ValueError(f"out of registers at {name!r} (max {REG_COUNT})")
        self.names[name] = len(self.names)
        return self.names[name]

    def __getitem__(self, name):
        return self.names[name]


_TOKEN = re.compile(r"\s*(?:(\d+\.?\d*(?:[eE][-+]?\d+)?)|([A-Za-z_]\w*)|(.))")
_FUNCS = {"abs": ("CALC_ABS", 1), "sqrt": ("CALC_SQRT", 1), "min": ("CALC_MIN", 2), "max": ("CALC_MAX", 2)}
_BIN = {"+": ("CALC_ADD", 1), "-": ("CALC_SUB", 1), "*": ("CALC_MUL", 2), "/": ("CALC_DIV", 2)}


def compile_rpn(expr, regs):
    """Infix -> list of ('const', v) | ('reg', n) | ('op', name). + - * / unary-, ( ), abs sqrt min max."""
    toks = []
    pos = 0
    while pos < len(expr):
        m = _TOKEN.match(expr, pos)
        if not m or m.end() == pos:
            break
        pos = m.end()
        num, ident, sym = m.groups()
        if num is not None:
            toks.append(("num", float(num)))
        elif ident is not None:
            toks.append(("func", ident) if ident in _FUNCS else ("id", ident))
        elif sym.strip():
            toks.append(("sym", sym))
    out, stack = [], []
    prev_value = False  # was the previous token a value (so '-' is binary)?

    def pop_while(min_prec):
        while stack and stack[-1][0] == "op" and stack[-1][2] >= min_prec:
            out.append(("op", stack.pop()[1]))

    for kind, val in toks:
        if kind == "num":
            out.append(("const", val)); prev_value = True
        elif kind == "id":
            if val not in regs.names:
                raise ValueError(f"unknown register {val!r} in {expr!r}")
            out.append(("reg", regs[val])); prev_value = True
        elif kind == "func":
            stack.append(("func", val)); prev_value = False
        elif val in _BIN and prev_value:
            op, prec = _BIN[val]
            pop_while(prec)
            stack.append(("op", op, prec)); prev_value = False
        elif val == "-":  # unary minus binds tighter than * /
            stack.append(("op", "CALC_NEG", 3)); prev_value = False
        elif val == "(":
            stack.append(("(",)); prev_value = False
        elif val == ",":
            pop_while(0)
            prev_value = False
        elif val == ")":
            pop_while(0)
            if not stack:
                raise ValueError(f"unbalanced ')' in {expr!r}")
            stack.pop()
            if stack and stack[-1][0] == "func":
                out.append(("op", _FUNCS[stack.pop()[1]][0]))
            prev_value = True
        else:
            raise ValueError(f"unexpected {val!r} in {expr!r}")
    while stack:
        top = stack.pop()
        if top[0] == "(":
            raise ValueError(f"unbalanced '(' in {expr!r}")
        out.append(("op", top[1]))
    return out


def add_calc(pb, profile, regs, dest, expr):
    """Append one calc step `dest = expr`; `dest` is allocated if it is a new name."""
    rpn = compile_rpn(expr, regs)
    d = regs.names[dest] if dest in regs.names else regs.alloc(dest)
    step = profile.calc.add()
    step.dest_reg, step.token_count = d, len(rpn)
    for kind, v in rpn:
        t = profile.program.add()
        if kind == "const":
            t.op, t.value = pb.CALC_PUSH_CONST, v
        elif kind == "reg":
            t.op, t.reg = pb.CALC_PUSH_REG, v
        else:
            t.op = getattr(pb, v)


def add_op(profile, **kw):
    op = profile.ops.add()
    for k, v in kw.items():
        setattr(op, k, v)
    return op


def add_init_op(profile, **kw):
    op = profile.init_ops.add()
    for k, v in kw.items():
        setattr(op, k, v)
    return op


def add_decode(profile, regs, dest, **kw):
    """Add a DecodeField writing register `dest` (allocated if new)."""
    d = profile.decode.add()
    d.dest_reg = regs.names[dest] if dest in regs.names else regs.alloc(dest)
    for k, v in kw.items():
        setattr(d, k, v)
    return d
