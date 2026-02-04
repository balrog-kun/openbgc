# vim: set ts=4 sw=4 sts=4 et :
from construct import *
import re

QUALIFIER_RE = re.compile(r"\b(const|volatile|restrict)\b\s*")
ARRAY_RE = re.compile(r"(.*?)(\[[0-9]+\])+$")
ARRAY_DIM_RE = re.compile(r"\[([0-9]+)\]")
ENUM_RE = re.compile(r"enum\s*(?::\s*([^{]+))?\s*\{([^}]+)\}")

def strip_qualifiers(s: str) -> str:
    return QUALIFIER_RE.sub("", s).strip()

def parse_enum(enum_str, size, endian):
    """
    Handles strings like:
    enum : unsigned char {A, B, C}
    enum foo {X=1, Y=2}
    """
    m = ENUM_RE.search(enum_str)
    if not m:
        raise ValueError(f"Invalid enum: {enum_str}")

    underlying = m.group(1) or "int"
    body = m.group(2)

    signed = "unsigned" not in underlying.split()
    base = BytesInteger(size, signed=signed, swapped=endian == "little")

    names = []
    values = {}

    cur = 0
    for item in body.split(","):
        item = item.strip()
        if "=" in item:
            name, val = item.split("=", 1)
            cur = int(val.strip(), 0)
        else:
            name = item
        values[name.strip()] = cur
        cur += 1

    return Enum(base, **values)

def ctype_to_construct(
    ctype: str,
    sizeof: int,
    *,
    endian="little"
):
    """
    Convert C type string + sizeof to Construct type.
    """
    # Arrays & strings
    m = ARRAY_RE.fullmatch(ctype)
    if m:
        base = m.group(1).strip()
        dims = [int(x, 0) for x in ARRAY_DIM_RE.findall(ctype)]
        words = base.split()
        if words and words[-1] == "char":
            if len(dims) == 1:
                if sizeof != dims[0]:
                    raise Exception("String size mismatch")
                return PaddedString(dims[0], 'utf8')
            base = base + '[' + str(dims[0]) + ']'
            dims = dims[1:]
        subcon = ctype_to_construct(base, sizeof, endian=endian)
        for d in reversed(dims):
            subcon = Array(d, subcon)
        return subcon

    words = ctype.split()

    # Enum
    if words and words[0] == "enum":
        return parse_enum(ctype, sizeof, endian)

    # Bool
    if ctype in ("_Bool", "bool"):
        return Flag

    # Float
    if ctype == "float":
        return Float32l if endian == "little" else Float32b
    if ctype == "double":
        return Float64l if endian == "little" else Float64b

    # Fixed-width ints
    m = re.fullmatch(r"(u)?int(\d+)_t", ctype)
    if m:
        unsigned, bits = m.groups()
        if sizeof != int(bits) // 8:
            raise Exception("Int size mismatch")
        return BytesInteger(sizeof, signed=not unsigned, swapped=endian == "little")

    # Text char
    #if words and words[-1] == "char":
    #    for q in words[:-1]:
    #        if q not in ["signed", "unsigned"]:
    #            raise Exception("Unknown qualifier " + q)
    #    return PaddedString(1, "ascii")

    # Standard ints
    if words and words[-1] in ["char", "short", "int", "long", "signed", "unsigned"]:
        for q in words[:-1]:
            if q not in ["short", "long", "signed", "unsigned"]:
                raise Exception("Unknown qualifier " + q)
        signed = "unsigned" not in words
        return BytesInteger(sizeof, signed=signed, swapped=endian == "little")

    raise Exception("Unknown type " + ctype)

def c_expr_to_param_name(expr: str) -> str:
    name = ''
    while expr:
        if expr[:2] in ['->', '.']:
            name += '.'
            expr = expr[2:]
        elif expr[:2] in '__':
            expr = expr[2:]
        elif expr[0] in '_':
            name += '-'
            expr = expr[1:]
        elif expr[0] == '[':
            idx, expr = expr[1:].split(']', 1)
            name += '.' + idx
        else:
            name += expr[:1]
            expr = expr[1:]
    return name.lower()
