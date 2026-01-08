# vim: set ts=4 sw=4 sts=4 et :
import re

QUALIFIER_RE = re.compile(r"\b(const|volatile|restrict)\b\s*")

def strip_qualifiers(s: str) -> str:
    return QUALIFIER_RE.sub("", s).strip()

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
