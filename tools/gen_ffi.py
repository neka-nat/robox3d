"""Auto-generate the cffi cdef definitions from box3d's public headers.

Usage:
    uv run python tools/gen_ffi.py

Preprocesses external/box3d/include/box3d/box3d.h with gcc -E, strips syntax that
cffi (pycparser) cannot parse, and writes src/robox3d/_ffi/_cdef.py.

Re-run when box3d's API changes, then review the diff and commit it.
"""

from __future__ import annotations

import re
import subprocess
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
BOX3D_INCLUDE = REPO_ROOT / "external" / "box3d" / "include"
OUTPUT = REPO_ROOT / "src" / "robox3d" / "_ffi" / "_cdef.py"

# cffi understands the stdint type names and bool natively, so replace the
# system headers with empty stubs to prevent their expansion
STUB_HEADERS = [
    "stdbool.h",
    "stdint.h",
    "stddef.h",
    "assert.h",
    "math.h",
    "float.h",
    "string.h",
    "stdlib.h",
    "limits.h",
]


def preprocess(header: Path) -> str:
    with tempfile.TemporaryDirectory() as fake_inc:
        for name in STUB_HEADERS:
            (Path(fake_inc) / name).write_text("")
        result = subprocess.run(
            [
                "gcc",
                "-E",
                "-P",
                "-nostdinc",
                "-I",
                fake_inc,
                "-I",
                str(BOX3D_INCLUDE),
                str(header),
            ],
            capture_output=True,
            text=True,
            check=True,
        )
    return result.stdout


def strip_attributes(text: str) -> str:
    # Remove __attribute__((...)), tracking parenthesis nesting
    out = []
    i = 0
    while True:
        m = re.search(r"__attribute__\s*\(", text[i:])
        if not m:
            out.append(text[i:])
            break
        start = i + m.start()
        out.append(text[i:start])
        j = i + m.end()  # just after the first '('
        depth = 1
        while depth > 0:
            if text[j] == "(":
                depth += 1
            elif text[j] == ")":
                depth -= 1
            j += 1
        i = j
    return "".join(out)


def strip_function_bodies(text: str) -> str:
    """Remove entire inline function definitions.

    A `)` followed by `{` is treated as a function definition; everything from the
    start of the declarator (just after the preceding `;` or `}`) through the end
    of the body is deleted. A `{` for a struct/enum/union follows an identifier,
    so it is not misdetected.
    """
    out = []
    i = 0
    n = len(text)
    last_stmt_end = 0  # end of the previous statement (start of the not-yet-emitted region)
    while i < n:
        c = text[i]
        if c in ";}":
            out.append(text[last_stmt_end : i + 1])
            last_stmt_end = i + 1
            i += 1
            continue
        if c == "{":
            # look at the preceding non-whitespace character
            k = i - 1
            while k >= 0 and text[k].isspace():
                k -= 1
            if k >= 0 and text[k] == ")":
                # function definition: skip the body and drop the whole declaration
                depth = 1
                j = i + 1
                while j < n and depth > 0:
                    if text[j] == "{":
                        depth += 1
                    elif text[j] == "}":
                        depth -= 1
                    j += 1
                last_stmt_end = j
                i = j
                continue
            # struct/enum/union body: advance to the matching '}'
            # (bypass last_stmt_end management since it contains ';' internally)
            depth = 1
            j = i + 1
            while j < n and depth > 0:
                if text[j] == "{":
                    depth += 1
                elif text[j] == "}":
                    depth -= 1
                j += 1
            out.append(text[last_stmt_end:j])
            last_stmt_end = j
            i = j
            continue
        i += 1
    out.append(text[last_stmt_end:])
    return "".join(out)


def dedupe_declarations(text: str) -> str:
    # Upstream headers sometimes contain the exact same function declaration twice
    # (e.g. b3IsValidFloat). Normalize whitespace and drop duplicates after the first
    parts = re.split(r"(;)", text)
    seen: set[str] = set()
    out = []
    i = 0
    while i < len(parts):
        stmt = parts[i]
        sep = parts[i + 1] if i + 1 < len(parts) else ""
        key = " ".join(stmt.split())
        if key and "(" in key and key in seen:
            pass  # skip the duplicate declaration
        else:
            if key:
                seen.add(key)
            out.append(stmt + sep)
        i += 2
    return "".join(out)


def clean(text: str) -> str:
    text = strip_attributes(text)
    text = strip_function_bodies(text)
    text = dedupe_declarations(text)
    # extern "C" blocks shouldn't appear in a C header, but strip them just in case
    text = text.replace('extern "C"', "")
    # collapse consecutive blank lines
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text


def validate(cdef_text: str) -> None:
    from cffi import FFI

    ffi = FFI()
    ffi.cdef(cdef_text)


def main() -> None:
    raw = preprocess(BOX3D_INCLUDE / "box3d" / "box3d.h")
    cdef_text = clean(raw)
    validate(cdef_text)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(
        '"""Auto-generated by tools/gen_ffi.py. Do not edit by hand."""\n\n'
        f"CDEF = r'''\n{cdef_text}\n'''\n"
    )
    n_funcs = len(re.findall(r"\bb3\w+\s*\(", cdef_text))
    print(f"OK: {OUTPUT.relative_to(REPO_ROOT)} ({len(cdef_text)} chars, ~{n_funcs} function refs)")


if __name__ == "__main__":
    main()
