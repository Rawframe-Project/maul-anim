#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The runtime's code size by part (mani-0036): the text of each object of
# a runtime-only archive (built with MAUL_ANIM_OFFLINE off), read with
# size(1), grouped into the parts tools/size-budget.txt names. A part
# lists its modules; "core:" takes its modules and every module they use
# by tools/modules.txt that no other part lists. A module no part takes
# is reported as unplaced and fails --check, so a new module is placed
# when it lands. With --check LEVEL, each part's text is held against
# its ceiling for that level (O2 or O3, as the archive was built); a
# ceiling only ever comes down.
#
# The runtime allocates nothing (mani-0001): the archive may name no
# allocator (nm(1)'s undefined symbols; NM names another nm, such as
# llvm-nm for a Windows library), checked on every run. The names are
# every C library and Windows CRT function that returns memory to free,
# read as ELF, COFF (with or without __imp_) and Mach-O (one leading
# underscore) spell them. --allocators runs that check alone, for a
# platform without size(1).
#
# usage: size_report.py <libmaul-anim.a> [--check O2|O3]
#        size_report.py <archive> --allocators

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_modules():
    """Each module and the modules it may use."""
    uses = {}
    with open(os.path.join(ROOT, "tools", "modules.txt"), encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if ":" in line:
                name, rest = line.split(":", 1)
                uses[name.strip()] = rest.split()
    return uses


def read_budget():
    """The parts in order with their modules, and the ceilings by part and
    level."""
    parts = []
    ceilings = {}
    with open(os.path.join(ROOT, "tools", "size-budget.txt"), encoding="utf-8") as f:
        for number, line in enumerate(f, 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            if line.startswith("ceiling "):
                words = line.split()
                if len(words) != 4:
                    sys.exit(f"size-budget.txt:{number}: ceiling PART LEVEL BYTES")
                ceilings[(words[1], words[2])] = int(words[3])
            elif ":" in line:
                name, rest = line.split(":", 1)
                parts.append((name.strip(), rest.split()))
            else:
                sys.exit(f"size-budget.txt:{number}: neither a part nor a ceiling")
    return parts, ceilings


def read_sizes(archive):
    """Each object's text in bytes, by module name."""
    out = subprocess.run(["size", archive], capture_output=True, text=True, check=True).stdout
    sizes = {}
    for line in out.splitlines()[1:]:
        words = line.split()
        if len(words) >= 6:
            name = os.path.basename(words[5])
            for suffix in (".c.o", ".c.obj", ".o"):
                if name.endswith(suffix):
                    name = name[: -len(suffix)]
                    break
            sizes[name] = sizes.get(name, 0) + int(words[0])
    return sizes


ALLOCATORS = frozenset((
    # C and POSIX
    "malloc", "calloc", "realloc", "reallocarray", "free", "aligned_alloc",
    "posix_memalign", "memalign", "valloc", "pvalloc", "strdup", "strndup",
    "wcsdup", "asprintf", "vasprintf", "getline", "getdelim", "open_memstream",
    # The Windows CRT
    "_aligned_malloc", "_aligned_realloc", "_aligned_recalloc", "_aligned_free",
    "_aligned_offset_malloc", "_recalloc", "_expand", "_malloca", "_freea",
    "_strdup", "_wcsdup", "_mbsdup",
))


def spellings(symbol):
    """A symbol and the names it may stand for: without COFF's import
    prefix, and without Mach-O's leading underscore."""
    names = {symbol}
    if symbol.startswith("__imp_"):
        names.add(symbol[len("__imp_"):])
    names.update(n[1:] for n in list(names) if n.startswith("_"))
    return names


def allocators(archive):
    """The allocator functions the archive calls."""
    nm = os.environ.get("NM", "nm")
    out = subprocess.run([nm, "-u", archive], capture_output=True, text=True, check=True).stdout
    called = set()
    for line in out.splitlines():
        words = line.split()
        if len(words) == 2 and words[0] == "U":
            called.update(spellings(words[1]).intersection(ALLOCATORS))
    return sorted(called)


def place(parts, uses, sizes):
    """Each part's modules present in the archive; the core takes what its
    modules use that no other part lists."""
    listed = {m for _, modules in parts for m in modules}
    placed = {}
    for name, modules in parts:
        chosen = set(modules)
        if name == "core":
            stack = list(modules)
            while stack:
                m = stack.pop()
                for u in uses.get(m, []):
                    if u not in chosen and u not in listed:
                        chosen.add(u)
                        stack.append(u)
        placed[name] = sorted(m for m in chosen if m in sizes)
    return placed


def main():
    args = sys.argv[1:]
    level = None
    if len(args) == 2 and args[1] == "--allocators":
        called = allocators(args[0])
        print("allocation: " + (", ".join(called) if called else "none"))
        return 1 if called else 0
    if len(args) == 3 and args[1] == "--check" and args[2] in ("O2", "O3"):
        level = args[2]
    elif len(args) != 1:
        sys.exit("usage: size_report.py <libmaul-anim.a> [--check O2|O3]\n"
                 "       size_report.py <archive> --allocators")
    uses = read_modules()
    parts, ceilings = read_budget()
    sizes = read_sizes(args[0])
    placed = place(parts, uses, sizes)
    failed = False
    total = 0
    for name, _ in parts:
        text = sum(sizes[m] for m in placed[name])
        total += text
        line = f"{name:<12} {text:>8}"
        ceiling = ceilings.get((name, level)) if level else None
        if ceiling is not None:
            line += f"  ceiling {ceiling}"
            if text > ceiling:
                line += "  OVER"
                failed = True
        elif level:
            line += "  no ceiling"
            failed = True
        print(line)
    taken = {m for modules in placed.values() for m in modules}
    unplaced = sorted(m for m in sizes if m not in taken)
    for m in unplaced:
        print(f"unplaced     {sizes[m]:>8}  {m}")
        failed = failed or level is not None
    print(f"{'total':<12} {total + sum(sizes[m] for m in unplaced):>8}")
    called = allocators(args[0])
    print("allocation: " + (", ".join(called) if called else "none"))
    return 1 if failed or called else 0


if __name__ == "__main__":
    sys.exit(main())
