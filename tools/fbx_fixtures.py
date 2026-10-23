#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Writes the binary forms of the FBX fixtures from their
# ASCII sources in test/data/fbx, by tools/fbx_binary.py: 13-byte records
# (6100, 7400), 25-byte records with deflated arrays (7500), and one
# big-endian file. Run after changing a source, then regenerate the
# expected values:
#
#   fbx_differential --deviations tools/fbx-deviations.txt \
#       --expect test/fbx_fixture_expect.h test/data/fbx/*.fbx
#
# With --compare, a freshly written header is held against the committed
# one: the same files and records, values within 1e-6 of their size (the
# last digits may differ between machines' libm and ufbx's build).
#
# usage: fbx_fixtures.py
#        fbx_fixtures.py --compare FRESH COMMITTED

import os
import re
import subprocess
import sys

# (source, form, version, big-endian)
FORMS = [
    ("layers.fbx", "layers_7400.fbx", 7400, False),
    ("shapes.fbx", "shapes_7500.fbx", 7500, False),
    ("curves.fbx", "curves_7500.fbx", 7500, False),
    ("skins.fbx", "skins_7400_be.fbx", 7400, True),
    ("take.fbx", "take_6100.fbx", 6100, False),
]


RECORD = re.compile(r'^    \{(\d+), "(\w+)", (\d+), (\S+), ("(?:[^"\\]|\\.)*"), (\d+), \{([^}]*)\}\},$')
FILE = re.compile(r'^    ("(?:[^"\\]|\\.)*"),$')


def read_header(path):
    files, records = [], []
    for line in open(path, encoding="utf-8"):
        m = RECORD.match(line)
        if m:
            values = [float(v) for v in m.group(7).split(",")]
            records.append((m.group(1, 2, 3, 4, 5, 6), values))
        elif FILE.match(line):
            files.append(FILE.match(line).group(1))
    return files, records


def near(a, b):
    return abs(a - b) <= 1e-6 * max(1.0, abs(b))


def compare(fresh, committed):
    new_files, new_records = read_header(fresh)
    old_files, old_records = read_header(committed)
    if new_files != old_files or len(new_records) != len(old_records):
        print(f"{committed}: {len(old_files)} files and {len(old_records)} records, "
              f"ufbx now gives {len(new_files)} and {len(new_records)}")
        return 1
    differ = 0
    for (new_key, new_values), (old_key, old_values) in zip(new_records, old_records):
        same = new_key == old_key and len(new_values) == len(old_values) and \
            all(near(a, b) for a, b in zip(new_values, old_values))
        if not same:
            print(f"{committed}: {old_key} {old_values}, ufbx now {new_key} {new_values}")
            differ += 1
    return 1 if differ else 0


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "--compare":
        return compare(sys.argv[2], sys.argv[3])
    tools = os.path.dirname(os.path.abspath(__file__))
    data = os.path.join(os.path.dirname(tools), "test", "data", "fbx")
    for source, form, version, big in FORMS:
        command = [sys.executable, os.path.join(tools, "fbx_binary.py"),
                   os.path.join(data, source), os.path.join(data, form),
                   "--version", str(version)]
        subprocess.run(command + (["--big-endian"] if big else []), check=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
