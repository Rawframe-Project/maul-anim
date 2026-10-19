#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The glTF fixtures' oracle (mani-0010): reads each sample model in
# test/data/gltf with its own small reader, evaluates every animation
# channel as glTF's Appendix C defines it, in binary64, at the sample
# times the clip builder uses (rate 30, the duration the animation's last
# key), and writes the values for test_gltf_fixtures.c. Independent of
# the library: no code is shared. A file gltfpack compressed with meshopt
# is read from its uncompressed fallback buffer, which the library never
# reads, so its tracks check the library's decoding.
#
# usage: gen_gltf_expect.py > test/gltf_fixture_expect.h
#        gen_gltf_expect.py --check   (the header matches)

import base64
import json
import math
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "test", "data", "gltf")
FILES = ["InterpolationTest.glb", "AnimatedMorphCube.glb", "SimpleMorph.gltf",
         "SimpleSkinEmbedded.gltf", "InterpolationTest-meshopt.glb"]
RATE = 30.0
KINDS = {"translation": 0, "rotation": 1, "scale": 2, "weights": 3}


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def load(name):
    path = os.path.join(DATA, name)
    raw = open(path, "rb").read()
    binary = None
    if name.endswith(".glb"):
        length = struct.unpack_from("<I", raw, 12)[0]
        doc = json.loads(raw[20:20 + length])
        bin_length = struct.unpack_from("<I", raw, 20 + length)[0]
        binary = raw[28 + length:28 + length + bin_length]
    else:
        doc = json.loads(raw)
    buffers = []
    for i, b in enumerate(doc.get("buffers", [])):
        uri = b.get("uri")
        if uri is None:
            assert i == 0 and binary is not None, "only a GLB's first buffer has no URI"
            buffers.append(binary)
        elif uri.startswith("data:"):
            buffers.append(base64.b64decode(uri.split(",", 1)[1]))
        else:
            buffers.append(open(os.path.join(os.path.dirname(path), uri), "rb").read())
    return doc, buffers


# Component types: struct format, size, and the divisor of a normalized
# integer (glTF's table: max(c / d, -1) for signed types).
COMPONENTS = {5120: ("b", 1, 127.0), 5121: ("B", 1, 255.0), 5122: ("h", 2, 32767.0),
              5123: ("H", 2, 65535.0), 5126: ("f", 4, None)}


def accessor(doc, buffers, index):
    a = doc["accessors"][index]
    view = doc["bufferViews"][a["bufferView"]]
    width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
    form, size, divisor = COMPONENTS[a["componentType"]]
    assert divisor is None or a.get("normalized", False), "floats or normalized integers"
    stride = view.get("byteStride", size * width)
    offset = view.get("byteOffset", 0) + a.get("byteOffset", 0)
    rows = []
    for k in range(a["count"]):
        row = list(struct.unpack_from("<%d%s" % (width, form), buffers[view["buffer"]],
                                      offset + k * stride))
        rows.append(row if divisor is None else [max(c / divisor, -1.0) for c in row])
    return rows


def normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return [c / n for c in q]


def slerp(a, b, u):
    a, b = normalize(a), normalize(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = [-c for c in b], -dot
    if dot > 1 - 1e-12:
        return normalize([x + (y - x) * u for x, y in zip(a, b)])
    angle = math.acos(min(1.0, dot))
    s = math.sin(angle)
    wa, wb = math.sin((1 - u) * angle) / s, math.sin(u * angle) / s
    return [wa * x + wb * y for x, y in zip(a, b)]


def evaluate(times, values, mode, t, rotation):
    """Appendix C: values are key vectors (cubic: in, value, out each)."""
    n = len(times)
    part = 1 if mode == "CUBICSPLINE" else 0
    stride = 3 if mode == "CUBICSPLINE" else 1

    def value(k, p=None):
        return values[k * stride + (part if p is None else p)]

    if t <= times[0] or n == 1:
        out = value(0)
    elif t >= times[-1]:
        out = value(n - 1)
    else:
        k = max(i for i in range(n) if times[i] <= t)
        t0, t1 = times[k], times[k + 1]
        span = t1 - t0
        u = (t - t0) / span
        if mode == "STEP" or t == t0:
            out = value(k)
        elif mode == "LINEAR" and rotation:
            out = slerp(value(k), value(k + 1), u)
        elif mode == "LINEAR":
            out = [x + (y - x) * u for x, y in zip(value(k), value(k + 1))]
        else:
            h00, h10 = 2 * u ** 3 - 3 * u ** 2 + 1, u ** 3 - 2 * u ** 2 + u
            h01, h11 = -2 * u ** 3 + 3 * u ** 2, u ** 3 - u ** 2
            out = [h00 * p0 + h10 * span * m0 + h01 * p1 + h11 * span * m1
                   for p0, m0, p1, m1 in zip(value(k), value(k, 2), value(k + 1), value(k + 1, 0))]
    return normalize(out) if rotation else list(out)


def sample_count(duration):
    """The clip builder's count: binary32 product, whole frames snapped."""
    if duration == 0:
        return 1
    product = f32(duration * RATE)
    nearest = math.floor(product)
    if product - nearest >= 0.5:
        nearest += 1
    intervals = nearest if abs(product - nearest) <= nearest * 2.0 ** -22 else math.ceil(product)
    return int(intervals) + 1


def sample_time(k, n, duration):
    return duration if k + 1 == n and k > 0 else k / RATE


def node_names(doc):
    """Bone names fixed per document: a node's name, or with "#" and its
    index when it has none or an earlier node has the same."""
    seen = set()
    names = []
    for i, node in enumerate(doc.get("nodes", [])):
        name = node.get("name", "")
        names.append(name + "#" + str(i) if name == "" or name in seen else name)
        if name != "":
            seen.add(name)
    return names


def weight_names(doc, node, count, names):
    mesh = doc["meshes"][doc["nodes"][node]["mesh"]]
    targets = mesh.get("extras", {}).get("targetNames")
    if not (isinstance(targets, list) and len(targets) == count
            and all(isinstance(t, str) and t for t in targets)):
        targets = [str(i) for i in range(count)]
    return "|".join(names[node] + "/" + t for t in targets)


def tracks():
    rows = []
    for name in FILES:
        doc, buffers = load(name)
        names = node_names(doc)
        for a, animation in enumerate(doc.get("animations", [])):
            duration = 0.0
            for c in animation["channels"]:
                times = accessor(doc, buffers, animation["samplers"][c["sampler"]]["input"])
                duration = max(duration, times[-1][0])
            n = sample_count(duration)
            for c in animation["channels"]:
                sampler = animation["samplers"][c["sampler"]]
                path = c["target"]["path"]
                node = c["target"]["node"]
                times = [t[0] for t in accessor(doc, buffers, sampler["input"])]
                out = accessor(doc, buffers, sampler["output"])
                mode = sampler.get("interpolation", "LINEAR")
                if path == "weights":
                    # Frames of one scalar per target, end to end.
                    per = len(out) // len(times) // (3 if mode == "CUBICSPLINE" else 1)
                    keys = [[out[i * per + j][0] for j in range(per)] for i in range(len(out) // per)]
                else:
                    keys = out
                values = []
                for k in range(n):
                    values += evaluate(times, keys, mode, sample_time(k, n, duration),
                                       path == "rotation")
                width = len(values) // n
                floats = weight_names(doc, node, width, names) if path == "weights" else ""
                rows.append((name, a, node, KINDS[path], width, n, duration, values, floats))
    return rows


def render():
    lines = ["// SPDX-License-Identifier: MIT",
             "// Copyright (c) 2026 Sirac Ozmen",
             "//",
             "// Generated by tools/gen_gltf_expect.py: each fixture channel evaluated",
             "// by glTF's Appendix C in binary64 at the clip's sample times (rate 30).",
             "// Do not edit.",
             "",
             "#include <stdint.h>",
             "",
             "typedef struct FixtureTrack",
             "{",
             "    const char* file;",
             "    uint32_t animation;",
             "    uint32_t node;",
             "    // 0 translation, 1 rotation, 2 scale, 3 weights.",
             "    uint32_t kind;",
             "    uint32_t width;",
             "    uint32_t samples;",
             "    float duration;",
             "    const double* values;",
             "    // Weights: the float tracks' names, joined by '|'.",
             "    const char* names;",
             "} FixtureTrack;",
             "",
             "// clang-format off"]
    rows = tracks()
    for i, row in enumerate(rows):
        values = ", ".join(repr(v) for v in row[7])
        lines.append("static const double kFixtureValues%d[] = {%s};" % (i, values))
    lines.append("static const FixtureTrack kFixtureTracks[] = {")
    for i, (name, a, node, kind, width, n, duration, _, floats) in enumerate(rows):
        lines.append('    {"%s", %d, %d, %d, %d, %d, %sf, kFixtureValues%d, "%s"},'
                     % (name, a, node, kind, width, n, repr(duration), i, floats))
    lines += ["};", "// clang-format on"]
    return "\n".join(lines) + "\n"


def main():
    text = render()
    if len(sys.argv) > 1 and sys.argv[1] == "--check":
        path = os.path.join(ROOT, "test", "gltf_fixture_expect.h")
        if open(path, encoding="utf-8").read() != text:
            print("test/gltf_fixture_expect.h differs from tools/gen_gltf_expect.py")
            sys.exit(1)
        return
    sys.stdout.write(text)


if __name__ == "__main__":
    main()
