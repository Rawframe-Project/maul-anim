#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# SimpleSkin (CC0) with a texture seam, for maul-anim's
# Draco skin test: a TEXCOORD_0 attribute added and vertex 4 split in
# two, the copy (vertex 10) with its own coordinate taking the triangles
# that use vertex 4 last; positions, joints and weights unchanged, so
# Draco keeps eleven points over ten values. Writes an embedded glTF.
#
# usage: draco_skin_seam.py <SimpleSkinEmbedded.gltf> <out.gltf>
# Test data only; the second step of test/data/gltf/SimpleSkinSeam-draco.glb.

import base64
import json
import struct
import sys


def data(gltf, buffer):
    uri = gltf["buffers"][buffer]["uri"]
    return base64.b64decode(uri.split(",", 1)[1])


def view_bytes(gltf, view):
    v = gltf["bufferViews"][view]
    b = data(gltf, v["buffer"])
    return b[v.get("byteOffset", 0):v.get("byteOffset", 0) + v["byteLength"]]


def main():
    gltf = json.load(open(sys.argv[1]))
    a = gltf["accessors"]
    indices = list(struct.unpack("<24H", view_bytes(gltf, 0)))
    positions = view_bytes(gltf, 1)
    skin = view_bytes(gltf, 2)
    # Joints at a 16-byte stride, weights after them: both made tight.
    joints = b"".join(skin[16 * i:16 * i + 8] for i in range(10))
    weights = skin[160:320]
    uses = [i for i, x in enumerate(indices) if x == 4]
    for i in uses[len(uses) // 2:]:
        indices[i] = 10
    positions += positions[48:60]
    joints += joints[32:40]
    weights += weights[64:80]
    uv = b"".join(struct.pack("<2f", (v % 2) * 0.5, v / 10.0) for v in range(10))
    uv += struct.pack("<2f", 0.9, 0.95)
    geometry = struct.pack("<24H", *indices) + positions + uv + joints + weights
    offsets = [0, 48, 48 + 132, 48 + 132 + 88]
    gltf["buffers"][0] = {
        "byteLength": len(geometry),
        "uri": "data:application/octet-stream;base64," + base64.b64encode(geometry).decode(),
    }
    gltf["bufferViews"][0] = {"buffer": 0, "byteLength": 48, "target": 34963}
    gltf["bufferViews"][1] = {"buffer": 0, "byteOffset": 48, "byteLength": 132, "target": 34962}
    gltf["bufferViews"][2] = {"buffer": 0, "byteOffset": offsets[3], "byteLength": 88}
    # Buffer 1 held the joints and weights; the later buffers move down.
    del gltf["buffers"][1]
    for v in gltf["bufferViews"]:
        v["buffer"] -= 1 if v["buffer"] > 1 else 0
    views = len(gltf["bufferViews"])
    gltf["bufferViews"].append({"buffer": 0, "byteOffset": offsets[3] + 88, "byteLength": 176})
    gltf["bufferViews"].append({"buffer": 0, "byteOffset": offsets[2], "byteLength": 88})
    a[1]["count"] = 11
    a[2] = {"bufferView": 2, "componentType": 5123, "count": 11, "type": "VEC4"}
    a[3] = {"bufferView": views, "componentType": 5126, "count": 11, "type": "VEC4"}
    a.append({"bufferView": views + 1, "componentType": 5126, "count": 11, "type": "VEC2"})
    gltf["meshes"][0]["primitives"][0]["attributes"]["TEXCOORD_0"] = len(a) - 1
    json.dump(gltf, open(sys.argv[2], "w"), indent=2)


main()
