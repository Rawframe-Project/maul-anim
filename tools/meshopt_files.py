#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Decodes every buffer view of a glTF file that
# EXT_meshopt_compression or KHR_meshopt_compression compresses with
# meshoptimizer's own decoder (and filters), and prints each view's
# mode, filter, stream version and the FNV-1a 64 of all decoded views in
# view order: the value test_gltf_meshopt_files.c pins for the
# library's decoding of the same file.
#
# usage: meshopt_files.py <libmeshoptimizer.so> <file.glb|file.gltf>...
# Build the library from meshoptimizer's sources (f0a684f1):
#   g++ -O2 -shared -fPIC src/*.cpp -o libmeshoptimizer.so
# Prints expected values only; never part of the library.

import ctypes
import json
import os
import struct
import sys


def load(path):
    raw = open(path, "rb").read()
    base = os.path.dirname(path)
    if path.endswith(".glb"):
        length = struct.unpack_from("<I", raw, 12)[0]
        doc = json.loads(raw[20:20 + length])
        chunk = 20 + length
        binary = raw[chunk + 8:chunk + 8 + struct.unpack_from("<I", raw, chunk)[0]]
    else:
        doc = json.loads(raw)
        binary = None
    buffers = []
    for i, b in enumerate(doc.get("buffers", [])):
        uri = b.get("uri")
        if uri is None:
            buffers.append(binary if i == 0 else None)
        elif uri.startswith("data:"):
            import base64
            buffers.append(base64.b64decode(uri.split(",", 1)[1]))
        else:
            buffers.append(open(os.path.join(base, uri), "rb").read())
    return doc, buffers


def fnv(h, data):
    for byte in data:
        h = ((h ^ byte) * 0x100000001b3) & 0xffffffffffffffff
    return h


def main():
    lib = ctypes.CDLL(sys.argv[1])
    filters = {"OCTAHEDRAL": lib.meshopt_decodeFilterOct,
               "QUATERNION": lib.meshopt_decodeFilterQuat,
               "EXPONENTIAL": lib.meshopt_decodeFilterExp,
               "COLOR": getattr(lib, "meshopt_decodeFilterColor", None)}
    for path in sys.argv[2:]:
        doc, buffers = load(path)
        h = 0xcbf29ce484222325
        for i, view in enumerate(doc.get("bufferViews", [])):
            ext = view.get("extensions", {})
            e = ext.get("KHR_meshopt_compression", ext.get("EXT_meshopt_compression"))
            if e is None:
                continue
            src = buffers[e["buffer"]][e.get("byteOffset", 0):][:e["byteLength"]]
            count, stride, mode = e["count"], e["byteStride"], e["mode"]
            out = ctypes.create_string_buffer(count * stride)
            if mode == "ATTRIBUTES":
                rc = lib.meshopt_decodeVertexBuffer(out, count, stride, src, len(src))
            elif mode == "TRIANGLES":
                rc = lib.meshopt_decodeIndexBuffer(out, count, stride, src, len(src))
            else:
                rc = lib.meshopt_decodeIndexSequence(out, count, stride, src, len(src))
            assert rc == 0, (path, i, rc)
            f = e.get("filter", "NONE")
            if f != "NONE":
                filters[f](out, count, stride)
            h = fnv(h, out.raw)
            print("%s view %d: %s %s stride %d count %d header 0x%02x" %
                  (os.path.basename(path), i, mode, f, stride, count, src[0]))
        print("%s: FNV-1a 64 of the decoded views 0x%016xull" % (os.path.basename(path), h))


main()
