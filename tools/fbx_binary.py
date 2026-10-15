#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Converts an ASCII FBX file to the binary form (mani-0018), as
# exporters write it, for the fixtures: independent of the library, no
# code shared. Records are 13 bytes before 7500 and 25 from it, each
# child list ended by a null record. Values are typed as real files type
# them: a number with a point or an exponent is D, another integer I (L
# when it needs 64 bits, for times and for 7.x object ids and connection
# ends; in a 6.x Key list every integer is a time), a one-letter word C. Properties70 and Properties60
# entries type their values by the entry's type name. A 7.x `*N { a: }`
# array becomes a typed array (by its node's name), deflated when it
# holds 64 bytes or more; 6.x files keep plain value lists. A string
# "Class::Name" is stored as "Name\x00\x01Class".
#
# usage: fbx_binary.py <in.fbx> <out.fbx> [--version N] [--big-endian]

import argparse
import re
import struct
import sys
import zlib

INTEGER_TYPES = {"int", "Integer", "enum", "Enum", "bool", "Bool", "Visibility Inheritance",
                 "ULongLong", "charptr"}
TIME_TYPES = {"KTime", "KTimeSpan"}
ARRAY_TYPES = {"KeyTime": "l", "KeyValueFloat": "f", "KeyAttrFlags": "i",
               "KeyAttrDataFloat": "f", "KeyAttrRefCount": "i", "PolygonVertexIndex": "i",
               "Edges": "i", "Indexes": "i", "Materials": "i", "UVIndex": "i",
               "NormalsIndex": "i"}
TOKEN = re.compile(r'\s*("(?:[^"\\]|\\.)*"|\*\d+|[-+]?[0-9][0-9.eE+-]*|[-+]?\.[0-9][0-9eE+-]*'
                   r'|[A-Za-z_][A-Za-z0-9_|]*|,|\{|\})')


class Node:
    def __init__(self, name):
        self.name = name
        self.values = []
        self.children = []
        self.array = None


def tokens(line):
    at = 0
    out = []
    while at < len(line):
        m = TOKEN.match(line, at)
        if m is None:
            if line[at:].strip() == "":
                break
            raise ValueError("cannot read: " + line[at:])
        out.append(m.group(1))
        at = m.end()
    return out


def parse(text):
    root = Node("")
    stack = [root]
    pending = None
    for raw in text.split("\n"):
        line = raw.split(";", 1)[0] if not raw.lstrip().startswith('"') else raw
        stripped = line.strip()
        if stripped == "":
            continue
        if stripped == "}":
            stack.pop()
            continue
        m = re.match(r"\s*([A-Za-z_][A-Za-z0-9_-]*):(.*)$", line)
        if m is None:
            # A continuation of the previous node's values.
            parts = [t for t in tokens(stripped) if t not in (",",)]
            target = pending if pending is not None else stack[-1]
            if parts and parts[-1] == "{":
                parts = parts[:-1]
            target.values.extend(parts)
            continue
        name, rest = m.group(1), m.group(2)
        node = Node(name)
        parts = [t for t in tokens(rest) if t != ","]
        opens = bool(parts) and parts[-1] == "{"
        if opens:
            parts = parts[:-1]
        if parts and parts[0].startswith("*"):
            node.array = int(parts[0][1:])
            parts = parts[1:]
        node.values = parts
        stack[-1].children.append(node)
        pending = node
        if opens:
            stack.append(node)
            pending = None
    # A 7.x array's values are its child a:'s.
    def fold(node):
        for c in node.children:
            fold(c)
        if node.array is not None:
            a = [c for c in node.children if c.name == "a"]
            node.values = a[0].values if a else []
            node.children = []
    fold(root)
    return root


def unquote(token):
    body = token[1:-1]
    m = re.fullmatch(r"([A-Za-z][A-Za-z0-9]*)::(.*)", body, re.S)
    data = body.encode("utf-8")
    if m is not None:
        data = m.group(2).encode("utf-8") + b"\x00\x01" + m.group(1).encode("utf-8")
    return data


def is_float(token):
    return any(c in token for c in ".eE") and not token.startswith('"')


class Writer:
    def __init__(self, version, big):
        self.version = version
        self.big = big
        self.wide = version >= 7500
        self.e = ">" if big else "<"

    def pack(self, fmt, *values):
        return struct.pack(self.e + fmt, *values)

    def scalar(self, token, kind):
        if token.startswith('"'):
            data = unquote(token)
            return b"S" + self.pack("I", len(data)) + data
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_|]*", token):
            if len(token) == 1:
                return b"C" + bytes([ord(token)])
            data = token.encode()
            return b"S" + self.pack("I", len(data)) + data
        if kind == "D" or is_float(token):
            return b"D" + self.pack("d", float(token))
        value = int(token)
        if kind == "L" or not -2**31 <= value < 2**31:
            return b"L" + self.pack("q", value)
        return b"I" + self.pack("i", value)

    def array(self, node):
        code = ARRAY_TYPES.get(node.name, "d")
        fmt = {"l": "q", "f": "f", "i": "i", "d": "d"}[code]
        values = [float(v) if code in "fd" else int(v) for v in node.values]
        raw = b"".join(self.pack(fmt, v) for v in values)
        encoding = 1 if len(raw) >= 64 else 0
        body = zlib.compress(raw) if encoding else raw
        return code.encode() + self.pack("III", len(values), encoding, len(body)) + body

    def kinds(self, node, parent, grand):
        n = len(node.values)
        kinds = [None] * n
        if node.name in ("P", "Property") and n >= 3:
            type_name = unquote(node.values[1]).decode("utf-8", "replace")
            first = 4 if node.name == "P" else 3
            for k in range(first, n):
                kinds[k] = ("L" if type_name in TIME_TYPES else
                            "I" if type_name in INTEGER_TYPES else "D")
        elif parent == "Objects" and self.version >= 7000 and n > 0:
            kinds[0] = "L"
        elif node.name in ("C", "Connect") and self.version >= 7000:
            for k in range(1, n):
                kinds[k] = "L"
        elif node.name == "Key" and self.version < 7000:
            # A 6.x key list: its integers are times (L), its other
            # numbers values, slopes and weights (D).
            kinds = ["L" if re.fullmatch(r"[-+]?[0-9]+", v) else "D" for v in node.values]
        elif node.name in ("LocalTime", "ReferenceTime", "KeyTime"):
            kinds = ["L"] * n
        return kinds

    def null(self):
        return b"\x00" * (25 if self.wide else 13)

    def tree(self, node, parent, grand):
        kinds = None
        if node.array is not None and self.version >= 7000:
            props = [self.array(node)]
        else:
            kinds = self.kinds(node, parent, grand)
            props = [self.scalar(v, kinds[k]) for k, v in enumerate(node.values)]
        name = node.name.encode()
        head = (3 * 8 if self.wide else 3 * 4) + 1 + len(name)
        kids = [self.tree(c, node.name, parent) for c in node.children]
        return (head, name, len(props), b"".join(props), kids)

    def file(self, root):
        out = bytearray(b"Kaydara FBX Binary  \x00\x1a")
        out += bytes([1 if self.big else 0])
        out += self.pack("I", self.version)
        for child in root.children:
            out += self.emit_tree(self.tree(child, "", ""), len(out))
        out += self.null()
        return bytes(out)

    def emit_tree(self, record, at):
        head, name, count, body, kids = record
        cursor = at + head + len(body)
        child_bytes = bytearray()
        for k in kids:
            b = self.emit_tree(k, cursor)
            child_bytes += b
            cursor += len(b)
        if kids:
            child_bytes += self.null()
        end = at + head + len(body) + len(child_bytes)
        fmt = "QQQ" if self.wide else "III"
        return self.pack(fmt, end, count, len(body)) + bytes([len(name)]) + name + body + \
            bytes(child_bytes)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("source")
    p.add_argument("target")
    p.add_argument("--version", type=int, default=None)
    p.add_argument("--big-endian", action="store_true")
    a = p.parse_args()
    text = open(a.source, encoding="utf-8").read()
    root = parse(text)
    version = a.version
    if version is None:
        m = re.search(r"FBXVersion:\s*(\d+)", text)
        version = int(m.group(1)) if m else 7400
    data = Writer(version, a.big_endian).file(root)
    open(a.target, "wb").write(data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
