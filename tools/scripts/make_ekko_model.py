#!/usr/bin/env python3
"""make_ekko_model -- turn Ekko's Onshape export into the Map tab's 3D model.

    python tools/scripts/make_ekko_model.py "../CAD/Assembly 1.glb"

Writes uav_groundstation/uav_groundstation/ekko_model.py (generated; do not
edit it). Needs numpy, nothing else: a laptop tool, run when the CAD changes.

EXPORT FROM ONSHAPE (right-click the top assembly's tab -> Export): GLB,
Resolution Coarse, "Export unique parts as individual files" OFF, "Compress"
OFF, "Export models oriented Y axis up" OFF, Download. The GLB stays OUT of the
repo; only the cut-down model below goes in.

WHAT IT KEEPS. The page draws every triangle 20 times a second with the canvas
alone, so the model is cut to about TARGET_TRIS: parts smaller than MIN_PART_M
(connectors, nuts) are dropped, the rest is merged onto a grid (vertex
clustering) per colour, and each propeller becomes a translucent disc -- how a
spinning prop looks, and what shows tilt best -- RED at the front, BLUE at the
rear. The camera and the GPS are RED too: they are Ekko's front.

THE AXES come from the geometry, not from how the CAD was drawn: the four
motors fix the arm plane, the props say which side of it is up, and the camera
fixes the front (the bisector of the two arms nearest it). Output is Ekko's
body frame, x forward, y right, z down, in millimetres.
"""
import json
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "uav_groundstation", "uav_groundstation",
                   "ekko_model.py")
TARGET_TRIS = 2500
MIN_PART_M = 0.02
PROP_SIDES = 20
#: PAL keys (the page's theme colours) and opacity, by group.
GROUPS = [("dim", 1.0), ("fg", 1.0), ("bRed", 1.0), ("accent", 1.0),
          ("warn", 1.0), ("bRed", 0.5), ("accent", 0.5)]
DIM, FG, FRONT, BLUE, ORANGE, PROP_FRONT, PROP_REAR = range(len(GROUPS))

_COMP = {5120: ("b", 1), 5121: ("B", 1), 5122: ("h", 2), 5123: ("H", 2),
         5125: ("I", 4), 5126: ("f", 4)}
_WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def read_glb(path):
    b = open(path, "rb").read()
    magic, _ver, _len = struct.unpack_from("<4sII", b, 0)
    if magic != b"glTF":
        raise SystemExit("%s is not a GLB file" % path)
    jlen, _ = struct.unpack_from("<I4s", b, 12)
    doc = json.loads(b[20:20 + jlen])
    blen, _ = struct.unpack_from("<I4s", b, 20 + jlen)
    binary = b[28 + jlen:28 + jlen + blen]
    return doc, binary


def accessor(doc, binary, i):
    a = doc["accessors"][i]
    bv = doc["bufferViews"][a["bufferView"]]
    fmt, size = _COMP[a["componentType"]]
    n = _WIDTH[a["type"]]
    stride = bv.get("byteStride") or size * n
    start = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
    arr = np.ndarray(shape=(a["count"], n), dtype=np.dtype("<" + fmt), buffer=binary,
                     offset=start, strides=(stride, size))
    return arr.astype(np.float64 if fmt == "f" else np.int64)


def local_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"], dtype=np.float64).reshape(4, 4).T
    m = np.eye(4)
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    m[:3, :3] = r * np.array(node.get("scale", [1, 1, 1]))
    m[:3, 3] = node.get("translation", [0, 0, 0])
    return m


def parts(doc, binary):
    """[(names on the path, world vertices (n,3), triangles (t,3), base colour,
    node index)] -- one row per PRIMITIVE; a part is every row with its node."""
    out = []

    def walk(i, parent, path):
        node = doc["nodes"][i]
        world = parent @ local_matrix(node)
        path = path + [node.get("name", "")]
        if "mesh" in node:
            for prim in doc["meshes"][node["mesh"]]["primitives"]:
                if prim.get("mode", 4) != 4:
                    continue
                v = accessor(doc, binary, prim["attributes"]["POSITION"])
                v = (world @ np.c_[v, np.ones(len(v))].T).T[:, :3]
                t = (accessor(doc, binary, prim["indices"]).reshape(-1, 3) if "indices" in prim
                     else np.arange(len(v)).reshape(-1, 3))
                mat = doc.get("materials", [{}])[prim.get("material", 0)] if doc.get("materials") else {}
                col = mat.get("pbrMetallicRoughness", {}).get("baseColorFactor", [0.5, 0.5, 0.5, 1])
                out.append((path, v, t, col[:3], i))
        for c in node.get("children", []):
            walk(c, world, path)

    for root in doc["scenes"][doc.get("scene", 0)]["nodes"]:
        walk(root, np.eye(4), [])
    return out


def has(path, *words):
    s = " ".join(path).lower()
    return any(w in s for w in words)


def colour_group(rgb):
    r, g, b = rgb
    hi, lo = max(rgb), min(rgb)
    if hi - lo > 0.35:                                   # saturated
        if b >= r and b >= g:
            return BLUE
        return ORANGE if r > b else DIM
    return FG if (r + g + b) / 3.0 > 0.45 else DIM


def body_frame(ps):
    """The 4x4 that takes the CAD's metres to Ekko's FRD millimetres."""
    def centroid(pred):
        """One centroid per PART (node), from all of its primitives."""
        by_node = {}
        for path, v, _t, _c, node in ps:
            if pred(path):
                by_node.setdefault(node, []).append(v)
        return np.array([np.vstack(vs).mean(axis=0) for vs in by_node.values()])
    motors = centroid(lambda p: has(p[-1:], "motor"))
    props = centroid(lambda p: has(p, "16x5.5"))
    front = centroid(lambda p: has(p, "siyi", "camera", "gps"))
    if len(motors) != 4 or len(front) == 0:
        raise SystemExit("need 4 motors and a camera/GPS to find the axes (found %d motors, "
                         "%d front parts)" % (len(motors), len(front)))
    centre = motors.mean(axis=0)
    _u, _s, vt = np.linalg.svd(motors - centre)
    up = vt[2]
    if len(props) and np.dot(props.mean(axis=0) - centre, up) < 0:
        up = -up
    to_front = front.mean(axis=0) - centre
    to_front -= np.dot(to_front, up) * up
    arms = motors - centre
    arms -= np.outer(arms @ up, up)
    near = arms[np.argsort(arms @ to_front)[-2:]]      # the two arms on the camera's side
    fwd = near.mean(axis=0)
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    rot = np.vstack([fwd, right, -up])                  # rows: x fwd, y right, z down
    m = np.eye(4)
    m[:3, :3] = rot * 1000.0
    m[:3, 3] = -(rot @ centre) * 1000.0
    return m


def cluster(v, t, cell):
    """Vertex clustering: every vertex moves to the mean of its grid cell."""
    keys = np.floor(v / cell).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    nv = np.zeros((len(uniq), 3))
    np.add.at(nv, inv, v)
    nv /= np.bincount(inv, minlength=len(uniq))[:, None]
    nt = inv[t]
    keep = (nt[:, 0] != nt[:, 1]) & (nt[:, 1] != nt[:, 2]) & (nt[:, 0] != nt[:, 2])
    nt = nt[keep]
    if len(nt):
        nt = np.unique(np.sort(nt, axis=1), axis=0)
    return nv, nt


def disc(centre, radius, sides):
    ang = np.linspace(0, 2 * np.pi, sides, endpoint=False)
    rim = np.c_[centre[0] + radius * np.cos(ang), centre[1] + radius * np.sin(ang),
                np.full(sides, centre[2])]
    v = np.vstack([centre, rim])
    t = np.array([[0, 1 + k, 1 + (k + 1) % sides] for k in range(sides)])
    return v, t


def build(path_glb):
    doc, binary = read_glb(path_glb)
    ps = parts(doc, binary)
    frame = body_frame(ps)
    to_body = lambda v: (frame @ np.c_[v, np.ones(len(v))].T).T[:, :3]
    by_group = {g: [] for g in range(len(GROUPS))}
    dropped = kept = 0
    props = {}                       # node -> [vertices]: a prop is 16 pieces in the CAD
    for path, v, t, col, node in ps:
        vb = to_body(v)
        size = np.linalg.norm(vb.max(axis=0) - vb.min(axis=0)) / 1000.0
        if has(path, "16x5.5"):
            props.setdefault(node, []).append(vb)
            continue
        if size < MIN_PART_M:
            dropped += 1
            continue
        g = FRONT if has(path, "siyi", "camera", "gps") else colour_group(col)
        by_group[g].append((vb, t))
        kept += 1

    # ONE disc per prop -- per PART, not per piece: a disc per piece drew 64
    # overlapping discs for 4 props (Chris spotted it, 19 Sep). The disc a
    # spinning blade sweeps: centred on the hub, as wide as the blade is long,
    # at the top of the prop.
    for vs in props.values():
        vb = np.vstack(vs)
        c = vb.mean(axis=0)
        r = float(np.max(np.linalg.norm((vb - c)[:, :2], axis=1)))
        dv, dt = disc(np.array([c[0], c[1], vb[:, 2].min()]), r, PROP_SIDES)
        by_group[PROP_FRONT if c[0] > 0 else PROP_REAR].append((dv, dt))
        kept += 1
    if len(props) != 4:
        raise SystemExit("expected 4 props, found %d" % len(props))

    def merged(items):
        vs, ts, n = [], [], 0
        for v, t in items:
            vs.append(v)
            ts.append(t + n)
            n += len(v)
        return (np.vstack(vs), np.vstack(ts)) if vs else (np.zeros((0, 3)), np.zeros((0, 3), int))

    solid = {g: merged(it) for g, it in by_group.items() if g not in (PROP_FRONT, PROP_REAR)}
    discs = {g: merged(by_group[g]) for g in (PROP_FRONT, PROP_REAR)}
    n_disc = sum(len(t) for _v, t in discs.values())
    lo, hi = 1.0, 60.0                                  # grid cell, mm
    for _ in range(18):
        cell = (lo + hi) / 2
        n = sum(len(cluster(v, t, cell)[1]) for v, t in solid.values() if len(t))
        if n + n_disc > TARGET_TRIS:
            lo = cell
        else:
            hi = cell
    cell = hi
    V, T, G = [], [], []
    base = 0
    for g, (v, t) in list(solid.items()) + list(discs.items()):
        if not len(t):
            continue
        if g in solid:
            v, t = cluster(v, t, cell)
        V.append(v)
        T.append(t + base)
        G.append(np.full(len(t), g))
        base += len(v)
    V = np.vstack(V)
    T = np.vstack(T)
    G = np.concatenate(G)
    return V, T, G, {"parts_kept": kept, "parts_dropped": dropped, "cell_mm": round(cell, 1),
                     "source_triangles": int(sum(len(t) for _p, _v, t, _c, _n in ps))}


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    V, T, G, info = build(sys.argv[1])
    Vq = np.round(V).astype("<i2")
    blob = (struct.pack("<HH", len(Vq), len(T)) + Vq.tobytes() + T.astype("<u2").tobytes()
            + G.astype("u1").tobytes())
    ext = V.max(axis=0) - V.min(axis=0)
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write('"""ekko_model -- Ekko\'s shape for the Map tab\'s 3D view.\n\n'
                "GENERATED by tools/scripts/make_ekko_model.py from %s; do not edit.\n"
                "%d triangles (from %d), %d parts kept, %d small ones dropped, grid %.1f mm.\n"
                "Body frame x forward, y right, z down, millimetres; extent %.0f x %.0f x %.0f.\n"
                "DATA is hex (not base64, which sooner or later spells \"cdn\" and trips the\n"
                "page's no-external-fetch check): uint16 vertex count, uint16 triangle\n"
                "count, int16 x/y/z per vertex, uint16 x3 per triangle, uint8 group per\n"
                "triangle. GROUPS: (PAL colour key, opacity).\n\"\"\"\n\n"
                % (os.path.basename(sys.argv[1]), len(T), info["source_triangles"],
                   info["parts_kept"], info["parts_dropped"], info["cell_mm"],
                   ext[0], ext[1], ext[2]))
        f.write("GROUPS = %r\n\n" % [list(g) for g in GROUPS])
        hx = blob.hex()
        f.write("DATA = (\n")
        for k in range(0, len(hx), 96):
            f.write('    "%s"\n' % hx[k:k + 96])
        f.write(")\n")
    print("%d triangles, %d vertices, %d bytes hex; extent %.0f x %.0f x %.0f mm; %s"
          % (len(T), len(V), len(blob) * 2, ext[0], ext[1], ext[2], info))
    print("wrote", os.path.normpath(OUT))


if __name__ == "__main__":
    main()
