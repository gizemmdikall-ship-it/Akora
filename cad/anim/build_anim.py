"""Product-animation data: one GLB with every moving frame as its own node + per-frame matrices.

    python anim/build_anim.py   ->  output/animation/akora_anim.glb, akora_anim.bin, akora_anim.json
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from akora import drill, export, model  # noqa: E402
from akora import params as P  # noqa: E402
from akora.geom import circles_hull  # noqa: E402
from akora.kinematics import frames  # noqa: E402
from akora.materials import MATERIALS  # noqa: E402

FPS = 30
DUR = 60.0
OUT = os.path.join(ROOT, "output", "animation")


def seg(t, a, b):
    return 0.0 if t <= a else 1.0 if t >= b else (t - a) / (b - a)


def ease(x):
    return x * x * (3 - 2 * x)


# ------------------------------------------------------------------ timeline (seconds)
def state_at(t):
    """Robot state + root x (mm) for time t. Same timing is used for captions in anim.html."""
    e = lambda a, b: ease(seg(t, a, b))
    drive = e(0.5, 8.0)
    x_root = -5000 * (1 - drive)
    # opening (PDF order): P2 to vertical tower -> P3 swings level -> both down
    j1 = 0.5 * e(12.0, 15.0) + 0.5 * e(17.0, 21.0)
    j2 = 0.5 * e(15.0, 17.0) + 0.5 * e(17.0, 21.0)
    rs = e(21.0, 25.0)
    fs = e(21.5, 25.5)
    legs = e(25.5, 28.5)
    rails = e(28.5, 31.5)
    # closing (reverse order)
    rails -= e(45.0, 46.5)
    legs -= e(46.5, 48.0)
    rs -= e(48.0, 50.5)
    fs -= e(48.0, 50.5)
    j1 -= 0.5 * e(50.5, 52.5) + 0.5 * e(53.5, 55.5)
    j2 -= 0.5 * e(50.5, 52.5) + 0.5 * e(52.5, 53.5)
    st = dict(j1=j1, j2=j2, rs=rs, fs=fs, legs=legs, rails=rails, cover="on")
    return st, x_root, drive


def belt_sign():
    """Pick the circulation direction that keeps the ground-contact links still in the world."""
    a, _ = link_frames(0)
    b, _ = link_frames(50)
    k = int(np.argmin([M[2, 3] for M in a]))         # a link on the bottom run
    dx = b[k][0, 3] - a[k][0, 3]                      # +50 mm of shift moves it by dx along x
    return -1.0 if dx > 0 else 1.0                    # hull +x  -> link must move -x relative to hull


def link_frames(shift):
    """Track-link placement along the belt path, shifted by 'shift' mm (belt circulation)."""
    circles = [P.SPROCKET, P.IDLER] + P.ROAD_WHEELS
    poly = circles_hull(circles, 0)
    line = poly.exterior
    Lp = line.length
    n = int(round(Lp / P.LINK_PITCH))
    pitch = Lp / n
    ccw = poly.exterior.is_ccw
    out = []
    for k in range(n):
        d = (k * pitch + shift) % Lp
        p0 = line.interpolate(d)
        p1 = line.interpolate((d + 1.0) % Lp)
        tvec = np.array([p1.x - p0.x, 0, p1.y - p0.y])
        tvec /= np.linalg.norm(tvec)
        nrm = np.array([tvec[2], 0, -tvec[0]]) if ccw else np.array([-tvec[2], 0, tvec[0]])
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2] = tvec, (0, 1, 0), nrm
        if np.linalg.det(M[:3, :3]) < 0:
            M[:3, 0] = -tvec
        M[:3, 3] = (p0.x, 0, p0.y)
        out.append(M)
    return out, Lp


def main():
    import trimesh
    from trimesh.visual.material import PBRMaterial
    os.makedirs(OUT, exist_ok=True)
    parts = model.build_all()
    drill.drill(parts)

    # ---- geometry: one node per kinematic frame (merged per material), links individually
    nodes = {}          # node key -> {mat: [(v, f)]}
    link_parts = []
    for p in parts:
        if p.name.startswith("TRACK_LINK_"):
            link_parts.append(p)
            continue
        v, f = export.mesh_of(p)
        if not len(f):
            continue
        vw = (p.M[:3, :3] @ v.T).T + p.M[:3, 3]
        nodes.setdefault(p.frame, {}).setdefault(p.mat, []).append((vw, f))
    # one link per side as template, n instances
    tmpl = {}
    for p in link_parts:
        side = p.name.split("_")[2]
        if side not in tmpl:
            tmpl[side] = p

    scene = trimesh.Scene()

    def add_mesh(node, mat, lst):
        vs, fs, off = [], [], 0
        for v, f in lst:
            vs.append(v)
            fs.append(f + off)
            off += len(v)
        rgb, metal, rough, alpha, _ = MATERIALS[mat]
        m = trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=False)
        m.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
            name=mat, baseColorFactor=[*(c ** 2.2 for c in rgb), alpha], metallicFactor=metal,
            roughnessFactor=rough, alphaMode="BLEND" if alpha < 1 else "OPAQUE", doubleSided=True))
        scene.add_geometry(m, node_name=f"{node}__{mat}", geom_name=f"{node}__{mat}")

    for node, mats in nodes.items():
        for mat, lst in mats.items():
            add_mesh(node, mat, lst)
    for side, p in tmpl.items():
        v, f = export.mesh_of(p)
        add_mesh(f"LINKTEMPLATE_{side}", p.mat, [(v, f)])
    scene.export(os.path.join(OUT, "akora_anim.glb"))

    # ---- per-frame matrices (column-major float32), model units mm, Z-up
    keys = sorted(nodes.keys())
    lf0, Lp = link_frames(0)
    n_links = len(lf0)
    N = int(DUR * FPS)
    K = len(keys) + 2 * n_links
    data = np.zeros((N, K, 16), np.float32)
    roots = []
    global BELT_SIGN
    BELT_SIGN = belt_sign()
    for i in range(N):
        t = i / FPS
        st, xr, drive = state_at(t)
        F = frames(st)
        R = np.eye(4)
        R[0, 3] = xr
        roots.append(float(xr))
        for k, key in enumerate(keys):
            data[i, k] = (R @ F[key]).T.reshape(-1)
        # belt circulation: bottom run stays still on the ground while the hull moves
        lfs, _ = link_frames(BELT_SIGN * (xr + 5000))
        for s, side in enumerate(("L", "R")):
            for j, M in enumerate(lfs):           # right side uses the mirrored template, same placement
                data[i, len(keys) + s * n_links + j] = (R @ M).T.reshape(-1)
    data.tofile(os.path.join(OUT, "akora_anim.bin"))
    meta = dict(fps=FPS, frames=N, keys=keys, n_links=n_links, link_sides=["L", "R"],
                deck_z_mm=P.DECK_Z, root_x_mm=roots)
    with open(os.path.join(OUT, "akora_anim.json"), "w") as f:
        json.dump(meta, f)
    print("nodes", len(keys), "links", n_links, "frames", N, "bin MB", round(data.nbytes / 1e6, 1))


if __name__ == "__main__":
    main()
