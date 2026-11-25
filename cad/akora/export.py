"""Exporters: STEP (NURBS B-rep, per configuration), Rhino 3DM (layer tree), GLB (render)."""
import math
import os

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

from . import params as P
from .kinematics import CONFIGS, EXPLODE, frames
from .materials import MATERIALS

LAYER_ORDER = ["01_MAIN_BODY", "02_TRACK_SYSTEM", "03_PLATFORM_SYSTEM", "04_STAIR_SYSTEM",
               "05_GUARDRAIL_SYSTEM", "06_SUPPORT_SYSTEM", "07_ELECTRONICS", "08_POWER_SYSTEM",
               "09_FASTENERS", "10_ASSEMBLY", "11_EXPLODED_VIEW"]

EXPLODED_LAYER = {  # top layer -> 11_EXPLODED_VIEW sub layer
    "01_MAIN_BODY": "EXPLODED_BODY", "02_TRACK_SYSTEM": "EXPLODED_TRACKS", "03_PLATFORM_SYSTEM": "EXPLODED_PLATFORM",
    "04_STAIR_SYSTEM": "EXPLODED_STAIRS", "05_GUARDRAIL_SYSTEM": "EXPLODED_GUARDRAILS",
    "06_SUPPORT_SYSTEM": "EXPLODED_SUPPORTS", "07_ELECTRONICS": "EXPLODED_ELECTRONICS",
    "08_POWER_SYSTEM": "EXPLODED_ELECTRONICS", "09_FASTENERS": "EXPLODED_BODY",
}


# ------------------------------------------------------------------ placement
PLATFORM_GROUPS = {"P1", "P2", "P3", "landing", "rear_stair", "front_stair", "legs"}

def placed(parts, cfg):
    F = frames(cfg["state"])
    out = []
    for p in parts:
        if cfg["layers"]:
            if p.layer[0] not in cfg["layers"]:
                continue
            # fasteners follow the sub-assembly they belong to
            if p.layer[0] == "09_FASTENERS":
                plat = p.group in PLATFORM_GROUPS or p.group.startswith(("J1_", "J2_", "rail_"))
                if plat and "03_PLATFORM_SYSTEM" not in cfg["layers"]:
                    continue
                if not plat and "01_MAIN_BODY" not in cfg["layers"]:
                    continue
        if p.group in cfg["hide_groups"]:
            continue
        M = F[p.frame] @ p.M
        if cfg["explode"]:
            M = M.copy()
            M[:3, 3] += np.asarray(EXPLODE.get(p.group, (0, 0, 0)), float)
        out.append((p, M))
    return out


def to_loc(M):
    t = gp_Trsf()
    t.SetValues(*[float(v) for v in M[0, :4]], *[float(v) for v in M[1, :4]], *[float(v) for v in M[2, :4]])
    return cq.Location(t)


# ------------------------------------------------------------------ STEP
def write_step(parts, cfg, path):
    """NURBS B-rep STEP (AP214), nested assemblies = Rhino layer tree, written in centimetres."""
    tree = {}
    for p, M in placed(parts, cfg):
        tree.setdefault(p.layer, []).append((p, M))
    paths = set()
    for lp in tree:
        for i in range(1, len(lp) + 1):
            paths.add(lp[:i])

    layer_names = set()

    def uname(lp):
        return lp[0] if len(lp) == 1 else "_".join(lp[1:])

    def build(lp):
        a = cq.Assembly(name=uname(lp))
        for p, M in tree.get(lp, []):
            rgb, *_ = MATERIALS[p.mat]
            nm = p.name + "_PART" if p.name in layer_names else p.name
            a.add(p.shape, name=nm, loc=to_loc(M), color=cq.Color(*rgb))
        for child in sorted(q for q in paths if len(q) == len(lp) + 1 and q[:len(lp)] == lp):
            a.add(build(child))
        return a

    layer_names.update(uname(q) for q in paths)
    root = cq.Assembly(name="AKORA_" + cfg["name"])
    for top in sorted(q for q in paths if len(q) == 1):
        root.add(build(top))
    root.export(path, exportType="STEP", unit="MM", outputUnit=P.EXPORT_UNIT.upper())


# ------------------------------------------------------------------ meshes
_MESH = {}


def mesh_of(p, tol=1.5, ang=0.35):
    key = id(p.shape)
    if key not in _MESH:
        vs, ts = p.shape.tessellate(tol, ang)
        v = np.array([(q.x, q.y, q.z) for q in vs], float) if vs else np.zeros((0, 3))
        f = np.array(ts, np.int64) if ts else np.zeros((0, 3), np.int64)
        _MESH[key] = (v, f)
    return _MESH[key]


def world_mesh(p, M):
    v, f = mesh_of(p)
    if len(v) == 0:
        return v, f
    vw = (M[:3, :3] @ v.T).T + M[:3, 3]
    return vw, f


# ------------------------------------------------------------------ GLB (for renders / viewer)
def write_glb(parts, cfg, path, merge_by_material=True):
    import trimesh
    from trimesh.visual.material import PBRMaterial
    scene = trimesh.Scene()
    buckets = {}
    for p, M in placed(parts, cfg):
        v, f = world_mesh(p, M)
        if len(f) == 0:
            continue
        key = p.mat if merge_by_material else p.name
        buckets.setdefault((key, p.mat), []).append((v * P.EXPORT_SCALE, f))
    for (key, mat), lst in buckets.items():
        vs, fs, off = [], [], 0
        for v, f in lst:
            vs.append(v)
            fs.append(f + off)
            off += len(v)
        rgb, metal, rough, alpha, _ = MATERIALS[mat]
        lin = [c ** 2.2 for c in rgb]                   # glTF factors are linear
        m = trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=False)
        m.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
            name=mat, baseColorFactor=[*lin, alpha], metallicFactor=metal, roughnessFactor=rough,
            alphaMode="BLEND" if alpha < 1 else "OPAQUE", doubleSided=True))
        # glTF is Y-up: rotate Z-up model
        m.apply_transform(np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float))
        scene.add_geometry(m, node_name=str(key), geom_name=str(key))
    scene.export(path)


# ------------------------------------------------------------------ Rhino 3DM
def write_3dm(parts, path, main_cfg, assembly_cfgs, exploded_cfg):
    import rhino3dm as r3
    f = r3.File3dm()
    f.Settings.ModelUnitSystem = r3.UnitSystem.Centimeters
    f.Settings.ModelAbsoluteTolerance = 0.01
    f.ApplicationName = "AKORA parametric CAD (CadQuery/OpenCASCADE)"
    f.Notes = ("AKORA 1:1, birim cm. NURBS katılar STEP dosyalarındadır (Rhino: Import/Open .stp). "
               "Bu 3DM, aynı geometrinin katman ağacına yerleştirilmiş görüntü mesh'lerini içerir.")

    # materials
    mat_index = {}
    for name, (rgb, metal, rough, alpha, desc) in MATERIALS.items():
        m = r3.Material()
        m.Name = name
        m.DiffuseColor = tuple(int(c * 255) for c in rgb) + (255,)
        m.Transparency = 1 - alpha
        m.Reflectivity = metal * 0.6
        mat_index[name] = f.Materials.Add(m)

    layer_ids = {}

    def layer(path_):
        if path_ in layer_ids:
            return layer_ids[path_]
        L = r3.Layer()
        L.Name = path_[-1]
        if len(path_) > 1:
            L.ParentLayerId = f.Layers[layer(path_[:-1])].Id
        idx = f.Layers.Add(L)
        layer_ids[path_] = idx
        return idx

    for top in LAYER_ORDER:
        layer((top,))

    def add_mesh(v, fcs, name, lay, mat, user=None):
        mesh = r3.Mesh()
        for x, y, z in (v * P.EXPORT_SCALE):
            mesh.Vertices.Add(float(x), float(y), float(z))
        for a, b, c in fcs:
            mesh.Faces.AddFace(int(a), int(b), int(c))
        mesh.Normals.ComputeNormals()
        mesh.Compact()
        at = r3.ObjectAttributes()
        at.Name = name
        at.LayerIndex = lay
        at.MaterialIndex = mat_index[mat]
        at.MaterialSource = r3.ObjectMaterialSource.MaterialFromObject
        at.ColorSource = r3.ObjectColorSource.ColorFromMaterial
        for k, val in (user or {}).items():
            at.SetUserString(k, str(val))
        f.Objects.AddMesh(mesh, at)

    # 01..09: the main (closed) model, one object per part, full layer tree
    for p, M in placed(parts, main_cfg):
        v, fc = world_mesh(p, M)
        if len(fc):
            add_mesh(v, fc, p.name, layer(p.layer), p.mat,
                     {"material": MATERIALS[p.mat][4], "layer_path": "/".join(p.layer), "frame": p.frame})
    # 10_ASSEMBLY: every configuration on its own sub-layer (turned off except CLOSED)
    for cfg in assembly_cfgs:
        li = layer(("10_ASSEMBLY", cfg["rhino_layer"]))
        f.Layers[li].Visible = False
        for p, M in placed(parts, cfg):
            v, fc = world_mesh(p, M)
            if len(fc):
                add_mesh(v, fc, p.name, li, p.mat, {"configuration": cfg["name"]})
    # 11_EXPLODED_VIEW
    for p, M in placed(parts, exploded_cfg):
        v, fc = world_mesh(p, M)
        if len(fc):
            li = layer(("11_EXPLODED_VIEW", EXPLODED_LAYER[p.layer[0]]))
            add_mesh(v, fc, p.name, li, p.mat, {"configuration": exploded_cfg["name"]})
    for k, idx in layer_ids.items():
        if k[0] == "11_EXPLODED_VIEW":
            f.Layers[idx].Visible = False
    f.Write(path, 8)
