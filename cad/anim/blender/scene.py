"""Photoreal AKORA scene in Blender (Cycles), driven by the CAD kinematics.

Used as a library by render.py. Units: 1 Blender unit = 1 m. Model data (mm, Z-up) is scaled by 0.001.
"""
import json
import math
import os

import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
DATA = os.path.join(ROOT, "output", "animation")

CH0, CH1 = 1.85, 3.92           # flooded channel between the banks (m, model x)
WATER_Z = -0.09


# ------------------------------------------------------------------ helpers
def node_mat(name):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    nt.links.new(bsdf.outputs[0], out.inputs["Surface"])
    return m, nt, bsdf


def set_in(bsdf, key, val):
    for k in ([key] if isinstance(key, str) else key):
        if k in bsdf.inputs:
            bsdf.inputs[k].default_value = val
            return


def noise_bump(nt, bsdf, scale=200.0, strength=0.08, detail=8.0):
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = scale
    nz.inputs["Detail"].default_value = detail
    bp = nt.nodes.new("ShaderNodeBump")
    bp.inputs["Strength"].default_value = strength
    nt.links.new(tc.outputs["Object"], nz.inputs["Vector"])
    nt.links.new(nz.outputs["Fac"], bp.inputs["Height"])
    nt.links.new(bp.outputs["Normal"], bsdf.inputs["Normal"])
    return nz


# ------------------------------------------------------------------ robot materials (PBR versions of akora/materials.py)
ROBOT_MATS = {
    "polymer_black": dict(c=(0.018, 0.018, 0.02), r=0.42, coat=0.15, bump=0.04),
    "polymer_dark": dict(c=(0.035, 0.035, 0.04), r=0.5, bump=0.04),
    "orange": dict(c=(0.85, 0.16, 0.012), r=0.32, coat=0.6, m=0.0, bump=0.02),
    "rubber": dict(c=(0.012, 0.012, 0.013), r=0.82, bump=0.25),
    "antislip": dict(c=(0.03, 0.03, 0.033), r=0.55, m=0.7, bump=0.15),
    "frame_black": dict(c=(0.03, 0.03, 0.034), r=0.38, m=0.8),
    "brushed": dict(c=(0.75, 0.75, 0.76), r=0.22, m=1.0, aniso=0.7),
    "stainless": dict(c=(0.6, 0.6, 0.62), r=0.18, m=1.0),
    "aluminum": dict(c=(0.7, 0.71, 0.73), r=0.3, m=1.0),
    "steel_dark": dict(c=(0.12, 0.12, 0.13), r=0.35, m=0.9),
    "silver_cap": dict(c=(0.7, 0.7, 0.72), r=0.25, m=1.0),
    "epdm": dict(c=(0.03, 0.03, 0.03), r=0.9),
    "pcb": dict(c=(0.01, 0.15, 0.05), r=0.4),
    "battery": dict(c=(0.06, 0.12, 0.3), r=0.4),
    "glass": dict(c=(0.8, 0.85, 0.9), r=0.02, trans=1.0),
    "reflector": dict(c=(1.0, 0.35, 0.0), r=0.15, emit=0.4),
    "red": dict(c=(0.6, 0.01, 0.01), r=0.35),
    "cable": dict(c=(0.02, 0.02, 0.02), r=0.6),
    "led": dict(c=(1, 1, 0.95), r=0.1, emit=6.0),
}


def robot_material(name):
    p = ROBOT_MATS.get(name, dict(c=(0.5, 0.5, 0.5), r=0.5))
    m, nt, b = node_mat("AK_" + name)
    set_in(b, "Base Color", (*p["c"], 1))
    set_in(b, "Roughness", p.get("r", 0.5))
    set_in(b, "Metallic", p.get("m", 0.0))
    if p.get("coat"):
        set_in(b, ["Coat Weight", "Clearcoat"], p["coat"])
        set_in(b, ["Coat Roughness", "Clearcoat Roughness"], 0.08)
    if p.get("aniso"):
        set_in(b, "Anisotropic", p["aniso"])
    if p.get("trans"):
        set_in(b, ["Transmission Weight", "Transmission"], p["trans"])
        set_in(b, "IOR", 1.5)
    if p.get("emit"):
        set_in(b, ["Emission Color", "Emission"], (*p["c"], 1))
        set_in(b, "Emission Strength", p["emit"])
    if p.get("bump"):
        noise_bump(nt, b, scale=400.0, strength=p["bump"])
    return m


# ------------------------------------------------------------------ robot (GLB from build_anim.py)
class Robot:
    def __init__(self):
        self.meta = json.load(open(os.path.join(DATA, "akora_anim.json")))
        self.K = len(self.meta["keys"]) + 2 * self.meta["n_links"]
        self.bin = np.fromfile(os.path.join(DATA, "akora_anim.bin"), dtype=np.float32).reshape(-1, self.K, 16)
        before = set(bpy.data.objects)
        bpy.ops.import_scene.gltf(filepath=os.path.join(DATA, "akora_anim.glb"))
        objs = [o for o in bpy.data.objects if o not in before]
        self.root = bpy.data.objects.new("AKORA_ROOT", None)
        bpy.context.scene.collection.objects.link(self.root)
        self.root.scale = (0.001, 0.001, 0.001)
        self.nodes = {}
        self.links = {"L": None, "R": None}
        mats = {}
        fix = Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, -1, 0, 0), (0, 0, 0, 1)))   # undo the importer's Y-up -> Z-up turn
        done = set()
        for o in objs:
            if o.type == "MESH" and o.data.name not in done:
                o.data.transform(o.matrix_basis)
                o.data.transform(fix)
                done.add(o.data.name)
        for o in objs:
            if o.type != "MESH":
                continue
            key, mat = o.name.split("__")[0], o.name.split("__")[-1].split(".")[0]
            if mat not in mats:
                mats[mat] = robot_material(mat)
            o.data.materials.clear()
            o.data.materials.append(mats[mat])
            for poly in o.data.polygons:
                poly.use_smooth = False
            o.parent = self.root
            o.matrix_parent_inverse = Matrix.Identity(4)
            if key.startswith("LINKTEMPLATE_"):
                self.links[key.split("_")[1]] = o
            else:
                self.nodes.setdefault(key, []).append(o)
        for o in objs:
            if o.type != "MESH" and o.name in bpy.data.objects:
                bpy.data.objects.remove(o, do_unlink=True)
        # track links: instances of the template mesh
        self.link_objs = {}
        for side, tmpl in self.links.items():
            if tmpl is None:
                continue
            lst = []
            for j in range(self.meta["n_links"]):
                ob = bpy.data.objects.new(f"LINK_{side}_{j:02d}", tmpl.data)
                ob.parent = self.root
                bpy.context.scene.collection.objects.link(ob)
                lst.append(ob)
            tmpl.hide_render = True
            self.link_objs[side] = lst

    def set_frame(self, i):
        i = min(i, self.bin.shape[0] - 1)
        d = self.bin[i]
        for k, key in enumerate(self.meta["keys"]):
            M = Matrix(np.array(d[k]).reshape(4, 4).T.tolist())
            for o in self.nodes.get(key, []):
                o.matrix_basis = M
        nk = len(self.meta["keys"])
        for s, side in enumerate(("L", "R")):
            for j, ob in enumerate(self.link_objs.get(side, [])):
                ob.matrix_basis = Matrix(np.array(d[nk + s * self.meta["n_links"] + j]).reshape(4, 4).T.tolist())

    def root_x(self, i):
        return self.meta["root_x_mm"][min(i, len(self.meta["root_x_mm"]) - 1)] / 1000.0


# ------------------------------------------------------------------ environment
def world_overcast(strength=1.0):
    w = bpy.data.worlds.get("W") or bpy.data.worlds.new("W")
    bpy.context.scene.world = w
    w.use_nodes = True
    nt = w.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    sky = nt.nodes.new("ShaderNodeTexSky")
    for st in ("MULTIPLE_SCATTERING", "NISHITA", "SINGLE_SCATTERING", "PREETHAM"):
        try:
            sky.sky_type = st
            break
        except Exception:
            pass
    try:
        sky.sun_elevation = math.radians(28)
        sky.sun_rotation = math.radians(140)
        sky.air_density = 3.0
        sky.dust_density = 8.0
    except Exception:
        pass
    # desaturate the clear sky into an overcast dome
    hsv = nt.nodes.new("ShaderNodeHueSaturation")
    hsv.inputs["Saturation"].default_value = 0.15
    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type = "RGBA"
    mix.inputs[0].default_value = 0.55
    mix.inputs[7].default_value = (0.42, 0.45, 0.5, 1)
    nt.links.new(sky.outputs[0], hsv.inputs["Color"])
    nt.links.new(hsv.outputs[0], mix.inputs[6])
    nt.links.new(mix.outputs[2], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = strength * 0.55
    clouds = nt.nodes.new("ShaderNodeTexNoise"); clouds.inputs["Scale"].default_value = 2.2; clouds.inputs["Detail"].default_value = 6
    cr = nt.nodes.new("ShaderNodeValToRGB")
    cr.color_ramp.elements[0].color = (0.2, 0.22, 0.25, 1); cr.color_ramp.elements[1].color = (0.5, 0.52, 0.55, 1)
    nt.links.new(clouds.outputs["Fac"], cr.inputs["Fac"])
    bg2 = nt.nodes.new("ShaderNodeBackground"); bg2.inputs["Strength"].default_value = 1.0
    nt.links.new(cr.outputs["Color"], bg2.inputs["Color"])
    lp = nt.nodes.new("ShaderNodeLightPath")
    ms = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(lp.outputs["Is Camera Ray"], ms.inputs[0])
    nt.links.new(bg.outputs[0], ms.inputs[1]); nt.links.new(bg2.outputs[0], ms.inputs[2])
    nt.links.new(ms.outputs[0], out.inputs["Surface"])
    sun = bpy.data.lights.new("Sun", "SUN")
    sun.energy = 1.1
    sun.angle = math.radians(18)          # very soft overcast shadows
    sun.color = (1.0, 0.97, 0.93)
    so = bpy.data.objects.new("Sun", sun)
    bpy.context.scene.collection.objects.link(so)
    so.rotation_euler = (math.radians(50), 0, math.radians(140))


def mat_asphalt():
    m, nt, b = node_mat("Asphalt")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    n1 = nt.nodes.new("ShaderNodeTexNoise"); n1.inputs["Scale"].default_value = 180; n1.inputs["Detail"].default_value = 12
    n2 = nt.nodes.new("ShaderNodeTexNoise"); n2.inputs["Scale"].default_value = 0.35; n2.inputs["Detail"].default_value = 4
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.52; ramp.color_ramp.elements[0].color = (0.8, 0.8, 0.8, 1)
    ramp.color_ramp.elements[1].position = 0.62; ramp.color_ramp.elements[1].color = (0.22, 0.22, 0.22, 1)
    col = nt.nodes.new("ShaderNodeValToRGB")
    col.color_ramp.elements[0].color = (0.022, 0.021, 0.02, 1); col.color_ramp.elements[1].color = (0.075, 0.07, 0.065, 1)
    nt.links.new(tc.outputs["Object"], n1.inputs["Vector"]); nt.links.new(tc.outputs["Object"], n2.inputs["Vector"])
    nt.links.new(n1.outputs["Fac"], col.inputs["Fac"]); nt.links.new(col.outputs["Color"], b.inputs["Base Color"])
    nt.links.new(n2.outputs["Fac"], ramp.inputs["Fac"]); nt.links.new(ramp.outputs["Color"], b.inputs["Roughness"])   # wet puddles
    bp = nt.nodes.new("ShaderNodeBump"); bp.inputs["Strength"].default_value = 0.25
    nt.links.new(n1.outputs["Fac"], bp.inputs["Height"]); nt.links.new(bp.outputs["Normal"], b.inputs["Normal"])
    return m


def mat_simple(name, c, r, bump=0.0, scale=40.0):
    m, nt, b = node_mat(name)
    set_in(b, "Base Color", (*c, 1)); set_in(b, "Roughness", r)
    if bump:
        noise_bump(nt, b, scale=scale, strength=bump)
    return m


def mat_water():
    m, nt, b = node_mat("FloodWater")
    set_in(b, "Base Color", (0.12, 0.08, 0.04, 1))
    set_in(b, "Roughness", 0.1)
    set_in(b, ["Transmission Weight", "Transmission"], 0.0)
    noise_bump(nt, b, scale=6.0, strength=0.35, detail=6.0)
    set_in(b, "IOR", 1.33)
    set_in(b, ["Specular IOR Level", "Specular"], 0.6)
    return m


def mat_facade(seed):
    """Plaster facade with a window grid from a brick texture (windows = dark glass)."""
    m, nt, b = node_mat(f"Facade{seed}")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    br = nt.nodes.new("ShaderNodeTexBrick")
    br.offset = 0.0; br.squash = 1.0
    br.inputs["Scale"].default_value = 1.0
    br.inputs["Mortar Size"].default_value = 0.35
    br.inputs["Brick Width"].default_value = 1.15
    br.inputs["Row Height"].default_value = 1.4
    br.inputs["Color1"].default_value = (0.04, 0.05, 0.06, 1)
    br.inputs["Color2"].default_value = (0.05, 0.055, 0.065, 1)
    tones = [(0.34, 0.3, 0.26), (0.4, 0.35, 0.29), (0.28, 0.27, 0.25), (0.37, 0.31, 0.26)]
    br.inputs["Mortar"].default_value = (*tones[seed % 4], 1)
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    add = nt.nodes.new("ShaderNodeMath"); add.operation = "ADD"
    comb = nt.nodes.new("ShaderNodeCombineXYZ")
    nt.links.new(tc.outputs["Object"], sep.inputs[0])
    nt.links.new(sep.outputs[0], add.inputs[0]); nt.links.new(sep.outputs[1], add.inputs[1])
    nt.links.new(add.outputs[0], comb.inputs[0]); nt.links.new(sep.outputs[2], comb.inputs[1])
    nt.links.new(comb.outputs[0], br.inputs["Vector"])
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    sepn = nt.nodes.new("ShaderNodeSeparateXYZ")
    absn = nt.nodes.new("ShaderNodeMath"); absn.operation = "ABSOLUTE"
    gt = nt.nodes.new("ShaderNodeMath"); gt.operation = "GREATER_THAN"; gt.inputs[1].default_value = 0.7
    mixc = nt.nodes.new("ShaderNodeMix"); mixc.data_type = "RGBA"
    mixc.inputs[7].default_value = (0.12, 0.11, 0.1, 1)
    nt.links.new(geo.outputs["Normal"], sepn.inputs[0]); nt.links.new(sepn.outputs[2], absn.inputs[0])
    nt.links.new(absn.outputs[0], gt.inputs[0]); nt.links.new(gt.outputs[0], mixc.inputs[0])
    nt.links.new(br.outputs["Color"], mixc.inputs[6])
    nt.links.new(mixc.outputs[2], b.inputs["Base Color"])
    rough = nt.nodes.new("ShaderNodeMapRange")
    rough.inputs["From Min"].default_value = 0.0; rough.inputs["From Max"].default_value = 1.0
    rough.inputs["To Min"].default_value = 0.05; rough.inputs["To Max"].default_value = 0.85
    nt.links.new(br.outputs["Fac"], rough.inputs["Value"])
    nt.links.new(rough.outputs["Result"], b.inputs["Roughness"])
    return m


def add_box(name, x0, x1, y0, y1, z0, z1, mat):
    bpy.ops.mesh.primitive_cube_add(size=1, location=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    o = bpy.context.active_object
    o.name = name
    o.scale = (x1 - x0, y1 - y0, z1 - z0)
    bpy.ops.object.transform_apply(scale=True)
    o.data.materials.append(mat)
    return o


def environment(with_buildings=True):
    world_overcast()
    asph = mat_asphalt()
    stone = mat_simple("Stone", (0.2, 0.18, 0.16), 0.85, bump=0.6, scale=12)
    mud = mat_simple("Mud", (0.07, 0.05, 0.03), 0.9)
    add_box("NearBank", -80, CH0, -60, 60, -0.3, 0.0, asph)
    add_box("FarBank", CH1, 80, -60, 60, -0.3, 0.0, asph)
    add_box("WallA", CH0 - 0.25, CH0, -60, 60, -1.5, 0.0, stone)
    add_box("WallB", CH1, CH1 + 0.25, -60, 60, -1.5, 0.0, stone)
    add_box("Bed", CH0, CH1, -60, 60, -1.5, -1.4, mud)
    # flowing water: ocean modifier on a plane between the walls
    bpy.ops.mesh.primitive_plane_add(size=1, location=((CH0 + CH1) / 2, 0, WATER_Z))
    w = bpy.context.active_object
    w.name = "Water"
    w.scale = (CH1 - CH0, 120, 1)
    bpy.ops.object.transform_apply(scale=True)
    oc = w.modifiers.new("Ocean", "OCEAN")
    oc.geometry_mode = "DISPLACE"
    oc.resolution = 12
    oc.spatial_size = 8
    oc.wave_scale = 0.22
    oc.choppiness = 0.8
    oc.wind_velocity = 6.0
    oc.wave_direction = math.radians(90)
    oc.damping = 0.3
    w.data.materials.append(mat_water())
    bpy.ops.object.shade_smooth()
    if with_buildings:
        for side in (-1, 1):
            x = -32.0
            i = 0
            while x < 38.0:
                wdt = 5 + (i * 1.37 % 3.8)
                h = 7 + (i * 2.71 % 9)
                if x + wdt > CH0 - 0.6 and x < CH1 + 0.6:
                    x = CH1 + 0.8
                    continue
                y0 = side * 10.5
                add_box(f"Bld_{side}_{i}", x, x + wdt, min(y0, y0 + side * 9), max(y0, y0 + side * 9), 0, h, mat_facade(i + (side > 0)))
                x += wdt + 0.4
                i += 1
        add_box("FarBuilding", 8.0, 17.0, -8, 8, 0, 11, mat_facade(7))
        add_box("Door", 7.97, 8.0, -0.7, 0.7, 0, 2.3, mat_simple("DoorDark", (0.02, 0.02, 0.025), 0.4))
        add_box("Stoop", 7.6, 8.0, -1.3, 1.3, 0, 0.14, mat_simple("Concrete", (0.35, 0.33, 0.3), 0.8, bump=0.3, scale=30))
    return w


def set_ocean_time(water, t):
    water.modifiers["Ocean"].time = t * 1.2


def camera(name="Cam", lens=35):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.sensor_width = 36
    co = bpy.data.objects.new(name, cd)
    bpy.context.scene.collection.objects.link(co)
    bpy.context.scene.camera = co
    return co


def look_at(cam, pos, tgt):
    cam.location = Vector(pos)
    d = Vector(tgt) - Vector(pos)
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def render_settings(w=1280, h=720, samples=24, mist=True):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    try:
        sc.cycles.denoiser = "OPENIMAGEDENOISE"
    except Exception:
        pass
    sc.cycles.max_bounces = 4
    sc.cycles.diffuse_bounces = 2
    sc.cycles.glossy_bounces = 2
    sc.cycles.transmission_bounces = 3
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.adaptive_threshold = 0.05
    sc.render.use_persistent_data = True
    sc.cycles.caustics_reflective = False
    sc.cycles.caustics_refractive = False
    sc.cycles.use_light_tree = True
    sc.render.resolution_x, sc.render.resolution_y = w, h
    sc.render.resolution_percentage = 100
    sc.render.image_settings.file_format = "JPEG"
    sc.render.image_settings.quality = 93
    sc.view_settings.view_transform = "AgX" if "AgX" in [v.identifier for v in type(sc.view_settings).bl_rna.properties["view_transform"].enum_items] else "Filmic"
    try:
        sc.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass
    sc.render.threads_mode = "AUTO"
