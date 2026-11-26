"""AKORA product film - photoreal Cycles render driven by the CAD kinematics.

    python anim/blender/film.py stills            # one still per shot (preview)
    python anim/blender/film.py render A B        # render film frames A..B-1 (12 fps), skips existing
Frames go to output/film/frames/f_00000.jpg; overlay.json collects 2D annotation data for post.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(HERE))

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402
from bpy_extras.object_utils import world_to_camera_view  # noqa: E402

import scene as S  # noqa: E402
from akora.kinematics import frames as kin_frames  # noqa: E402

FPS = 12
OUT = os.path.join(ROOT, "output", "film")
FRAMES = os.path.join(OUT, "frames")
FLOOD_Z = 0.06                     # shallow flood: tracks stand ~6 cm deep at the water edge
DRY_X, DRY_Z = -0.84, 0.10          # safe side rises out of the water: rear stair foot lands on dry ground


def ease(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def seg(t, a, b):
    return ease((t - a) / (b - a))


# ------------------------------------------------------------------ mechanism timeline (s)
def mech_state(t):
    """Opening order: rear stair -> J1 (P2 vertical) -> J2 (P3 level) -> both down -> front stair -> legs -> rails."""
    rs = seg(t, 28.3, 31.8)
    j1 = 0.5 * seg(t, 32.3, 36.6) + 0.5 * seg(t, 41.2, 43.8)
    j2 = 0.5 * seg(t, 37.3, 40.7) + 0.5 * seg(t, 41.2, 43.8)
    fs = seg(t, 44.2, 45.8)
    legs = seg(t, 46.2, 47.8)
    rails = seg(t, 48.1, 49.8)
    # closing, reverse
    rails -= seg(t, 85.3, 87.7)
    legs -= seg(t, 88.3, 90.7)
    rs -= seg(t, 91.3, 93.7)
    fs -= seg(t, 91.3, 93.5)
    j1 -= 0.5 * seg(t, 94.3, 96.4) + 0.5 * seg(t, 97.9, 99.7)
    j2 -= 0.5 * seg(t, 94.3, 96.4) + 0.5 * seg(t, 96.6, 97.8)
    return dict(j1=j1, j2=j2, rs=rs, fs=fs, legs=legs, rails=rails, cover="on")


CLOSED = dict(j1=0, j2=0, rs=0, fs=0, legs=0, rails=0, cover="on")
OPEN = dict(j1=1, j2=1, rs=1, fs=1, legs=1, rails=1, cover="on")


def drive_x(t):
    return -7.0 * (1 - seg(t, 19.0, 24.4))


# ------------------------------------------------------------------ shots
def lerp3(a, b, u):
    return tuple(a[i] + (b[i] - a[i]) * u for i in range(3))


def orbit(center, r, a0, a1, z0, z1):
    def f(u):
        a = math.radians(a0 + (a1 - a0) * u)
        return (center[0] + r * math.cos(a), center[1] + r * math.sin(a), z0 + (z1 - z0) * u)
    return f


SHOTS = [
    # name, t0, t1, set, camera pos(u), target(u), lens(u), robot state(t), robot x(t)
    dict(n="1A", t0=0, t1=5, set="street", pos=((-22, -26, 42), (-9, -17, 19)), tgt=((3, 0, 0), (3, 0, 0.5)), lens=(24, 24), robot=None),
    dict(n="1B", t0=5, t1=9, set="street", pos=((-6.5, -5.2, 0.75), (-3.0, -4.0, 0.6)), tgt=((3.0, 1.0, 0.25), (4.0, 0.5, 0.3)), lens=(35, 35), robot=None),
    dict(n="1C", t0=9, t1=12, set="street", pos=((1.3, -2.1, 1.6), (1.6, -1.9, 1.6)), tgt=((7.2, 0, 1.2), (7.2, 0, 1.2)), lens=(50, 50), robot=None),
    dict(n="2A", t0=12, t1=16, set="studio", orb=((0.85, 0, 0.7), 4.3, -120, -20, 1.5, 2.4), tgt=((0.85, 0, 0.75), (0.85, 0, 0.8)), lens=(50, 50), robot="closed", belt=0.0),
    dict(n="2B", t0=16, t1=19, set="studio", pos=((2.0, -3.6, 0.4), (0.0, -3.6, 0.42)), tgt=((1.6, -1.0, 0.32), (0.4, -1.0, 0.34)), lens=(70, 70), robot="closed", belt=0.35),
    dict(n="2C", t0=19, t1=25, set="street", pos=((-4.2, -4.6, 1.2), (1.2, -4.3, 1.35)), tgt="robot", lens=(35, 35), robot="mech"),
    dict(n="3A", t0=25, t1=28, set="street", pos=((-1.6, -3.6, 1.4), (-1.0, -2.6, 3.1)), tgt=((0.9, 0, 1.0), (0.9, 0, 1.15)), lens=(35, 35), robot="mech"),
    dict(n="3B", t0=28, t1=32, set="street", pos=((-3.3, -2.5, 1.8), (-3.1, -2.9, 1.7)), tgt=((-0.4, 0, 0.75), (-0.4, 0, 0.65)), lens=(50, 50), robot="mech"),
    dict(n="3C", t0=32, t1=37, set="street", pos=((1.8, -1.05, 1.02), (3.7, -4.4, 2.3)), tgt=((1.55, -0.52, 0.93), (1.6, 0, 1.6)), lens=(100, 35), robot="mech"),
    dict(n="3D", t0=37, t1=41, set="street", pos=((5.6, -4.6, 0.55), (5.3, -4.9, 0.6)), tgt=((1.9, 0, 2.0), (2.0, 0, 1.9)), lens=(28, 28), robot="mech"),
    dict(n="3E", t0=41, t1=44, set="street", pos=((2.4, -6.6, 1.7), (2.6, -6.4, 1.6)), tgt=((2.3, 0, 1.0), (2.4, 0, 0.9)), lens=(35, 35), robot="mech"),
    dict(n="3F", t0=44, t1=46, set="street", pos=((6.6, -3.9, 1.9), (6.4, -4.1, 1.8)), tgt=((4.4, 0, 0.6), (4.5, 0, 0.5)), lens=(35, 35), robot="mech"),
    dict(n="3G", t0=46, t1=48, set="street", pos=((5.3, -2.5, 0.38), (5.2, -2.3, 0.4)), tgt=((4.08, -0.35, 0.4), (4.08, -0.35, 0.4)), lens=(35, 35), robot="mech"),
    dict(n="3H", t0=48, t1=50, set="street", pos=((-0.4, -1.7, 1.75), (-0.2, -1.6, 1.7)), tgt=((3.0, 0, 1.3), (3.0, 0, 1.3)), lens=(35, 35), robot="mech"),
    dict(n="4A", t0=50, t1=56, set="tech", pos=((0.85, -9.5, 1.0), (0.85, -9.0, 1.0)), tgt=((0.85, 0, 0.9), (0.85, 0, 0.9)), lens=(85, 85), robot="closed", dims="closed"),
    dict(n="4B", t0=56, t1=65, set="tech", pos=((2.12, -13.5, 1.3), (2.12, -12.8, 1.3)), tgt=((2.12, 0, 0.95), (2.12, 0, 0.95)), lens=(50, 50), robot="open", dims="open"),
    dict(n="5A", t0=65, t1=70, set="street", pos=((5.2, -9.2, 7.5), (4.6, -8.4, 6.4)), tgt=((2.0, 0, 0.6), (2.0, 0, 0.7)), lens=(28, 28), robot="open"),
    dict(n="5B", t0=70, t1=78, set="street", pos=((4.8, -2.6, 2.3), (1.0, -2.6, 2.3)), tgt=((4.4, 0, 1.3), (0.7, 0, 1.3)), lens=(35, 35), robot="open"),
    dict(n="5C", t0=78, t1=82, set="street", pos=((1.6, -1.9, 1.35), (1.3, -1.9, 1.35)), tgt=((0.6, 0, 1.35), (0.3, 0, 1.35)), lens=(50, 50), robot="open"),
    dict(n="5D", t0=82, t1=85, set="street", pos=((-3.6, -3.1, 1.6), (-3.8, -2.9, 1.6)), tgt=((-1.2, 0, 0.6), (-1.6, 0, 0.6)), lens=(35, 35), robot="open"),
    dict(n="6A", t0=85, t1=88, set="street", pos=((2.6, -1.4, 1.3), (2.5, -1.5, 1.35)), tgt=((2.3, -0.4, 1.15), (2.3, -0.4, 1.1)), lens=(50, 50), robot="mech"),
    dict(n="6B", t0=88, t1=91, set="street", pos=((5.1, -2.7, 0.42), (5.0, -2.6, 0.45)), tgt=((4.08, -0.35, 0.42), (4.08, -0.35, 0.5)), lens=(35, 35), robot="mech"),
    dict(n="6C", t0=91, t1=94, set="street", pos=((2.2, -7.8, 1.5), (2.1, -7.6, 1.6)), tgt=((2.2, 0, 0.8), (2.0, 0, 0.9)), lens=(35, 35), robot="mech"),
    dict(n="6D", t0=94, t1=100, set="street", pos=((1.8, -1.05, 1.02), (4.2, -6.2, 2.7)), tgt=((1.55, -0.52, 0.93), (0.9, 0, 1.3)), lens=(100, 35), robot="mech"),
    dict(n="7A", t0=100, t1=106, set="studio", orb=((0.85, 0, 0.7), 4.6, 20, 200, 1.9, 1.7), tgt=((0.85, 0, 0.8), (0.85, 0, 0.8)), lens=(50, 50), robot="closed", belt=0.0),
    dict(n="7B", t0=106, t1=111, set="studio2", pos=((2.6, -10.5, 2.4), (2.4, -10.0, 2.3)), tgt=((2.1, 1.3, 0.8), (2.1, 1.3, 0.8)), lens=(35, 35), robot="open"),
    dict(n="7C", t0=111, t1=115, set="black", robot=None),
]
DURATION = 115.0


def shot_at(t):
    for s in SHOTS:
        if s["t0"] <= t < s["t1"]:
            return s
    return SHOTS[-1]


# ------------------------------------------------------------------ scene construction
def coll(name):
    c = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(c)
    return c


def move_to(objs, c):
    for o in objs:
        for uc in list(o.users_collection):
            uc.objects.unlink(o)
        c.objects.link(o)


def build_street():
    before = set(bpy.data.objects)
    S.world_overcast()
    asph, curb = S.mat_asphalt(), S.mat_simple("Curb", (0.33, 0.31, 0.29), 0.8, bump=0.3, scale=25)
    stone = S.mat_simple("Stone", (0.2, 0.18, 0.16), 0.85, bump=0.6, scale=12)
    mud = S.mat_simple("Mud", (0.05, 0.035, 0.02), 0.9)
    S.add_box("OuterA", -250, DRY_X, -250, 250, -0.6, DRY_Z - 0.02, S.mat_simple("OuterGround", (0.05, 0.045, 0.04), 0.9))
    S.add_box("OuterA2", DRY_X, S.CH0 - 0.25, -250, 250, -0.6, -0.02, bpy.data.materials["OuterGround"])
    S.add_box("OuterB", S.CH1 + 0.25, 250, -250, 250, -0.6, -0.02, bpy.data.materials["OuterGround"])
    S.add_box("NearRoadDry", -80, DRY_X, -8, 8, -0.3, DRY_Z, asph)
    S.add_box("NearRoad", 0.0, S.CH0, -8, 8, -0.3, 0.0, asph)
    ramp = S.add_box("Ramp", DRY_X, 0.0, -8, 8, -0.3, 0.0, asph)          # gentle slope out of the water
    me = ramp.data
    for v in me.vertices:
        wx = ramp.matrix_world @ v.co
        if wx.z > -0.1:
            v.co.z = DRY_Z * max(0.0, min(1.0, -wx.x / -DRY_X)) - ramp.location.z
    S.add_box("FarRoad", S.CH1, 80, -8, 8, -0.3, 0.0, asph)
    S.add_box("WallA", S.CH0 - 0.25, S.CH0, -60, 60, -1.6, 0.0, stone)
    S.add_box("WallB", S.CH1, S.CH1 + 0.25, -60, 60, -1.6, 0.0, stone)
    S.add_box("Bed", S.CH0, S.CH1, -60, 60, -1.6, -1.5, mud)
    for side in (-1, 1):            # raised sidewalks (dry), interrupted at the stream
        y0, y1 = sorted((side * 8, side * 10.6))
        S.add_box(f"WalkNear{side}", -80, S.CH0 - 0.25, y0, y1, -0.3, 0.2, curb)
        S.add_box(f"WalkFar{side}", S.CH1 + 0.25, 80, y0, y1, -0.3, 0.2, curb)
    S.add_box("Plaza", 5.36, 8.0, -2.4, 2.4, -0.3, 0.2, curb)       # stranded people wait here (dry)
    # flood water: one surface over road and stream (deep part darker through the transparent water)
    bpy.ops.mesh.primitive_plane_add(size=1, location=(0, 0, FLOOD_Z))
    w = bpy.context.active_object
    w.name = "Flood"
    w.scale = (500, 500, 1)
    bpy.ops.object.transform_apply(scale=True)
    oc = w.modifiers.new("Ocean", "OCEAN")
    oc.geometry_mode = "DISPLACE"; oc.resolution = 16; oc.spatial_size = 10; oc.repeat_x = 50; oc.repeat_y = 50; oc.wave_scale = 0.11
    oc.choppiness = 0.9; oc.wind_velocity = 6.0; oc.wave_direction = math.radians(90); oc.damping = 0.4
    m, nt, b = S.node_mat("FloodWater")
    S.set_in(b, "Base Color", (0.13, 0.085, 0.045, 1)); S.set_in(b, "Roughness", 0.16); S.set_in(b, "IOR", 1.33)
    S.set_in(b, ["Transmission Weight", "Transmission"], 0.3)
    S.noise_bump(nt, b, scale=9.0, strength=0.25, detail=6.0)
    vol = nt.nodes.new("ShaderNodeVolumeAbsorption")
    vol.inputs["Color"].default_value = (0.45, 0.3, 0.15, 1); vol.inputs["Density"].default_value = 3.0
    nt.links.new(vol.outputs[0], nt.nodes["Material Output"].inputs["Volume"])
    w.data.materials.append(m)
    bpy.ops.object.shade_smooth()
    # buildings
    for side in (-1, 1):
        x, i = -34.0, 0
        while x < 40.0:
            wdt, h = 5 + (i * 1.37 % 3.8), 7 + (i * 2.71 % 9)
            if x + wdt > S.CH0 - 0.8 and x < S.CH1 + 0.8:
                x = S.CH1 + 1.0
                continue
            y0 = side * 10.6
            S.add_box(f"Bld_{side}_{i}", x, x + wdt, min(y0, y0 + side * 9), max(y0, y0 + side * 9), 0, h, S.mat_facade(i + (side > 0)))
            x += wdt + 0.35
            i += 1
    S.add_box("FarBuilding", 8.0, 17.0, -8, 8, 0, 11, S.mat_facade(7))
    S.add_box("Door", 7.97, 8.0, -0.75, 0.75, 0.2, 2.5, S.mat_simple("DoorDark", (0.015, 0.015, 0.02), 0.35))
    S.add_box("Canopy", 7.55, 8.0, -1.2, 1.2, 2.55, 2.65, S.mat_simple("Concrete", (0.35, 0.33, 0.3), 0.8))
    # rescue truck on the safe (near) side + a flooded car on the far side
    white = S.mat_simple("TruckWhite", (0.8, 0.8, 0.8), 0.35)
    orng = S.mat_simple("TruckOrange", (0.85, 0.2, 0.02), 0.35)
    dark = S.mat_simple("Tyre", (0.02, 0.02, 0.02), 0.8)
    S.add_box("TruckBox", -12.2, -6.0, 4.4, 6.8, 0.6, 3.2, white)
    S.add_box("TruckStripe", -12.21, -5.99, 4.39, 6.81, 1.4, 1.7, orng)
    S.add_box("TruckCab", -6.0, -4.2, 4.4, 6.8, 0.6, 2.6, white)
    S.add_box("TruckWind", -4.21, -4.19, 4.6, 6.6, 1.6, 2.4, S.mat_simple("Glass", (0.03, 0.04, 0.05), 0.05))
    beacon = S.mat_simple("Beacon", (0.1, 0.3, 1.0), 0.2)
    S.set_in(beacon.node_tree.nodes["Principled BSDF"], ["Emission Color", "Emission"], (0.1, 0.35, 1.0, 1))
    S.set_in(beacon.node_tree.nodes["Principled BSDF"], "Emission Strength", 25.0)
    S.add_box("Beacon", -5.4, -4.6, 5.2, 6.0, 2.6, 2.75, beacon)
    for (x, y) in ((-11, 4.35), (-11, 6.85), (-5.2, 4.35), (-5.2, 6.85)):
        bpy.ops.mesh.primitive_cylinder_add(radius=0.48, depth=0.3, location=(x, y, 0.48), rotation=(math.pi / 2, 0, 0))
        bpy.context.active_object.data.materials.append(dark)
    car = S.mat_simple("CarRed", (0.35, 0.03, 0.02), 0.25)
    S.add_box("Car", 6.0, 10.3, -7.6, -5.8, 0.15, 0.95, car)
    S.add_box("CarTop", 6.9, 9.3, -7.5, -5.9, 0.95, 1.5, S.mat_simple("CarGlass", (0.02, 0.025, 0.03), 0.05))
    for o in bpy.data.objects:
        if o.name.startswith(("Car", "CarTop")):
            o.rotation_euler.z = 0.2
    # warning barriers at the broken road edge
    bar = S.mat_simple("Barrier", (0.75, 0.75, 0.73), 0.5)
    red = S.mat_simple("BarrierRed", (0.6, 0.04, 0.02), 0.45)
    for y in (-6.5, -4.5, 4.5, 6.5):
        S.add_box(f"Bar{y}", S.CH0 - 0.75, S.CH0 - 0.6, y - 0.9, y + 0.9, 0, 0.9, bar)
        S.add_box(f"BarR{y}", S.CH0 - 0.76, S.CH0 - 0.59, y - 0.91, y + 0.91, 0.55, 0.72, red)
    objs = [o for o in bpy.data.objects if o not in before and o.type != "LIGHT"]
    c = coll("STREET")
    move_to(objs, c)
    return c, w


def build_studio():
    before = set(bpy.data.objects)
    floor = S.mat_simple("StudioFloor", (0.05, 0.05, 0.055), 0.32)
    bpy.ops.mesh.primitive_plane_add(size=80, location=(0, 0, 0))
    bpy.context.active_object.data.materials.append(floor)
    bpy.context.active_object.name = "StudioFloor"
    # curved cyclorama behind (quarter cylinder + wall)
    bpy.ops.mesh.primitive_cylinder_add(vertices=96, radius=26, depth=80, location=(1.0, 0, 26), rotation=(0, math.pi / 2, 0))
    cyc = bpy.context.active_object
    cyc.name = "Cyclorama"
    cyc.data.materials.append(S.mat_simple("Cyc", (0.06, 0.06, 0.065), 0.8))
    bpy.ops.object.shade_smooth()
    for i in range(len(cyc.data.polygons)):
        cyc.data.polygons[i].use_smooth = True
    # flip normals so the inside is lit
    bpy.context.view_layer.objects.active = cyc
    bpy.ops.object.mode_set(mode="EDIT"); bpy.ops.mesh.select_all(action="SELECT"); bpy.ops.mesh.flip_normals(); bpy.ops.object.mode_set(mode="OBJECT")
    lights = []
    for name, loc, rot, size, energy, col in (
            ("Softbox", (0.85, 0, 6.0), (0, 0, 0), (6.0, 3.0), 2200, (1, 1, 1)),
            ("Rim", (-3.5, 3.5, 2.2), (math.radians(70), 0, math.radians(-135)), (2.0, 1.0), 650, (1.0, 0.55, 0.25)),
            ("Fill", (4.5, -4.0, 1.8), (math.radians(70), 0, math.radians(48)), (3.0, 2.0), 380, (0.85, 0.9, 1.0))):
        ld = bpy.data.lights.new(name, "AREA")
        ld.shape = "RECTANGLE"; ld.size, ld.size_y = size; ld.energy = energy; ld.color = col
        lo = bpy.data.objects.new(name, ld)
        lo.location = loc
        lo.rotation_euler = rot
        bpy.context.scene.collection.objects.link(lo)
        lights.append(lo)
    objs = [o for o in bpy.data.objects if o not in before]
    c = coll("STUDIO")
    move_to(objs, c)
    return c


_VIS = {}


def set_visible(c, on):
    if _VIS.get(c.name) == on:
        return
    _VIS[c.name] = on
    for o in c.all_objects:
        o.hide_render = not on


# ------------------------------------------------------------------ people
def ellipsoid(name, r, at, mat, parent):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=14, radius=1, location=(0, 0, 0))
    o = bpy.context.active_object
    o.name = name
    o.scale = r
    bpy.ops.object.transform_apply(scale=True)
    bpy.ops.object.shade_smooth()
    o.data.materials.append(mat)
    o.parent = parent
    o.location = at
    return o


def empty(name, at, parent=None):
    e = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(e)
    e.parent = parent
    e.location = at
    return e


def fabric(name, c, sheen=0.4):
    m, nt, b = S.node_mat(name)
    S.set_in(b, "Base Color", (*c, 1)); S.set_in(b, "Roughness", 0.75)
    S.set_in(b, ["Sheen Weight", "Sheen"], sheen)
    S.noise_bump(nt, b, scale=600, strength=0.15)
    return m


def skin_mat(name, c):
    m, nt, b = S.node_mat(name)
    S.set_in(b, "Base Color", (*c, 1)); S.set_in(b, "Roughness", 0.5)
    S.set_in(b, ["Subsurface Weight", "Subsurface"], 0.15)
    return m


def human(tag, h, jacket, pants, skin, hair, vest=False):
    k = h / 1.75
    root = empty(f"P_{tag}", (0, 0, 0))
    body = empty(f"PB_{tag}", (0, 0, 0), root)
    body.scale = (k, k, k)
    mJ, mP, mS, mH = fabric(f"J{tag}", jacket), fabric(f"T{tag}", pants), skin_mat(f"S{tag}", skin), fabric(f"H{tag}", hair, 0.1)
    boots = S.mat_simple(f"B{tag}", (0.02, 0.02, 0.02), 0.5)
    parts = {}
    for side, y in (("L", 0.095), ("R", -0.095)):
        hip = empty(f"hip{side}_{tag}", (0, y, 0.9), body)
        ellipsoid(f"thigh{side}_{tag}", (0.078, 0.078, 0.235), (0, 0, -0.21), mP, hip)
        knee = empty(f"knee{side}_{tag}", (0, 0, -0.42), hip)
        ellipsoid(f"shin{side}_{tag}", (0.06, 0.06, 0.22), (0, 0, -0.2), mP, knee)
        ellipsoid(f"foot{side}_{tag}", (0.13, 0.05, 0.045), (0.05, 0, -0.42), boots, knee)
        sh = empty(f"sh{side}_{tag}", (0, y * 2.25, 1.42), body)
        ellipsoid(f"uarm{side}_{tag}", (0.05, 0.05, 0.16), (0, 0, -0.14), mJ, sh)
        el = empty(f"el{side}_{tag}", (0, 0, -0.29), sh)
        ellipsoid(f"farm{side}_{tag}", (0.043, 0.043, 0.15), (0, 0, -0.13), mJ, el)
        ellipsoid(f"hand{side}_{tag}", (0.045, 0.03, 0.06), (0, 0, -0.29), mS, el)
        parts[side] = dict(hip=hip, knee=knee, sh=sh, el=el)
    ellipsoid(f"pelvis_{tag}", (0.12, 0.17, 0.11), (0, 0, 0.95), mP, body)
    ellipsoid(f"torso_{tag}", (0.13, 0.2, 0.27), (0, 0, 1.2), mJ, body)
    ellipsoid(f"neck_{tag}", (0.05, 0.05, 0.06), (0, 0, 1.49), mS, body)
    ellipsoid(f"head_{tag}", (0.095, 0.08, 0.115), (0.01, 0, 1.6), mS, body)
    ellipsoid(f"hair_{tag}", (0.1, 0.085, 0.07), (-0.008, 0, 1.66), mH, body)
    if vest:
        ref = S.mat_simple(f"V{tag}", (0.9, 1.0, 0.1), 0.3)
        S.set_in(ref.node_tree.nodes["Principled BSDF"], ["Emission Color", "Emission"], (0.6, 0.7, 0.05, 1))
        S.set_in(ref.node_tree.nodes["Principled BSDF"], "Emission Strength", 0.6)
        ellipsoid(f"vest_{tag}", (0.135, 0.205, 0.035), (0, 0, 1.12), ref, body)
    root["parts"] = tag
    return root, parts


PEOPLE_SPEC = [("a", 1.75, (0.03, 0.12, 0.35), (0.02, 0.02, 0.025), (0.55, 0.36, 0.24), (0.03, 0.02, 0.015), True),
               ("b", 1.12, (0.6, 0.06, 0.03), (0.05, 0.09, 0.2), (0.62, 0.42, 0.3), (0.12, 0.06, 0.02), False),
               ("c", 1.66, (0.16, 0.2, 0.07), (0.03, 0.035, 0.04), (0.45, 0.28, 0.18), (0.015, 0.01, 0.01), False),
               ("d", 1.71, (0.25, 0.07, 0.2), (0.06, 0.065, 0.07), (0.6, 0.45, 0.33), (0.4, 0.4, 0.38), False)]
WAIT = [(7.0, 0.4, 0.2), (6.9, -0.2, 0.2), (7.35, -0.7, 0.2), (7.45, 0.7, 0.2)]
ROUTE = [(5.55, 0, 0.2), (5.33, 0, 0.2), (4.3, 0, 0.9), (-0.04, 0, 0.9), (-0.06, 0, 0.72), (-0.2, 0, 0.72), (-0.24, 0, 0.54),
         (-0.4, 0, 0.54), (-0.44, 0, 0.36), (-0.6, 0, 0.36), (-0.64, 0, 0.18), (-0.8, 0, 0.18), (-0.86, 0, 0.1), (-3.0, 2.5, 0.1),
         (-3.6, 6.5, 0.1), (-3.8, 8.3, 0.2)]
ORDER = ["b", "a", "d", "c"]                # child first, rescuer right behind
WALK_T0 = 66.0


class People:
    def __init__(self):
        self.p = []
        for i, spec in enumerate(PEOPLE_SPEC):
            root, parts = human(*spec)
            pts = [Vector(WAIT[i])] + [Vector(q) for q in ROUTE]
            pts[-1] = Vector((-3.8 - 0.7 * i, 8.4 + 0.5 * (i % 2), 0.2))
            segl = [(pts[j + 1] - pts[j]).length for j in range(len(pts) - 1)]
            self.p.append(dict(tag=spec[0], root=root, parts=parts, pts=pts, seg=segl, total=sum(segl), h=spec[1]))
        objs = [o for o in bpy.data.objects if o.name.startswith(("P_", "PB_", "hip", "knee", "sh", "el", "thigh", "shin", "foot",
                                                                  "uarm", "farm", "hand", "pelvis", "torso", "neck", "head", "hair", "vest"))]
        self.c = coll("PEOPLE")
        move_to(objs, self.c)

    def along(self, P, d):
        d = max(0.0, min(P["total"], d))
        for j, L in enumerate(P["seg"]):
            if d <= L:
                return P["pts"][j].lerp(P["pts"][j + 1], d / L if L else 0), P["pts"][j + 1] - P["pts"][j]
            d -= L
        return P["pts"][-1].copy(), P["pts"][-1] - P["pts"][-2]

    def update(self, t):
        for P in self.p:
            k = ORDER.index(P["tag"])
            spd = 0.95 if P["tag"] == "b" else 1.08
            d = (t - WALK_T0 - k * 1.5) * spd
            pos, dirv = self.along(P, d)
            P["root"].location = pos
            u = P["parts"]
            walking = 0 < d < P["total"]
            if d <= 0:
                P["root"].rotation_euler = (0, 0, math.pi)
                wv = math.sin(t * 5 + k * 1.3)
                for s, sg in (("L", 1), ("R", -1)):
                    u[s]["hip"].rotation_euler = (0, 0, 0); u[s]["knee"].rotation_euler = (0, 0, 0)
                raise_l = k % 2 == 1
                u["R"]["sh"].rotation_euler = (-2.5, 0, 0); u["R"]["el"].rotation_euler = (-0.4 - 0.35 * wv, 0, 0)
                u["L"]["sh"].rotation_euler = ((2.3 if raise_l else 0.1), 0, 0); u["L"]["el"].rotation_euler = ((0.3 + 0.3 * wv) if raise_l else 0, 0, 0)
                continue
            P["root"].rotation_euler = (0, 0, math.atan2(dirv.y, dirv.x))
            stride = 0.62 * P["h"] / 1.75
            ph = d / stride * math.pi
            s, c = (math.sin(ph), math.cos(ph)) if walking else (0.0, 0.0)
            climb = dirv.z / max(0.01, math.hypot(dirv.x, dirv.y))
            lift = 0.3 * max(0.0, climb)
            u["L"]["hip"].rotation_euler = (0, -0.45 * s - lift, 0); u["R"]["hip"].rotation_euler = (0, 0.45 * s - lift, 0)
            u["L"]["knee"].rotation_euler = (0, 0.6 * max(0.0, c) + lift, 0); u["R"]["knee"].rotation_euler = (0, 0.6 * max(0.0, -c) + lift, 0)
            u["L"]["sh"].rotation_euler = (0.08, 0.35 * s, 0); u["R"]["sh"].rotation_euler = (-0.08, -0.35 * s, 0)
            u["L"]["el"].rotation_euler = (0, -0.35, 0); u["R"]["el"].rotation_euler = (0, -0.35, 0)
            P["root"].location.z += 0.012 * abs(c) if walking else 0


# ------------------------------------------------------------------ rain
class Rain:
    def __init__(self, n=2600):
        self.n = n
        rng = np.random.default_rng(3)
        self.seed = np.c_[rng.uniform(-9, 9, n), rng.uniform(-9, 9, n), rng.uniform(0, 7, n), rng.uniform(0.85, 1.15, n)]
        me = bpy.data.meshes.new("Rain")
        verts = np.zeros((n * 4, 3), np.float32)
        faces = np.arange(n * 4).reshape(n, 4)
        me.from_pydata(verts.tolist(), [], faces.tolist())
        self.ob = bpy.data.objects.new("Rain", me)
        m, nt, b = S.node_mat("RainMat")
        S.set_in(b, "Base Color", (0.85, 0.88, 0.92, 1)); S.set_in(b, "Roughness", 0.1); S.set_in(b, "Alpha", 0.22)
        me.materials.append(m)
        self.ob.visible_shadow = False
        self.c = coll("RAIN")
        self.c.objects.link(self.ob)

    def update(self, t, cam):
        cp = cam.location
        fwd = cam.matrix_world.to_3x3() @ Vector((0, 0, -1))
        side = Vector((-fwd.y, fwd.x, 0)).normalized() if abs(fwd.z) < 0.99 else Vector((1, 0, 0))
        x = cp.x + fwd.x * 6 + self.seed[:, 0]
        y = cp.y + fwd.y * 6 + self.seed[:, 1]
        z = (self.seed[:, 2] - t * 9.0 * self.seed[:, 3]) % 7.0 - 0.5 + max(0.0, cp.z - 3.5)
        L, Wd = 0.32, 0.006
        v = np.zeros((self.n, 4, 3), np.float32)
        for j, (a, bb) in enumerate(((-1, 0), (1, 0), (1, 1), (-1, 1))):
            v[:, j, 0] = x + side.x * Wd * a - 0.06 * bb
            v[:, j, 1] = y + side.y * Wd * a
            v[:, j, 2] = z + L * bb
        self.ob.data.vertices.foreach_set("co", v.reshape(-1))
        self.ob.data.update()


# ------------------------------------------------------------------ main
class Film:
    def __init__(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        S.render_settings(1280, 720, 12)
        bpy.context.scene.render.use_persistent_data = os.environ.get("AKORA_PERSIST", "0") == "1"
        self.street, self.water = build_street()
        self.street_world = bpy.context.scene.world
        self.sun = bpy.data.objects["Sun"]
        self.studio = build_studio()
        self.robot = S.Robot()
        self.robot2 = None
        self.people = People()
        self.rain = Rain()
        self.cam = S.camera(lens=35)
        sw = bpy.data.worlds.new("StudioWorld"); sw.use_nodes = True
        sw.node_tree.nodes["Background"].inputs[0].default_value = (0.012, 0.012, 0.014, 1)
        tw = bpy.data.worlds.new("TechWorld"); tw.use_nodes = True
        tw.node_tree.nodes["Background"].inputs[0].default_value = (0.55, 0.57, 0.6, 1)
        tw.node_tree.nodes["Background"].inputs[1].default_value = 0.9
        self.worlds = dict(street=self.street_world, studio=sw, studio2=sw, tech=tw, black=sw)
        self.overlay = {}

    def robot_objs(self, r):
        for lst in r.nodes.values():
            yield from lst
        for lst in r.link_objs.values():
            yield from lst

    def show_robot(self, r, on):
        if getattr(r, "_shown", None) == on:
            return
        r._shown = on
        for o in self.robot_objs(r):
            o.hide_render = not on

    @staticmethod
    def ground(xw):
        return DRY_Z * max(0.0, min(1.0, xw / DRY_X))

    def set_state(self, r, st, x=0.0, y=0.0, belt=0.0, street=False):
        F = kin_frames(st)
        zr, zf = (self.ground(x + 0.22), self.ground(x + 1.48)) if street else (0.0, 0.0)
        pitch = math.atan2(zf - zr, 1.26)
        R = (Matrix.Translation((x * 1000.0, y * 1000.0, zr * 1000.0)) @ Matrix.Translation((220, 0, 0))
             @ Matrix.Rotation(-pitch, 4, "Y") @ Matrix.Translation((-220, 0, 0)))
        for key, lst in r.nodes.items():
            M = R @ Matrix(F[key].tolist())
            for o in lst:
                o.matrix_basis = M
        from build_anim import link_frames
        lfs, _ = link_frames(belt * 1000.0 - x * 1000.0)
        for side, lst in r.link_objs.items():
            for j, ob in enumerate(lst):
                ob.matrix_basis = R @ Matrix(lfs[j].tolist())

    def frame(self, i):
        t = i / FPS
        s = shot_at(t)
        u = (t - s["t0"]) / (s["t1"] - s["t0"])
        ue = ease(u)
        bpy.context.scene.world = self.worlds[s["set"]]
        street = s["set"] == "street"
        set_visible(self.street, street)
        set_visible(self.rain.c, street)
        set_visible(self.people.c, street and t >= 9.0)
        set_visible(self.studio, s["set"] in ("studio", "studio2", "tech"))
        self.sun.hide_render = not street
        floor = bpy.data.objects["StudioFloor"]
        floor.data.materials[0].node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = \
            (0.5, 0.52, 0.55, 1) if s["set"] == "tech" else (0.05, 0.05, 0.055, 1)
        bpy.data.objects["Cyclorama"].hide_render = s["set"] == "tech" or not s["set"].startswith(("studio",))
        rob = s.get("robot")
        self.show_robot(self.robot, rob is not None)
        x = 0.0
        if rob == "closed":
            st = CLOSED
        elif rob == "open":
            st = OPEN
        elif rob == "mech":
            st = mech_state(t)
            x = drive_x(t)
        else:
            st = CLOSED
        belt = s.get("belt", 0.0) * (t - s["t0"])
        self.set_state(self.robot, st, x=x, belt=belt, street=(s["set"] == "street"))
        if s["set"] == "studio2":
            if self.robot2 is None:
                self.robot2 = S.Robot()
            self.show_robot(self.robot2, True)
            self.set_state(self.robot2, CLOSED, x=0.0, y=2.6)
        elif self.robot2 is not None:
            self.show_robot(self.robot2, False)
        if s["set"] == "black":
            return None
        # camera
        if "orb" in s:
            c, r, a0, a1, z0, z1 = s["orb"]
            pos = orbit(c, r, a0, a1, z0, z1)(ue)
        else:
            pos = lerp3(s["pos"][0], s["pos"][1], ue)
        if s["tgt"] == "robot":
            tgt = (x + 0.85, 0, 0.6)
        else:
            tgt = lerp3(s["tgt"][0], s["tgt"][1], ue)
        self.cam.data.lens = s["lens"][0] + (s["lens"][1] - s["lens"][0]) * ue
        self.cam.data.dof.use_dof = self.cam.data.lens >= 50
        self.cam.data.dof.focus_distance = (Vector(tgt) - Vector(pos)).length
        self.cam.data.dof.aperture_fstop = 4.0
        S.look_at(self.cam, pos, tgt)
        bpy.context.view_layer.update()
        if street:
            S.set_ocean_time(self.water, t)
            self.people.update(t)
            self.rain.update(t, self.cam)
        if s.get("dims"):
            self.overlay[i] = self.dims(s["dims"])
        return s

    def dims(self, kind):
        sc, cam = bpy.context.scene, self.cam
        P = lambda p: tuple(round(float(v), 4) for v in world_to_camera_view(sc, cam, Vector(p))[:2])
        if kind == "closed":
            return [dict(a=P((0.0, 0, -0.08)), b=P((1.755, 0, -0.08)), label="170 cm gövde (≈176 cm toplam)"),
                    dict(a=P((1.95, 0, 0.0)), b=P((1.95, 0, 1.80)), label="≈180 cm")]
        return [dict(a=P((-0.94, 0, -0.08)), b=P((5.18, 0, -0.08)), label="600 cm açılım"),
                dict(a=P((2.4, -0.5, 0.0)), b=P((2.4, -0.5, 0.9)), label="90 cm yürüyüş"),
                dict(a=P((3.5, -0.5, 0.9)), b=P((3.5, -0.5, 1.9)), label="100 cm korkuluk")]


def main():
    os.makedirs(FRAMES, exist_ok=True)
    mode = sys.argv[1] if len(sys.argv) > 1 else "stills"
    f = Film()
    sc = bpy.context.scene
    if mode == "stills":
        os.makedirs(os.path.join(OUT, "stills"), exist_ok=True)
        only = sys.argv[2].split(",") if len(sys.argv) > 2 else None
        for s in SHOTS:
            if only and s["n"] not in only:
                continue
            t = s["t0"] + 0.5 * (s["t1"] - s["t0"])
            i = int(round(t * FPS))
            if f.frame(i) is None:
                continue
            sc.render.filepath = os.path.join(OUT, "stills", f"{s['n']}.jpg")
            bpy.ops.render.render(write_still=True)
            print("STILL", s["n"], flush=True)
        return
    a, b = int(sys.argv[2]), int(sys.argv[3])
    ov_path = os.path.join(OUT, f"overlay_{a}_{b}.json")
    for i in range(a, b):
        out = os.path.join(FRAMES, f"f_{i:05d}.jpg")
        s = f.frame(i)
        if s is None or os.path.exists(out):
            continue
        sc.render.filepath = out
        bpy.ops.render.render(write_still=True)
        print("FRAME", i, flush=True)
    with open(ov_path, "w") as fh:
        json.dump({str(k): v for k, v in f.overlay.items()}, fh)


if __name__ == "__main__":
    main()
