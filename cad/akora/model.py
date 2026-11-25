"""AKORA 1:1 parametric CAD model (units: mm, X forward, Y left, Z up, ground z=0).

Every part is built ONCE in a local frame and placed by a kinematic frame, so all
configurations (closed, opening stages, deployed, exploded) share identical geometry.
"""
import math
from dataclasses import dataclass, field

import cadquery as cq
import numpy as np

from . import params as P
from .geom import (R, T, box, chain, circles_hull, cyl, lerp, poly_extrude, rack,
                   rbox, rect_tube, ring_xz, rounded_rect_points, rounded_rect_ring,
                   spur_gear, sprocket, tube, tube_between)


# ====================================================================== part record
@dataclass
class Part:
    name: str
    layer: tuple            # e.g. ("01_MAIN_BODY", "TOP_COVER")
    mat: str
    shape: object           # cq.Shape (local frame)
    frame: str = "world"
    M: np.ndarray = field(default_factory=lambda: np.eye(4))
    group: str = "body"     # explode group


def S(wp):
    vals = wp.vals() if isinstance(wp, cq.Workplane) else [wp]
    vals = [v for v in vals if isinstance(v, cq.Shape)]
    if len(vals) == 1:
        return vals[0]
    return cq.Compound.makeCompound(vals)


def mirror_y(shape):
    return shape.mirror("XZ")


PARTS = []
PEDESTAL_X = (600, 1380)          # between P1 cross members
PEDESTAL_Y = 380
LEG_HINGE_Z = -160
FRONT_PIVOT_DX = 50                # stair pivot 50 mm beyond the P3 end face
FRONT_START_DX = 30                # stair top edge 30 mm beyond the P3 end face
REAR_TREAD_D = 202                 # rear stair: 4 x 202 + landing offset -> foot 83 cm behind the hull [D 600 total]


def add(name, layer, mat, wp, frame="world", M=None, group="body"):
    p = Part(name, tuple(layer), mat, S(wp) if not isinstance(wp, cq.Shape) else wp,
             frame, np.eye(4) if M is None else M, group)
    PARTS.append(p)
    return p


def polyline_tube(pts, r):
    """Cable / pipe along way-points with spherical joints."""
    w = None
    for a, b in zip(pts[:-1], pts[1:]):
        seg = tube_between(a, b, r)
        w = seg if w is None else w.union(seg)
    for p in pts[1:-1]:
        w = w.union(cq.Workplane("XY").sphere(r).translate(p))
    return w


def hexbolt(d, L, head_h=None):
    """Hex bolt along -Z, head on top at z=0..head_h (local)."""
    s = 1.6 * d
    head_h = head_h or 0.65 * d
    head = cq.Workplane("XY").polygon(6, s / math.cos(math.pi / 6)).extrude(head_h)
    shank = cyl(d / 2, L, "z", (0, 0, -L))
    return head.union(shank)


# ====================================================================== 01 MAIN BODY
def build_body():
    L1 = ("01_MAIN_BODY",)
    x0, x1, yh, r = P.HULL_X0, P.HULL_X1, P.HULL_Y, P.HULL_R
    w = P.HULL_WALL
    z_split = 430
    ring_z0 = P.HULL_TOP - P.RING_DEPTH          # 670

    # --- tub (split lower / upper at a sealed flange line z=430)
    outer = rbox(x0, x1, -yh, yh, P.HULL_Z0, ring_z0, r_vert=r, r_bot=60)
    inner = rbox(x0 + w, x1 - w, -yh + w, yh - w, P.HULL_Z0 + w, ring_z0 + 50, r_vert=r - w, r_bot=52)
    tub = outer.cut(inner)
    # front lamp recesses + rear service opening
    for sy in (1, -1):
        tub = tub.cut(box(x1 - 30, x1 + 5, sy * 330 - 110, sy * 330 + 110, 420, 540))
    tub = tub.cut(box(x0 - 5, x0 + 30, -380, 380, 300, 540))
    lower = tub.intersect(box(0, 1700, -700, 700, 0, z_split))
    upper = tub.intersect(box(0, 1700, -700, 700, z_split, ring_z0))
    # flange bead on the joint line (visible design line, bolt flange inside)
    bead = rounded_rect_ring(x0, x1, -yh, yh, r, -4, 0, z_split - 6, z_split + 6)
    add("BODY_LOWER_TUB", L1 + ("BODY_LOWER",), "polymer_black", lower, group="hull")
    add("BODY_UPPER_TUB", L1 + ("BODY_UPPER",), "polymer_black", upper, group="hull")
    inner_flange = rounded_rect_ring(x0, x1, -yh, yh, r, w, w + 30, z_split - 15, z_split + 15)
    # local interruptions: lamp pods (front), service panel (rear), track gearboxes
    for sy in (1, -1):
        pod_cut = box(x1 - 40, x1 + 10, sy * 330 - 105, sy * 330 + 105, 420, 540)
        bead = bead.cut(pod_cut)
        inner_flange = inner_flange.cut(pod_cut)
        inner_flange = inner_flange.cut(cyl(102, 120, "y", (P.SPROCKET[0], 520 if sy > 0 else -640, P.SPROCKET[1])))
    bead = bead.cut(box(x0 - 15, x0 + 10, -400, 400, 280, 560))
    add("BODY_JOINT_BEAD", L1 + ("BODY_UPPER",), "polymer_dark", bead, group="hull")
    add("BODY_JOINT_FLANGE_INNER", L1 + ("BODY_LOWER",), "polymer_dark", inner_flange, group="hull")

    # --- interlock ring (bonded to tub rim) with continuous seating groove
    ring = rounded_rect_ring(x0, x1, -yh, yh, r, 0, 45, ring_z0, P.HULL_TOP)
    groove = rounded_rect_ring(x0, x1, -yh, yh, r, P.GROOVE_OFF[0], P.GROOVE_OFF[1],
                               P.HULL_TOP - P.GROOVE_DEPTH, P.HULL_TOP + 1)
    ring = ring.cut(groove)
    add("COVER_SEAT_RING_FEMALE", L1 + ("COVER_INTERLOCK",), "polymer_dark", ring, group="hull")

    # --- EPDM gasket cord in the groove
    g_z0 = P.HULL_TOP - P.GROOVE_DEPTH
    gasket = rounded_rect_ring(x0, x1, -yh, yh, r, P.GASKET_OFF[0], P.GASKET_OFF[1], g_z0, g_z0 + P.GASKET_H)
    add("COVER_GASKET_EPDM_10x8", L1 + ("COVER_GASKET",), "epdm", gasket, frame="cover_seal", group="gasket")

    # --- TOP COVER (separate, removable)
    cz0, cz1, cw = P.HULL_TOP, P.COVER_TOP, P.COVER_WALL
    c_out = rbox(x0, x1, -yh, yh, cz0, cz1, r_vert=r, r_top=30)
    c_in = rbox(x0 + cw, x1 - cw, -yh + cw, yh - cw, cz0 - 1, cz1 - cw, r_vert=r - cw, r_top=24)
    cover = c_out.cut(c_in)
    lip = rounded_rect_ring(x0, x1, -yh, yh, r, P.LIP_OFF[0], P.LIP_OFF[1], cz0 - P.LIP_LEN, cz0 + 2)
    cover = cover.union(lip)
    # internal stiffening ribs
    for xr in (420, 830, 1240):
        cover = cover.union(box(xr - 4, xr + 4, -yh + cw, yh - cw, cz1 - 40, cz1 - cw))
    # fastener bosses (quarter-turn) and alignment pin bosses on the inner land
    lock_pts = rounded_rect_points(x0, x1, -yh, yh, r, 36, 10)
    pin_pts = [(x0 + 200, yh - 36), (x1 - 200, yh - 36), (x0 + 200, -yh + 36), (x1 - 200, -yh + 36)]
    for (px, py) in lock_pts:
        cover = cover.union(cyl(15, cz1 - cz0 - 2, "z", (px, py, cz0)))
    for (px, py) in pin_pts:
        cover = cover.union(cyl(14, 30, "z", (px, py, cz0)))
    for (px, py) in lock_pts:
        cover = cover.cut(cyl(5, 200, "z", (px, py, cz0 - 50)))
        cover = cover.cut(cyl(12, 20, "z", (px, py, cz1 - 8)))
    for (px, py) in pin_pts:
        cover = cover.cut(cyl(5.25, 22, "z", (px, py, cz0 - 1)))
    # pedestal pass-throughs for the platform supports and recessed lift handles
    for px in PEDESTAL_X:
        for sy in (1, -1):
            cover = cover.cut(cyl(40, 200, "z", (px, sy * PEDESTAL_Y, cz0)))
    for hx in (200, 1600):
        cover = cover.cut(rbox(hx - 90, hx + 90, -35, 35, cz1 - 28, cz1 + 1, r_vert=20))
    add("TOP_COVER_SHELL", L1 + ("TOP_COVER",), "polymer_black", cover, frame="cover", group="cover")
    for i, hx in enumerate((200, 1600)):
        h = tube_between((hx - 70, 0, cz1 - 14), (hx + 70, 0, cz1 - 14), 8)
        h = h.union(box(hx - 80, hx - 66, -10, 10, cz1 - 28, cz1 - 10)).union(box(hx + 66, hx + 80, -10, 10, cz1 - 28, cz1 - 10))
        add(f"COVER_LIFT_HANDLE_{i+1}", L1 + ("TOP_COVER",), "stainless", h, frame="cover", group="cover")
    # grommets around pedestal pass-throughs
    for px in PEDESTAL_X:
        for sy in (1, -1):
            gr = tube(40, 31, 11, "z", (px, sy * PEDESTAL_Y, cz1 - 10))
            add(f"PEDESTAL_GROMMET_{px}_{'L' if sy > 0 else 'R'}", L1 + ("COVER_GASKET",), "epdm", gr,
                frame="cover", group="cover")

    # quarter-turn fasteners (studs in cover, receptacles under the ring land)
    for i, (px, py) in enumerate(lock_pts):
        stud = cyl(11, 7, "z", (px, py, cz1 - 8)).union(cyl(4.5, cz1 - cz0 + 6, "z", (px, py, cz0 - 14)))
        stud = stud.cut(box(px - 8, px + 8, py - 1.2, py + 1.2, cz1 - 3, cz1 + 1))
        add(f"COVER_QUARTER_TURN_STUD_{i+1:02d}", L1 + ("COVER_LOCKS",), "stainless", stud, frame="cover", group="cover")
        rec = cyl(10, 16, "z", (px, py, ring_z0 - 16)).cut(cyl(4.6, 20, "z", (px, py, ring_z0 - 18)))
        add(f"COVER_RECEPTACLE_{i+1:02d}", L1 + ("COVER_LOCKS",), "stainless", rec, group="hull")
    for i, (px, py) in enumerate(pin_pts):
        pin = cyl(5, 34, "z", (px, py, P.HULL_TOP - 14)).union(cyl(8, 3, "z", (px, py, P.HULL_TOP - 3)))
        add(f"COVER_ALIGN_PIN_{i+1}", ("09_FASTENERS", "PINS"), "stainless", pin, group="hull")

    # --- service panels
    rp = rbox(x0 - 10, x0 - 4, -390, 390, 290, 550, r_vert=0)
    rp = rp.edges("|X").fillet(20)
    add("SERVICE_PANEL_REAR", L1 + ("SERVICE_PANELS",), "polymer_dark", rp, group="panel_rear")
    rpg = box(x0 - 4, x0, -385, 385, 295, 545).cut(box(x0 - 5, x0 + 1, -370, 370, 310, 530))
    add("SERVICE_PANEL_REAR_GASKET", L1 + ("COVER_GASKET",), "epdm", rpg, group="panel_rear")
    for i, (py, pz) in enumerate([(-370, 310), (-370, 530), (0, 310), (0, 530), (370, 310), (370, 530)]):
        b = hexbolt(8, 24).rotate((0, 0, 0), (0, 1, 0), -90).translate((x0 - 10, py, pz))
        add(f"SERVICE_PANEL_SCREW_{i+1}", ("09_FASTENERS", "SCREWS"), "stainless", b, group="panel_rear")
    # rear panel connectors + E-stop
    for i, (py, rr) in enumerate([(-250, 28), (-150, 22), (150, 18)]):
        c = cyl(rr, 50, "x", (x0 - 46, py, 430))
        add(f"REAR_CONNECTOR_{i+1}", ("09_FASTENERS", "CONNECTORS"), "steel_dark", c, group="panel_rear")
    es = cyl(30, 20, "x", (x0 - 30, 260, 430)).union(cyl(22, 25, "x", (x0 - 55, 260, 430))).union(cyl(12, 30, "x", (x0 - 10, 260, 430)))
    add("EMERGENCY_STOP", ("08_POWER_SYSTEM", "POWER_CONNECTIONS"), "red", es, group="panel_rear")

    # front lamp / camera pods (PDF front face: two lamps)
    for sy, tag in ((1, "L"), (-1, "R")):
        pod = rbox(x1 - 28, x1 + 6, sy * 330 - 100, sy * 330 + 100, 425, 535, r_vert=0).edges("|X").fillet(40)
        add(f"FRONT_LAMP_HOUSING_{tag}", ("07_ELECTRONICS", "CAMERA"), "polymer_dark", pod, group="hull")
        lens = cyl(38, 6, "x", (x1 + 6, sy * 330 - 45, 480)).union(cyl(38, 6, "x", (x1 + 6, sy * 330 + 45, 480)))
        add(f"FRONT_LAMP_LENS_{tag}", ("07_ELECTRONICS", "CAMERA"), "led", lens, group="hull")

    # --- internal aluminium frame
    LF = L1 + ("INTERNAL_FRAME",)
    zf = P.HULL_Z0 + w                     # 158
    for sy, tag in ((1, "L"), (-1, "R")):
        add(f"FRAME_RAIL_{tag}", LF, "aluminum", rect_tube(40, 80, 4, 1460, "x", (120, sy * 400, zf + 40)), group="frame")
    for i, xc in enumerate((150, 520, 1000, 1240, 1460)):
        add(f"FRAME_CROSS_{i+1}", LF, "aluminum", rect_tube(40, 40, 3, 760, "y", (xc, -380, zf + 60)), group="frame")
    for sy, tag in ((1, "L"), (-1, "R")):
        # track motor mounting plates
        add(f"TRACK_MOTOR_PLATE_{tag}", LF, "aluminum",
            box(120, 330, sy * 420 if sy > 0 else -440, sy * 440 if sy > 0 else -420, zf + 80, 520), group="frame")
        # side wall brackets
        for xb in (400, 1300):
            add(f"FRAME_WALL_BRACKET_{tag}_{xb}", L1 + ("MOUNTING_BRACKETS",), "aluminum",
                box(xb - 40, xb + 40, min(sy * 420, sy * 621), max(sy * 420, sy * 621), zf + 80, zf + 90), group="frame")

    # --- platform pedestals (P1 support through cover grommets) and quick-release pins
    zu = P.DECK_Z - P.FRAME_H                     # P1 underside 800
    for px in PEDESTAL_X:
        for sy, tag in ((1, "L"), (-1, "R")):
            py = sy * PEDESTAL_Y
            ped = (box(px - 60, px + 60, py - 40, py + 40, zf + 80, zf + 92)
                   .union(tube(30, 24, zu - 8 - zf - 92, "z", (px, py, zf + 92)))
                   .union(box(px - 50, px + 50, py - 40, py + 40, zu - 8, zu)))
            add(f"P1_PEDESTAL_{px}_{tag}", L1 + ("MOUNTING_BRACKETS",), "orange", ped, group="frame")
            qp = cyl(6, 120, "x", (px - 60, py, zu + 20)).union(tube(14, 6.5, 4, "x", (px - 64, py, zu + 20)))
            add(f"P1_QUICK_RELEASE_PIN_{px}_{tag}", ("09_FASTENERS", "PINS"), "stainless", qp, group="frame")

    # rear landing brackets (outside the cover footprint, bolted to the tub rear wall)
    for sy, tag in ((1, "L"), (-1, "R")):
        br = (box(x0 - 14, x0, sy * 400 - 40, sy * 400 + 40, 560, 680)
              .union(box(0, x0 - 14, sy * 400 - 30, sy * 400 + 30, 560, 800)))
        add(f"REAR_LANDING_BRACKET_{tag}", L1 + ("MOUNTING_BRACKETS",), "orange", br, group="hull")

    # cable channels hung on the outside of the frame rails
    for sy, tag in ((1, "L"), (-1, "R")):
        yc = sy * 440
        ch = box(170, 1540, yc - 15, yc + 15, zf + 42, zf + 72).cut(box(169, 1541, yc - 12, yc + 12, zf + 45, zf + 73))
        add(f"CABLE_CHANNEL_{tag}", ("07_ELECTRONICS", "CABLE_CHANNELS"), "polymer_dark", ch, group="internal")


# ====================================================================== 07/08 ELECTRONICS & POWER
def build_internals():
    zt = P.HULL_Z0 + P.HULL_WALL + 80          # top of frame rails = 238
    E, PW = ("07_ELECTRONICS",), ("08_POWER_SYSTEM",)
    # battery tray / housing / module / lid locks
    add("BATTERY_TRAY", PW + ("BATTERY_HOUSING",), "aluminum", box(690, 1180, -400, 400, zt, zt + 6), group="internal")
    hous = rbox(700, 1170, -300, 300, zt + 6, zt + 236, r_vert=15)
    hous = hous.cut(rbox(708, 1162, -292, 292, zt + 14, zt + 260, r_vert=10))
    add("BATTERY_HOUSING", PW + ("BATTERY_HOUSING",), "polymer_dark", hous, group="internal")
    add("BATTERY_HOUSING_LID", PW + ("BATTERY_HOUSING",), "polymer_dark",
        rbox(696, 1174, -304, 304, zt + 236, zt + 246, r_vert=17), group="internal_lid")
    for i in range(4):
        cell = rbox(716 + i * 112, 816 + i * 112, -270, 270, zt + 14, zt + 214, r_vert=6)
        add(f"BATTERY_MODULE_{i+1}", PW + ("BATTERY",), "battery", cell, group="internal")
    for i, xl in enumerate((780, 1090)):
        lk = box(xl - 30, xl + 30, -316, -304, zt + 150, zt + 250).union(box(xl - 30, xl + 30, 304, 316, zt + 150, zt + 250))
        add(f"BATTERY_LID_LATCH_{i+1}", PW + ("BATTERY_LOCK",), "stainless", lk, group="internal_lid")
    add("BATTERY_HANDLE", PW + ("BATTERY_HOUSING",), "stainless",
        tube_between((800, 0, zt + 262), (980, 0, zt + 262), 10).union(box(790, 810, -10, 10, zt + 246, zt + 262))
        .union(box(970, 990, -10, 10, zt + 246, zt + 262)), group="internal_lid")
    # BMS + PDU
    add("BMS_BOARD", PW + ("POWER_MANAGEMENT",), "pcb", box(1170, 1180, -200, 200, zt + 40, zt + 200), group="internal")
    pdu = rbox(720, 1000, 310, 395, zt + 6, zt + 90, r_vert=6)
    add("POWER_DISTRIBUTION_UNIT", PW + ("POWER_MANAGEMENT",), "polymer_dark", pdu, group="internal")
    add("PDU_BOARD", PW + ("POWER_MANAGEMENT",), "pcb", box(730, 990, 318, 387, zt + 90, zt + 93), group="internal")
    # main power / signal cables, routed with explicit way-points
    routes = [
        ("POWER_CABLE_BATTERY_PDU", "red", [(950, 250, zt + 256), (950, 350, zt + 256), (950, 350, zt + 104)]),
        ("POWER_CABLE_PDU_DRIVER_L", "red", [(720, 330, zt + 62), (510, 330, zt + 62), (510, 230, zt + 42), (492, 230, zt + 42)]),
        ("POWER_CABLE_PDU_DRIVER_R", "cable", [(720, 315, zt + 82), (525, 315, zt + 82), (525, -230, zt + 82), (492, -230, zt + 42)]),
        ("SIGNAL_CABLE_PDU_CONTROLLER", "cable", [(1000, 350, zt + 92), (1190, 350, zt + 92), (1190, 200, zt + 92), (1212, 200, zt + 92)]),
    ]
    for nm, m, pts in routes:
        add(nm, PW + ("POWER_CONNECTIONS",), m, polyline_tube(pts, 8), group="internal")
    add("MAIN_FUSE_HOLDER", PW + ("POWER_CONNECTIONS",), "red", box(1020, 1080, 320, 380, zt + 6, zt + 60), group="internal")

    # motor drivers with heat sinks
    for sy, tag in ((1, "L"), (-1, "R")):
        d = rbox(300, 490, sy * 120 if sy > 0 else -330, sy * 330 if sy > 0 else -120, zt, zt + 60, r_vert=5)
        for k in range(8):
            fy = (120 + 12 + k * 25) * sy
            d = d.union(box(305, 485, fy - 2, fy + 2, zt + 60, zt + 95))
        add(f"MOTOR_DRIVER_{tag}", E + ("MOTOR_DRIVERS",), "aluminum", d, group="internal")
    # electronics enclosure + main controller
    add("ELECTRONICS_TRAY", E + ("ELECTRONIC_ENCLOSURES",), "aluminum", box(1200, 1560, -300, 300, zt, zt + 6), group="internal")
    enc = rbox(1210, 1550, -220, 220, zt + 6, zt + 146, r_vert=12)
    enc = enc.cut(rbox(1216, 1544, -214, 214, zt + 12, zt + 160, r_vert=8))
    add("ELECTRONICS_ENCLOSURE", E + ("ELECTRONIC_ENCLOSURES",), "polymer_dark", enc, group="internal")
    add("ELECTRONICS_ENCLOSURE_LID_PC", E + ("ELECTRONIC_ENCLOSURES",), "glass",
        rbox(1208, 1552, -222, 222, zt + 146, zt + 152, r_vert=13), group="internal_lid")
    add("MAIN_CONTROLLER_BOARD", E + ("MAIN_CONTROLLER",), "pcb", box(1240, 1420, -180, 60, zt + 30, zt + 33), group="internal")
    for i, (a, b, c, d2) in enumerate([(1260, 1310, -150, -100), (1330, 1400, -60, 20), (1250, 1290, 0, 40)]):
        add(f"MAIN_CONTROLLER_IC_{i+1}", E + ("MAIN_CONTROLLER",), "polymer_dark", box(a, b, c, d2, zt + 33, zt + 45), group="internal")
    add("IO_BOARD", E + ("MAIN_CONTROLLER",), "pcb", box(1440, 1530, -180, 180, zt + 30, zt + 33), group="internal")
    # receiver + wireless module
    add("RC_RECEIVER", E + ("CONTROL_RECEIVER",), "polymer_dark", rbox(1230, 1340, 228, 292, zt + 6, zt + 50, r_vert=6), group="internal")
    add("WIRELESS_MODULE", E + ("CONTROL_RECEIVER",), "polymer_dark", rbox(1360, 1480, 228, 292, zt + 6, zt + 40, r_vert=6), group="internal")
    for i, sy in enumerate((1, -1)):
        ant = cyl(12, 190, "z", (1600, sy * 600, P.COVER_TOP - 20)).union(cyl(20, 15, "z", (1600, sy * 600, P.COVER_TOP - 25)))
        add(f"ANTENNA_{i+1}", E + ("CONTROL_RECEIVER",), "polymer_dark", ant, frame="cover", group="cover")
    # camera (pan-tilt dome on the cover nose) - stays visible in front of P1
    cx = P.HULL_X1
    add("PTZ_CAMERA_BASE", E + ("CAMERA",), "polymer_dark", cyl(70, 15, "x", (cx, 0, 590)), group="hull")
    add("PTZ_CAMERA_DOME", E + ("CAMERA",), "glass",
        cq.Workplane("XZ").moveTo(0, 0).lineTo(60, 0).threePointArc((60 * 0.7071, 60 * 0.7071), (0, 60)).lineTo(0, 56)
        .threePointArc((56 * 0.7071, 56 * 0.7071), (56, 0)).close().revolve(360, (0, 0, 0), (1, 0, 0))
        .translate((cx + 15, 0, 590)), group="hull")
    add("PTZ_CAMERA_MODULE", E + ("CAMERA",), "polymer_dark",
        cq.Workplane("XY").sphere(25).translate((cx + 40, 0, 590)), group="hull")
    # sensors: ultrasonic front / rear, water level probes
    for i, sy in enumerate((1, -1)):
        add(f"ULTRASONIC_FRONT_{i+1}", E + ("SENSORS",), "steel_dark", cyl(22, 18, "x", (P.HULL_X1 - 4, sy * 120, 330)), group="hull")
        add(f"ULTRASONIC_REAR_{i+1}", E + ("SENSORS",), "steel_dark", cyl(22, 18, "x", (P.HULL_X0 - 14, sy * 470, 330)), group="hull")
    for i, sy in enumerate((1, -1)):
        probe = cyl(6, 140, "z", (P.HULL_X1 - 60, sy * 200, 175)).union(rbox(P.HULL_X1 - 85, P.HULL_X1 - 35, sy * 200 - 25, sy * 200 + 25, 300, 330, r_vert=5))
        add(f"WATER_LEVEL_PROBE_{i+1}", E + ("SENSORS",), "stainless", probe, group="internal")
    add("WATER_PRESSURE_SENSOR", E + ("SENSORS",), "stainless", cyl(15, 40, "x", (P.HULL_X1 - 45, 0, 220)), group="internal")


# ====================================================================== 02 TRACK SYSTEM
def track_path():
    circles = [P.SPROCKET, P.IDLER] + P.ROAD_WHEELS
    return circles


def build_tracks():
    circles = track_path()
    hull0 = circles_hull(circles, 0)
    yb0, yb1 = P.BELT_Y
    for side, sy in (("LEFT_TRACK", 1), ("RIGHT_TRACK", -1)):
        base = ("02_TRACK_SYSTEM", side)
        tag = "L" if sy > 0 else "R"
        grp = f"track_{tag}"

        def put(name, sub, mat, wp):
            shp = S(wp)
            if sy < 0:
                shp = mirror_y(shp)
            add(f"{name}_{tag}", base + (sub,), mat, shp, group=grp)

        # --- frame: orange outer ring plate + inner plate + top beam + spacers + tensioner
        ring = ring_xz(circles_hull(circles, -15), circles_hull(circles, -105), 12, 985)
        put("TRACK_FRAME_OUTER_RING", "TRACK_FRAME", "orange", ring)
        inner_pl = ring_xz(circles_hull(circles, -20), circles_hull([(850, 330, 40)], 0), 12, 650)
        put("TRACK_FRAME_INNER_PLATE", "TRACK_FRAME", "frame_black", inner_pl)
        put("TRACK_FRAME_TOP_BEAM", "TRACK_FRAME", "orange", box(440, 1260, 662, 985, 420, 480))
        for (sx, sz) in ((500, 520), (1200, 520), (685, 100), (1015, 100)):
            put(f"TRACK_SPACER_{sx}_{sz}", "TRACK_FRAME", "steel_dark", cyl(14, 323, "y", (sx, 662, sz)))
        # idler tensioner (slotted block + screw), between inner plate and idler
        tb = box(1380, 1560, 663, 680, 330, 430)
        put("IDLER_TENSIONER_BLOCK", "TRACK_FRAME", "orange", tb)
        put("IDLER_TENSION_SCREW", "TRACK_FRAME", "stainless", tube_between((1250, 671, 380), (1381, 671, 380), 7))
        # suspension: flat swing arm (y 664..674) + compact damper (y 682) per road wheel
        for i, (wx, wz, wr) in enumerate(P.ROAD_WHEELS):
            piv = (wx + 120, wz + 150)
            arm = (cq.Workplane("XZ").center((wx + piv[0]) / 2, (wz + piv[1]) / 2)
                   .slot2D(math.hypot(120, 150) + 60, 50, angle=math.degrees(math.atan2(150, 120))).extrude(-10)
                   .translate((0, 664, 0)))
            put(f"ROAD_WHEEL_SWING_ARM_{i+1}", "TRACK_FRAME", "orange", arm)
            put(f"ROAD_WHEEL_ARM_PIVOT_{i+1}", "TRACK_AXLES", "stainless", cyl(16, 30, "y", (piv[0], 648, piv[1])))
            put(f"DAMPER_BODY_{i+1}", "TRACK_FRAME", "steel_dark", tube_between((wx + 40, 682, wz + 40), (wx - 5, 682, wz + 175), 8))
            put(f"DAMPER_ROD_{i+1}", "TRACK_FRAME", "stainless", tube_between((wx - 5, 682, wz + 175), (wx - 20, 682, wz + 225), 4))

        # --- wheels (dual rims either side of the central guide horn y 805..845)
        sx, sz, sr = P.SPROCKET
        spr = sprocket(sr - 42, 13, 38, 40).translate((sx, 700, sz)).union(
            sprocket(sr - 42, 13, 38, 40).translate((sx, 910, sz)))
        spr = spr.union(cyl(sr - 45, 170, "y", (sx, 740, sz)))
        put("DRIVE_SPROCKET", "DRIVE_SPROCKETS", "steel_dark", spr)
        put("DRIVE_SPROCKET_HUB_CAP", "DRIVE_SPROCKETS", "silver_cap",
            cyl(sr - 50, 20, "y", (sx, 950, sz)).union(cyl(50, 8, "y", (sx, 970, sz))))
        ix, iz, ir = P.IDLER
        idl = cyl(ir - 2, 110, "y", (ix, 690, iz)).union(cyl(ir - 2, 110, "y", (ix, 850, iz))).union(cyl(ir - 60, 50, "y", (ix, 800, iz)))
        put("FRONT_IDLER_WHEEL", "IDLER_WHEELS", "rubber", idl)
        put("FRONT_IDLER_HUB_CAP", "IDLER_WHEELS", "silver_cap",
            cyl(ir - 40, 6, "y", (ix, 960, iz)).union(cyl(50, 12, "y", (ix, 960, iz))))
        for i, (rx, rz, rr) in enumerate(P.SUPPORT_ROLLERS):
            put(f"SUPPORT_ROLLER_{i+1}", "IDLER_WHEELS", "rubber",
                cyl(rr, 110, "y", (rx, 690, rz)).union(cyl(rr, 110, "y", (rx, 850, rz))).union(cyl(10, 50, "y", (rx, 800, rz))))
        for i, (wx, wz, wr) in enumerate(P.ROAD_WHEELS):
            rw = cyl(wr - 2, 110, "y", (wx, 690, wz)).union(cyl(wr - 2, 110, "y", (wx, 850, wz))).union(
                cyl(wr - 50, 50, "y", (wx, 800, wz)))
            put(f"ROAD_WHEEL_{i+1}", "ROAD_WHEELS", "rubber", rw)
            put(f"ROAD_WHEEL_HUB_CAP_{i+1}", "ROAD_WHEELS", "silver_cap",
                cyl(wr - 25, 6, "y", (wx, 960, wz)).union(cyl(40, 12, "y", (wx, 960, wz))))

        # --- axles & bearings
        axles = [(sx, sz, 560, 978), (ix, iz, 640, 972)] + [(wx, wz, 664, 972) for (wx, wz, _) in P.ROAD_WHEELS]
        for i, (ax, az, y0, y1) in enumerate(axles):
            put(f"TRACK_AXLE_{i+1}", "TRACK_AXLES", "stainless", cyl(20, y1 - y0, "y", (ax, y0, az)))
        for i, (ax, az, *_r) in enumerate(axles[:2]):
            put(f"FLANGED_BEARING_{i+1}", "BEARINGS", "steel_dark", cyl(45, 16, "y", (ax, 632, az)))
        for i, (ax, az, *_r) in enumerate(axles):
            put(f"WHEEL_BEARING_{i+1}", "BEARINGS", "stainless", cyl(36, 30, "y", (ax, 805, az)))

        # --- track motor (inside hull) + gearbox
        put("TRACK_MOTOR", "TRACK_MOTORS", "steel_dark", cyl(80, 190, "y", (sx, 350, sz)))
        put("TRACK_GEARBOX", "TRACK_MOTORS", "aluminum", cyl(95, 50, "y", (sx, 550, sz)))
        put("TRACK_MOTOR_FLANGE", "TRACK_MOTORS", "aluminum", box(sx - 110, sx + 110, 540, 550, sz - 110, sz + 110))

    # --- track links (belt), one instance shape placed along the path
    circles_mid = circles_hull(circles, 0)
    line = circles_mid.exterior
    Lp = line.length
    n = int(round(Lp / P.LINK_PITCH))
    pitch = Lp / n
    link = (box(-pitch / 2 + 4, pitch / 2 - 4, P.BELT_Y[0], P.BELT_Y[1], 0, 15)
            .union(box(-16, 16, P.BELT_Y[0], P.BELT_Y[1], 15, 40))
            .union(box(-22, 22, 805, 845, -32, 0)))
    link = link.edges("|Y").fillet(3)
    link_s = S(link)
    link_r = mirror_y(link_s)
    coords = list(line.coords)
    # orientation of the ring (counter-clockwise in x/z?)
    ccw = circles_mid.exterior.is_ccw
    for side, sy, shp in (("LEFT_TRACK", 1, link_s), ("RIGHT_TRACK", -1, link_r)):
        tag = "L" if sy > 0 else "R"
        for k in range(n):
            d = k * pitch
            p0 = line.interpolate(d)
            p1 = line.interpolate((d + 1.0) % Lp)
            t = np.array([p1.x - p0.x, 0, p1.y - p0.y])
            t /= np.linalg.norm(t)
            nrm = np.array([t[2], 0, -t[0]]) if ccw else np.array([-t[2], 0, t[0]])
            M = np.eye(4)
            M[:3, 0] = t
            M[:3, 1] = (0, 1, 0)
            M[:3, 2] = nrm
            if np.linalg.det(M[:3, :3]) < 0:
                M[:3, 0] = -t
            M[:3, 3] = (p0.x, 0, p0.y)
            add(f"TRACK_LINK_{tag}_{k+1:02d}", ("02_TRACK_SYSTEM", side, "TRACK_LINKS"), "rubber", shp,
                M=M, group=f"track_{tag}")


# ====================================================================== 03 PLATFORM SYSTEM
def deck_plate(L, name, layer, frame, grp):
    w = P.PLATFORM_CLEAR / 2 - 0          # 450
    plate = box(0, L, -w + 5, w - 5, -P.DECK_T, 0)
    # drain slots along both edges
    xs = list(np.arange(120, L - 60, 160))
    for xs_ in xs:
        for sy in (1, -1):
            plate = plate.cut(rbox(xs_ - 35, xs_ + 35, sy * 415 - 6, sy * 415 + 6, -P.DECK_T - 1, 1, r_vert=5.9))
    add(name, layer, "antislip", plate, frame=frame, group=grp)
    # raised anti-slip pattern (diamond-plate style bosses), single solid
    pts_a, pts_b = [], []
    for i, xx in enumerate(np.arange(40, L - 30, 60)):
        for j, yy in enumerate(np.arange(-370, 371, 60)):
            (pts_a if (i + j) % 2 == 0 else pts_b).append((xx, yy))
    a = cq.Workplane("XY").pushPoints(pts_a).rect(28, 7).extrude(2.5)
    a = a.rotate((0, 0, 0), (0, 0, 1), 0)
    studs = S(a)
    b = S(cq.Workplane("XY").pushPoints(pts_b).rect(7, 28).extrude(2.5))
    add(name + "_ANTISLIP_PATTERN", layer, "antislip", cq.Compound.makeCompound([studs, b]), frame=frame, group=grp)


def platform_module(key, L, layer, has_underside_xmembers=True):
    """Platform module in its local frame: x 0..L, deck top z=0, centred on y."""
    grp = key
    frame = key
    deck_plate(L, f"{key}_DECK", ("03_PLATFORM_SYSTEM", layer), frame, grp)
    yo = P.PLATFORM_WIDTH / 2
    for sy, tag in ((1, "L"), (-1, "R")):
        prof = rect_tube(P.PROFILE_W, P.FRAME_H, 4, L, "x", (0, sy * (yo - P.PROFILE_W / 2), -P.FRAME_H / 2))
        add(f"{key}_SIDE_PROFILE_{tag}", ("03_PLATFORM_SYSTEM", "PLATFORM_FRAME"), "frame_black", prof, frame=frame, group=grp)
        # deck support angle along the profile
        add(f"{key}_DECK_LEDGE_{tag}", ("03_PLATFORM_SYSTEM", "PLATFORM_FRAME"), "frame_black",
            box(0, L, sy * 440 if sy > 0 else -450, sy * 450 if sy > 0 else -440, -P.DECK_T - 12, -P.DECK_T), frame=frame, group=grp)
    for i, xc in enumerate([30] + list(np.linspace(30, L - 30, 5)[1:-1]) + [L - 30]):
        add(f"{key}_CROSS_MEMBER_{i+1}", ("03_PLATFORM_SYSTEM", "PLATFORM_FRAME"), "frame_black",
            rect_tube(40, 60, 3, 2 * (yo - P.PROFILE_W), "y", (xc, -(yo - P.PROFILE_W), -P.FRAME_H + 30)), frame=frame, group=grp)
    # end caps on side profiles
    for sy, tag in ((1, "L"), (-1, "R")):
        for xe, te in ((-4, "A"), (L, "B")):
            add(f"{key}_PROFILE_CAP_{tag}{te}", ("03_PLATFORM_SYSTEM", "PLATFORM_FRAME"), "polymer_dark",
                box(xe, xe + 4, sy * (yo - P.PROFILE_W) if sy > 0 else -yo, sy * yo if sy > 0 else -(yo - P.PROFILE_W), -P.FRAME_H, 0),
                frame=frame, group=grp)


def gear_hinge(jkey, mod_a, mod_b, xa, za, folds_under):
    """Synchronised dual-axis geared hinge between module A (axis at xa,za) and module B.

    Gear A is fixed to A, gear B is fixed to B; they mesh (z=25, m=8, centre distance 200)
    and are carried by orange link plates (frame '<jkey>_LINK'). Rolling B around the
    fixed A gives B a rotation of 2x the link rotation -> 180 deg fold for 90 deg of link.
    """
    G = ("03_PLATFORM_SYSTEM", "PLATFORM_GEARS")
    H = ("03_PLATFORM_SYSTEM", "PLATFORM_HINGES")
    D = ("03_PLATFORM_SYSTEM", "PLATFORM_DRIVE_LINKS")
    Lk = ("03_PLATFORM_SYSTEM", "PLATFORM_LOCKS")
    gear = spur_gear(P.HINGE_GEAR_M, P.HINGE_GEAR_Z, P.GEAR_W, bore=20, lightening=5)
    gear_s = S(gear)
    cd = P.HINGE_CD
    for sy, tag in ((1, "L"), (-1, "R")):
        yg = sy * P.GEAR_Y if sy > 0 else -(P.GEAR_Y + P.GEAR_W)
        g = gear_s.translate(cq.Vector(0, yg, 0))
        add(f"{jkey}_GEAR_A_{tag}", G, "orange", g.translate(cq.Vector(xa, 0, za)), frame=mod_a, group=f"{jkey}_A")
        add(f"{jkey}_GEAR_B_{tag}", G, "orange", g, frame=mod_b + "_AXIS", group=f"{jkey}_B")
        # gear mount plates bolted to the side profiles
        yp0, yp1 = (P.PLATFORM_WIDTH / 2, P.GEAR_Y) if sy > 0 else (-P.GEAR_Y, -P.PLATFORM_WIDTH / 2)
        mpa = box(xa - 130, xa + 20, yp0, yp1, za - 50, za + 50)
        add(f"{jkey}_GEAR_MOUNT_A_{tag}", H, "orange", mpa, frame=mod_a, group=f"{jkey}_A")
        mpb = box(-20, 130, yp0, yp1, -50, 50)
        add(f"{jkey}_GEAR_MOUNT_B_{tag}", H, "orange", mpb, frame=mod_b + "_AXIS", group=f"{jkey}_B")
        # link plate (rounded slot shape) in link frame (axis A at origin, B at +cd)
        yl0 = P.LINK_Y if sy > 0 else -(P.LINK_Y + 12)
        lp = (cq.Workplane("XZ").center(cd / 2, 0).slot2D(cd + 90, 90).extrude(-12)
              .translate((0, yl0, 0)))
        lp = lp.cut(cyl(20.5, 20, "y", (0, yl0 - 4, 0))).cut(cyl(20.5, 20, "y", (cd, yl0 - 4, 0)))
        lp = lp.cut(cq.Workplane("XZ").center(cd / 2, 0).slot2D(70, 30).extrude(-20).translate((0, yl0 - 4, 0)))
        add(f"{jkey}_LINK_PLATE_{tag}", D, "orange", lp, frame=f"{jkey}_LINK", group=f"{jkey}_L")
        # hinge axles (A and B), nuts
        ya0 = P.PLATFORM_WIDTH / 2 - 60 if sy > 0 else -(P.LINK_Y + 22)
        for ax, nm in ((0, "A"), (cd, "B")):
            sh = cyl(20, P.LINK_Y + 22 - (P.PLATFORM_WIDTH / 2 - 60), "y", (ax, ya0, 0))
            add(f"{jkey}_HINGE_AXLE_{nm}_{tag}", ("09_FASTENERS", "AXLES"), "stainless", sh, frame=f"{jkey}_LINK", group=f"{jkey}_L")
            yn = P.LINK_Y + 12 if sy > 0 else -(P.LINK_Y + 22)
            nut = cq.Workplane("XZ").polygon(6, 58).extrude(-10).translate((ax, yn, 0))
            add(f"{jkey}_HINGE_NUT_{nm}_{tag}", ("09_FASTENERS", "NUTS"), "stainless", nut, frame=f"{jkey}_LINK", group=f"{jkey}_L")
        # deployed-position lock bolt (spring plunger through link into gear-B hub)
        yo_ = yl0 if sy > 0 else yl0 + 12            # inner face of the link plate
        lk = (cyl(9, 26, "y", (cd - 70, yo_ - 2, 0)) if sy > 0 else cyl(9, 26, "y", (cd - 70, yo_ - 24, 0)))
        lk = lk.union(cyl(16, 22, "y", (cd - 70, yo_ + 12 if sy > 0 else yo_ - 34, 0)))
        add(f"{jkey}_LOCK_PLUNGER_{tag}", Lk, "stainless", lk, frame=f"{jkey}_LINK", group=f"{jkey}_L")
        # mechanical end stops on the modules (contact faces in deployed state)
        st = box(xa - 30, xa - 5, yp0, yp1, za + (-95 if not folds_under else 55), za + (-70 if not folds_under else 80))
        add(f"{jkey}_END_STOP_A_{tag}", Lk, "polymer_dark", st, frame=mod_a, group=f"{jkey}_A")
        # gear bolts (6 per gear)
        for k in range(6):
            a = 2 * math.pi * k / 6
            bx, bz = 32 * math.cos(a), 32 * math.sin(a)
            yb = (P.GEAR_Y + P.GEAR_W - 6.5) if sy > 0 else -(P.GEAR_Y + P.GEAR_W - 6.5)   # heads flush in counterbores
            bolt = hexbolt(10, 40).rotate((0, 0, 0), (1, 0, 0), -90 if sy > 0 else 90).translate((bx, yb, bz))
            add(f"{jkey}_GEAR_B_BOLT_{tag}{k+1}", ("09_FASTENERS", "BOLTS"), "stainless", S(bolt),
                frame=mod_b + "_AXIS", group=f"{jkey}_B")
            add(f"{jkey}_GEAR_A_BOLT_{tag}{k+1}", ("09_FASTENERS", "BOLTS"), "stainless",
                S(bolt).translate(cq.Vector(xa, 0, za)), frame=mod_a, group=f"{jkey}_A")
    # drive: worm gearmotor on module A driving the link shaft (axis A)
    zc = -55                                      # inside the module frame (deck -8 .. underside -100)
    gm = cyl(45, 160, "y", (xa - 140, -80, zc))
    gm = gm.union(box(xa - 200, xa - 80, -90, 90, -100, -12))
    add(f"{jkey}_GEARMOTOR", D, "steel_dark", gm, frame=mod_a, group=f"{jkey}_A")
    # cross drive shaft inside the module frame -> toothed-belt drive housed in gear mount plate A
    add(f"{jkey}_DRIVE_SHAFT", D, "stainless", cyl(15, 2 * P.GEAR_Y, "y", (xa - 140, -P.GEAR_Y, zc)),
        frame=mod_a, group=f"{jkey}_A")
    # bridge flap on module B (closes the walkway gap over the hinge)
    bz = -za if not folds_under else P.FRAME_H        # deck level, relative to axis B
    flap = box(-cd + 10, 0, -440, 440, bz - P.DECK_T, bz)
    add(f"{jkey}_BRIDGE_FLAP", ("03_PLATFORM_SYSTEM", "PLATFORM_HINGES"), "antislip", flap, frame=mod_b + "_AXIS", group=f"{jkey}_B")
    add(f"{jkey}_BRIDGE_FLAP_RIB", ("03_PLATFORM_SYSTEM", "PLATFORM_HINGES"), "frame_black",
        box(-cd + 10, 0, -10, 10, bz - P.DECK_T - 40, bz - P.DECK_T), frame=mod_b + "_AXIS", group=f"{jkey}_B")


def build_platforms():
    platform_module("P1", P.P1_LEN, "PLATFORM_01")
    platform_module("P2", P.P2_LEN, "PLATFORM_02")
    platform_module("P3", P.P3_LEN, "PLATFORM_EXTENSION")
    # J1: deck-level hinge P1 -> P2 ; J2: underside hinge P2 -> P3
    gear_hinge("J1", "P1", "P2", P.P1_LEN, P.J1_AXIS_DZ, folds_under=False)
    gear_hinge("J2", "P2", "P3", P.P2_LEN, -P.FRAME_H, folds_under=True)
    # P1 underside pedestal seats
    for px in PEDESTAL_X:
        for sy, tag in ((1, "L"), (-1, "R")):
            cx, cy = px - P.P1_X0, sy * PEDESTAL_Y
            seat = (box(cx - 55, cx + 55, cy - 45, cy + 45, -P.FRAME_H, -P.FRAME_H + 40)
                    .cut(box(cx - 51, cx + 51, cy - 41, cy + 41, -P.FRAME_H - 1, -P.FRAME_H + 30)))
            add(f"P1_PEDESTAL_SEAT_{px}_{tag}", ("03_PLATFORM_SYSTEM", "PLATFORM_FRAME"), "orange", seat, frame="P1", group="P1")
    # P3 front stair pivot brackets
    yo = P.PLATFORM_WIDTH / 2
    for sy, tag in ((1, "L"), (-1, "R")):
        yb = (yo, yo + 18) if sy > 0 else (-yo - 18, -yo)
        br = box(P.P3_LEN - 150, P.P3_LEN + FRONT_PIVOT_DX + 35, yb[0], yb[1], -80, P.FRONT_PIVOT_DZ + 35).edges("|Y").fillet(15)
        add(f"P3_STAIR_PIVOT_BRACKET_{tag}", ("04_STAIR_SYSTEM", "STAIR_HINGES"), "orange", br, frame="P3", group="P3")
        y0 = P.FRONT_STAIR_W / 2 + 30 if sy > 0 else -(yo + 22)
        add(f"FRONT_STAIR_PIVOT_PIN_{tag}", ("04_STAIR_SYSTEM", "STAIR_HINGES"), "stainless",
            cyl(15, yo + 22 - (P.FRONT_STAIR_W / 2 + 30), "y", (P.P3_LEN + FRONT_PIVOT_DX, y0, P.FRONT_PIVOT_DZ)), frame="P3", group="P3")
    # P3 support-leg hinge brackets
    for sy, tag in ((1, "L"), (-1, "R")):
        hb = box(P.P3_LEN - 180, P.P3_LEN - 40, sy * 350 - 70, sy * 350 + 70, -190, -P.FRAME_H)
        hb = hb.cut(box(P.P3_LEN - 181, P.P3_LEN - 39, sy * 350 - 56, sy * 350 + 56, -191, -P.FRAME_H - 5))
        add(f"P3_LEG_HINGE_BRACKET_{tag}", ("06_SUPPORT_SYSTEM", "SUPPORT_HINGES"), "orange", hb, frame="P3", group="P3")


# ====================================================================== 05 RAILINGS
RAIL_POSTS = {"P1": (60, 550, 1040), "P2": (60, 552, 1045), "P3": (60, 600, 1140)}


def build_railings():
    for mod, xs in RAIL_POSTS.items():
        for sy, tag in ((1, "L"), (-1, "R")):
            fr_low = f"RAIL_{mod}_{tag}"
            fr_up = f"RAIL_{mod}_{tag}_UP"
            grp = f"rail_{mod}"
            hz = P.RAIL_HINGE_Z[0] if sy > 0 else P.RAIL_HINGE_Z[1]
            # hinge knuckles on deck (module frame)
            for x in xs:
                kn = box(x - 30, x + 30, sy * P.RAIL_SPACING / 2 - 22, sy * P.RAIL_SPACING / 2 + 22, 2.5, hz + 8)
                kn = kn.union(cyl(14, 100, "x", (x - 50, sy * P.RAIL_SPACING / 2, hz)))
                add(f"RAIL_HINGE_{mod}_{tag}_{x}", ("05_GUARDRAIL_SYSTEM", "RAILING_HINGES"), "orange", kn, frame=mod, group=mod)
            # lower posts + mid rail (panel frame: origin on hinge axis, z up along post)
            for x in xs:
                lp = tube(P.POST_R_LOW, P.POST_R_LOW - 2.5, P.POST_LOW_LEN - 24, "z", (x, 0, 24))
                add(f"RAIL_POST_LOWER_{mod}_{tag}_{x}", ("05_GUARDRAIL_SYSTEM", "RAILING_VERTICAL"), "brushed", lp, frame=fr_low, group=grp)
                col = tube(P.POST_R_LOW + 3, P.POST_R_LOW, 22, "z", (x, 0, P.POST_LOW_LEN - 23))
                add(f"RAIL_TELESCOPE_LOCK_COLLAR_{mod}_{tag}_{x}", ("05_GUARDRAIL_SYSTEM", "RAILING_LOCKS"), "orange", col, frame=fr_low, group=grp)
                up = tube(P.POST_R_UP, P.POST_R_UP - 2, 372, "z", (x, 0, 28))
                add(f"RAIL_POST_UPPER_{mod}_{tag}_{x}", ("05_GUARDRAIL_SYSTEM", "RAILING_VERTICAL"), "brushed", up, frame=fr_up, group=grp)
                cap = tube(P.POST_R_UP + 1.5, P.POST_R_UP, 40, "z", (x, 0, 330))
                add(f"RAIL_POST_REFLECTOR_CAP_{mod}_{tag}_{x}", ("05_GUARDRAIL_SYSTEM", "RAILING_VERTICAL"), "reflector", cap, frame=fr_up, group=grp)
            mid = tube_between((xs[0], 0, P.MID_RAIL_Z - 25), (xs[-1], 0, P.MID_RAIL_Z - 25), P.RAIL_R, P.RAIL_R - 2.5)
            for x in xs:                       # saddle-welded onto the posts, never into the post bore
                mid = mid.cut(cyl(P.POST_R_LOW, 200, "z", (x, 0, P.MID_RAIL_Z - 125)))
            add(f"RAIL_MID_{mod}_{tag}", ("05_GUARDRAIL_SYSTEM", "RAILING_HORIZONTAL"), "brushed", mid, frame=fr_low, group=grp)
            top = tube_between((xs[0] - 40, 0, 420), (xs[-1] + 40, 0, 420), P.RAIL_R, P.RAIL_R - 2.5)
            add(f"RAIL_TOP_{mod}_{tag}", ("05_GUARDRAIL_SYSTEM", "RAILING_HORIZONTAL"), "brushed", top, frame=fr_up, group=grp)
            for xe, d in ((xs[0] - 40, -1), (xs[-1] + 40, 1)):
                capb = cq.Workplane("XY").sphere(P.RAIL_R).translate((xe, 0, 420)).intersect(
                    box(xe if d > 0 else xe - 30, xe + 30 if d > 0 else xe, -30, 30, 380, 460))
                add(f"RAIL_END_CAP_{mod}_{tag}_{int(xe)}", ("05_GUARDRAIL_SYSTEM", "RAILING_HORIZONTAL"), "polymer_dark",
                    capb, frame=fr_up, group=grp)
            # reflector strips on top rail (inside face)
            for x in np.linspace(xs[0] + 150, xs[-1] - 150, 2):
                rf = box(x - 60, x + 60, -P.RAIL_R - 4 if sy > 0 else P.RAIL_R + 1, -P.RAIL_R - 1 if sy > 0 else P.RAIL_R + 4, 410, 430)
                add(f"RAIL_REFLECTOR_{mod}_{tag}_{int(x)}", ("05_GUARDRAIL_SYSTEM", "RAILING_VERTICAL"), "reflector", rf, frame=fr_up, group=grp)
            # hinge lock pins (index pins at first and last post)
            for x in (xs[0], xs[-1]):
                lpn = cyl(6, 50, "y", (x + 48, -25, 0)).union(cyl(12, 8, "y", (x + 48, 25, 0)))
                add(f"RAIL_HINGE_LOCK_PIN_{mod}_{tag}_{x}", ("05_GUARDRAIL_SYSTEM", "RAILING_LOCKS"), "stainless",
                    lpn.translate((0, sy * P.RAIL_SPACING / 2, hz)), frame=mod, group=mod)


# ====================================================================== 04 STAIRS


def build_rear_stair():
    W = P.REAR_STAIR_W / 2           # 450
    ys = W + 25                      # side tube centre 475
    L = ("04_STAIR_SYSTEM", "STAIR_MODULE_01")
    # landing (top platform over the hull rear) - fixed, frame world
    lx0, lx1 = P.REAR_LANDING_X0 + 15, P.P1_X0 - 6
    deck_plate(lx1 - lx0, "REAR_LANDING_DECK", L, "LANDING", "landing")
    for sy, tag in ((1, "L"), (-1, "R")):
        add(f"REAR_LANDING_PROFILE_{tag}", L, "frame_black",
            rect_tube(P.PROFILE_W, P.FRAME_H, 4, lx1 - lx0, "x", (0, sy * 475, -P.FRAME_H / 2)), frame="LANDING", group="landing")
    for i, xc in enumerate((30, (lx1 - lx0) - 30)):
        add(f"REAR_LANDING_CROSS_{i+1}", L, "frame_black", rect_tube(40, 60, 3, 900, "y", (xc, -450, -70)), frame="LANDING", group="landing")
    # step modules (nested L-frames): local origin = front-top edge of tread, centred
    for k in range(4):
        fr = f"RS_{k}"
        tread = box(-REAR_TREAD_D, 0, -W, W, -30, 0)
        add(f"REAR_STEP_TREAD_{k+1}", ("04_STAIR_SYSTEM", "STAIR_STEPS"), "antislip", tread, frame=fr, group="rear_stair")
        nose = box(-30, 0, -W, W, 0, 2)
        add(f"REAR_STEP_NOSING_{k+1}", ("04_STAIR_SYSTEM", "STAIR_STEPS"), "reflector", nose, frame=fr, group="rear_stair")
        for sy, tag in ((1, "L"), (-1, "R")):
            h = tube_between((-REAR_TREAD_D + 15, sy * ys, -45), (-15, sy * ys, -45), 15, 12.5)
            v = tube_between((0, sy * ys, -60), (0, sy * ys, 100), 15, 12.5)
            br = box(-REAR_TREAD_D + 15, -15, sy * ys - 25 if sy > 0 else -ys - 5, sy * ys + 5 if sy > 0 else -ys + 25, -30, -20)
            add(f"REAR_STEP_SIDE_FRAME_{k+1}_{tag}", ("04_STAIR_SYSTEM", "STAIR_SIDE_PROFILES"), "brushed",
                cq.Compound.makeCompound([S(h), S(v)]), frame=fr, group="rear_stair")
            add(f"REAR_STEP_TREAD_CLEAT_{k+1}_{tag}", ("04_STAIR_SYSTEM", "STAIR_SIDE_PROFILES"), "brushed", br, frame=fr, group="rear_stair")
            # parallel link knuckle at riser top (connects to the module above / landing)
            kn = box(-15, 15, sy * ys + 15 if sy > 0 else -ys - 25, sy * ys + 25 if sy > 0 else -ys - 15, 85, 150)
            add(f"REAR_STEP_LINK_KNUCKLE_{k+1}_{tag}", ("04_STAIR_SYSTEM", "STAIR_HINGES"), "orange", kn, frame=fr, group="rear_stair")
            lockp = cyl(6, 50, "y", (-REAR_TREAD_D + 20, sy * ys - 25, -45))
            add(f"REAR_STEP_LOCK_PIN_{k+1}_{tag}", ("04_STAIR_SYSTEM", "STAIR_LOCKS"), "stainless", lockp, frame=fr, group="rear_stair")
    # retractable feet on the lowest module (frame RS_FEET follows RS_3 + slide)
    for sy, tag in ((1, "L"), (-1, "R")):
        for xf in (0,):
            ft = tube_between((xf, sy * ys, -60), (xf, sy * ys, -200), 11)
            pad = cyl(12, 15, "z", (xf, sy * ys, -215))
            add(f"REAR_STAIR_FOOT_{tag}_{abs(xf)}", ("04_STAIR_SYSTEM", "STAIR_SIDE_PROFILES"), "stainless", ft, frame="RS_FEET", group="rear_stair")
            add(f"REAR_STAIR_FOOT_PAD_{tag}_{abs(xf)}", ("04_STAIR_SYSTEM", "STAIR_SIDE_PROFILES"), "rubber", pad, frame="RS_FEET", group="rear_stair")


def front_stair_geom():
    """Return local stringer start (x0, ztop) so that the deployed stair meets the deck edge."""
    th = math.radians(P.FRONT_STAIR_ANGLE_DEPLOYED)
    c, s = math.cos(th), math.sin(th)
    h = P.FRONT_PIVOT_DZ
    # local point that maps (by Ry(th)) onto (10, -h) relative to pivot
    d = FRONT_START_DX - FRONT_PIVOT_DX          # stair top edge relative to the pivot (world x)
    x0 = c * d - s * (-h)
    z0 = s * d + c * (-h)
    return x0, z0


def build_front_stair():
    L2 = ("04_STAIR_SYSTEM",)
    x0, zt = front_stair_geom()
    Ls = P.FRONT_STAIR_LEN
    W = P.FRONT_STAIR_W / 2
    th = P.FRONT_STAIR_ANGLE_DEPLOYED
    for sy, tag in ((1, "L"), (-1, "R")):
        st = rect_tube(150, 30, 3, Ls, "x", (x0, sy * (W + 15), zt - 75))
        st = st.rotate((0, 0, 0), (1, 0, 0), 0)
        stg = rect_tube(30, 150, 3, Ls, "x", (x0, sy * (W + 15), zt - 75))
        # plumb-cut top end: nothing of the stair reaches behind the P3 end face when deployed
        trim = box(-2000, 5 - FRONT_PIVOT_DX, -600, 600, -2000, 2000).rotate((0, 0, 0), (0, 1, 0), -th)
        stg = stg.cut(trim)
        add(f"FRONT_STAIR_STRINGER_{tag}", L2 + ("STAIR_SIDE_PROFILES",), "frame_black", S(stg), frame="FS", group="front_stair")
        # pivot plate at the top end
        pp = box(x0 - 20, x0 + 140, sy * (W + 30) if sy > 0 else -(W + 32), sy * (W + 32) if sy > 0 else -(W + 30), zt - 150, 30)
        pp = cq.Workplane("XZ").polyline([(-25, 25), (25, 25), (x0 + 140, zt), (x0 + 140, zt - 150), (x0 - 20, zt - 150), (-25, -25)]).close().extrude(-14)
        pp = pp.translate((0, (W + 30) if sy > 0 else -(W + 44), 0))
        pp = pp.cut(box(-2000, 5 - FRONT_PIVOT_DX, -600, 600, -2000, 2000).rotate((0, 0, 0), (0, 1, 0), -th)
                    .cut(cyl(40, 1200, "y", (0, -600, 0))))
        add(f"FRONT_STAIR_PIVOT_PLATE_{tag}", L2 + ("STAIR_HINGES",), "orange", pp, frame="FS", group="front_stair")
        # foot shoe
        shoe = box(x0 + Ls, x0 + Ls + 25, sy * W if sy > 0 else -W - 30, sy * W + 30 if sy > 0 else -W, zt - 150, zt)
        add(f"FRONT_STAIR_FOOT_SHOE_{tag}", L2 + ("STAIR_SIDE_PROFILES",), "rubber", shoe, frame="FS", group="front_stair")
        # deployed-angle lock strut
        add(f"FRONT_STAIR_LOCK_PIN_{tag}", L2 + ("STAIR_LOCKS",), "stainless",
            cyl(8, 40, "y", (x0 + 90, (W + 20) if sy > 0 else -(W + 60), zt - 60)), frame="FS", group="front_stair")
    n = 6
    for i in range(n):
        sx = x0 + 120 + i * (Ls - 200) / (n - 1)
        tr = box(-95, 95, -W, W, -14, 14)
        tr = tr.rotate((0, 0, 0), (0, 1, 0), -th)   # horizontal in the deployed pose
        tr = tr.translate((sx, 0, zt - 70))
        add(f"FRONT_STAIR_TREAD_{i+1}", L2 + ("STAIR_STEPS",), "antislip", tr, frame="FS", group="front_stair")
        nos = box(80, 95, -W, W, 14, 17).rotate((0, 0, 0), (0, 1, 0), -th).translate((sx, 0, zt - 70))
        add(f"FRONT_STAIR_TREAD_NOSING_{i+1}", L2 + ("STAIR_STEPS",), "reflector", nos, frame="FS", group="front_stair")
    # top landing plate (horizontal when deployed) bridging the 110 mm to the P3 deck edge
    tp = box(5 - FRONT_PIVOT_DX, FRONT_START_DX - FRONT_PIVOT_DX + 15, -W, W, -P.FRONT_PIVOT_DZ - 8, -P.FRONT_PIVOT_DZ).rotate((0, 0, 0), (0, 1, 0), -th)
    add("FRONT_STAIR_TOP_PLATE", L2 + ("STAIR_STEPS",), "antislip", tp, frame="FS", group="front_stair")
    add("FRONT_STAIR_BOTTOM_BAR", L2 + ("STAIR_SIDE_PROFILES",), "frame_black",
        rect_tube(40, 40, 3, 2 * W, "y", (x0 + Ls - 40, -W, zt - 120)), frame="FS", group="front_stair")


# ====================================================================== 06 SUPPORT LEGS
def build_legs():
    SP = ("06_SUPPORT_SYSTEM",)
    for sy, tag in ((1, "L"), (-1, "R")):
        fr = f"LEG_{tag}"
        add(f"SUPPORT_LEG_OUTER_{tag}", SP + ("SUPPORT_LEGS",), "frame_black",
            rect_tube(100, 100, 5, 500, "z", (0, 0, -520)).union(cyl(24, 112, "y", (0, -56, 0)))
            .union(box(-50, 50, -50, 50, -30, -20)), frame=fr, group="legs")
        add(f"SUPPORT_LEG_HINGE_PIN_{tag}", SP + ("SUPPORT_HINGES",), "stainless",
            cyl(12, 150, "y", (P.P3_LEN - 110, sy * 350 - 75, LEG_HINGE_Z)), frame="P3", group="P3")
        add(f"SUPPORT_LEG_INDEX_PIN_{tag}", SP + ("SUPPORT_LOCKS",), "stainless",
            cyl(8, 140, "y", (0, -70, -480)).union(cyl(18, 10, "y", (0, 60, -480))), frame=fr, group="legs")
        add(f"SUPPORT_LEG_LOCK_COLLAR_{tag}", SP + ("SUPPORT_LOCKS",), "orange",
            rect_tube(116, 116, 8, 40, "z", (0, 0, -520)), frame=fr, group="legs")
        fr2 = f"LEG_{tag}_INNER"
        ext = (P.DECK_Z + LEG_HINGE_Z) - 590
        inner = rect_tube(84, 84, 4, 400, "z", (0, 0, -520))
        for zh in (-480, -480 + ext):
            inner = inner.cut(cyl(8.5, 140, "y", (0, -70, zh)))
        add(f"SUPPORT_LEG_INNER_{tag}", SP + ("TELESCOPIC_PARTS",), "aluminum", inner, frame=fr2, group="legs")
        add(f"SUPPORT_LEG_BALL_JOINT_{tag}", SP + ("TELESCOPIC_PARTS",), "stainless",
            cq.Workplane("XY").sphere(20).translate((0, 0, -540)).union(cyl(12, 30, "z", (0, 0, -540))), frame=fr2, group="legs")
        # foot plate 180 x 260, offset 40 mm off the leg axis so it fits between P3 and P2 when folded
        add(f"SUPPORT_FOOT_PLATE_{tag}", SP + ("FOOT_PLATES",), "orange",
            rbox(-50, 130, -130, 130, -575, -560, r_vert=20), frame=fr2, group="legs")
        add(f"SUPPORT_FOOT_PAD_{tag}", SP + ("FOOT_PLATES",), "rubber",
            rbox(-46, 126, -126, 126, -590, -575, r_vert=18), frame=fr2, group="legs")


# ====================================================================== BUILD ALL
def build_all():
    PARTS.clear()
    build_body()
    build_internals()
    build_tracks()
    build_platforms()
    build_railings()
    build_rear_stair()
    build_front_stair()
    build_legs()
    return PARTS
