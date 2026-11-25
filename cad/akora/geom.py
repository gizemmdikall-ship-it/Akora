"""Geometry helpers for the AKORA CAD model (CadQuery / OpenCASCADE, units: mm).

Every helper returns a cadquery.Workplane holding closed solids (B-rep / NURBS).
"""
import math

import cadquery as cq
import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union


# ---------------------------------------------------------------- primitives
def box(x0, x1, y0, y1, z0, z1):
    """Axis aligned box from min/max coordinates."""
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def rbox(x0, x1, y0, y1, z0, z1, r_vert=0.0, r_top=0.0, r_bot=0.0):
    """Box with filleted vertical edges and optional top/bottom edge fillets."""
    w = box(x0, x1, y0, y1, z0, z1)
    if r_vert > 0:
        w = w.edges("|Z").fillet(r_vert)
    if r_top > 0:
        w = w.faces(">Z").edges().fillet(r_top)
    if r_bot > 0:
        w = w.faces("<Z").edges().fillet(r_bot)
    return w


def cyl(r, h, axis="z", base=(0, 0, 0)):
    """Cylinder of radius r and length h starting at base along +axis."""
    plane = {"x": "YZ", "y": "XZ", "z": "XY"}[axis]
    w = cq.Workplane(plane).circle(r).extrude(h)
    if axis == "y":          # XZ workplane extrudes towards -Y
        w = w.mirror("XZ")
    return w.translate(base)


def tube(r_out, r_in, h, axis="z", base=(0, 0, 0)):
    plane = {"x": "YZ", "y": "XZ", "z": "XY"}[axis]
    w = cq.Workplane(plane).circle(r_out).circle(r_in).extrude(h)
    if axis == "y":
        w = w.mirror("XZ")
    return w.translate(base)


def rect_tube(w, h, t, length, axis="x", base=(0, 0, 0)):
    """Rectangular hollow section, cross-section w x h, wall t, along axis."""
    plane = {"x": "YZ", "y": "XZ", "z": "XY"}[axis]
    s = (cq.Workplane(plane).rect(w, h).rect(w - 2 * t, h - 2 * t).extrude(length))
    if axis == "y":
        s = s.mirror("XZ")
    return s.translate(base)


def tube_between(p0, p1, r_out, r_in=0.0):
    """Round tube (or rod when r_in == 0) between two 3D points."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    wp = cq.Workplane(cq.Plane(origin=tuple(p0), xDir=_perp(d), normal=tuple(d / L)))
    wp = wp.circle(r_out)
    if r_in > 0:
        wp = wp.circle(r_in)
    return wp.extrude(L)


def _perp(d):
    d = np.asarray(d, float) / np.linalg.norm(d)
    a = np.array([0, 0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0, 0])
    v = np.cross(d, a)
    return tuple(v / np.linalg.norm(v))


# ---------------------------------------------------------------- 2D profiles
def poly_extrude(pts, h, plane="XZ", z0=0.0):
    """Extrude a closed 2D polyline. plane 'XZ' -> profile in X/Z, extruded +Y from y=z0."""
    if plane == "XZ":
        w = cq.Workplane("XZ").polyline(pts).close().extrude(-h)  # towards +Y
        return w.translate((0, z0, 0))
    if plane == "XY":
        return cq.Workplane("XY").polyline(pts).close().extrude(h).translate((0, 0, z0))
    if plane == "YZ":
        return cq.Workplane("YZ").polyline(pts).close().extrude(h).translate((z0, 0, 0))
    raise ValueError(plane)


def shapely_to_pts(poly):
    pts = list(poly.exterior.coords)[:-1]
    return [(float(x), float(y)) for x, y in pts]


def circles_hull(circles, offset=0.0, res=48):
    """Convex hull of circles (cx, cz, r) grown by offset -> shapely Polygon."""
    geoms = [Point(cx, cz).buffer(r + offset, quad_segs=res // 4) for cx, cz, r in circles]
    return unary_union(geoms).convex_hull


def ring_xz(outer, inner, thickness, y0, simplify=0.5):
    """Plate in the XZ plane with outline 'outer' and hole 'inner' (shapely polygons)."""
    o = outer.simplify(simplify)
    i = inner.simplify(simplify)
    w = (cq.Workplane("XZ").polyline(shapely_to_pts(o)).close()
         .polyline(shapely_to_pts(i)).close().extrude(-thickness))
    return w.translate((0, y0, 0))


def rounded_rect_ring(x0, x1, y0, y1, r, off_out, off_in, z0, z1):
    """Ring following a rounded rectangle, between inward offsets off_out..off_in (off_in > off_out)."""
    def rr(o):
        return (cq.Workplane("XY").rect(x1 - x0 - 2 * o, y1 - y0 - 2 * o)
                .extrude(z1 - z0).edges("|Z").fillet(max(r - o, 1.0)))
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    ring = rr(off_out).cut(rr(off_in))
    return ring.translate((cx, cy, z0))


def rounded_rect_points(x0, x1, y0, y1, r, off, n):
    """n points spread along the centre line of a rounded rectangle inset by off."""
    poly = Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)]).buffer(-r).buffer(r - off, quad_segs=16)
    line = poly.exterior
    L = line.length
    return [(line.interpolate(L * k / n).x, line.interpolate(L * k / n).y) for k in range(n)]


# ---------------------------------------------------------------- gears
def involute_gear_profile(m, z, pa_deg=20.0, n_flank=7):
    """Closed 2D polyline of a standard external involute spur gear (centre at origin)."""
    pa = math.radians(pa_deg)
    rp = m * z / 2
    rb = rp * math.cos(pa)
    ra = rp + m
    rf = rp - 1.25 * m

    def inv(a):
        return math.tan(a) - a

    half_pitch = math.pi / (2 * z) + inv(pa)          # half tooth angle at base circle
    r_start = max(rb, rf)
    radii = np.linspace(r_start, ra, n_flank)

    def half(r):
        a = math.acos(min(1.0, rb / r))
        return half_pitch - inv(a)

    pts = []
    for k in range(z):
        c = 2 * math.pi * k / z
        h0 = half(r_start)
        pts.append((rf * math.cos(c - h0 - 0.0001), rf * math.sin(c - h0 - 0.0001)))
        for r in radii:                                 # right flank up
            a = c - half(r)
            pts.append((r * math.cos(a), r * math.sin(a)))
        for r in radii[::-1]:                           # left flank down
            a = c + half(r)
            pts.append((r * math.cos(a), r * math.sin(a)))
        pts.append((rf * math.cos(c + h0 + 0.0001), rf * math.sin(c + h0 + 0.0001)))
        # root arc to next tooth
        nxt = 2 * math.pi * (k + 1) / z - h0
        a0 = c + h0
        for t in np.linspace(0, 1, 4)[1:-1]:
            a = a0 + (nxt - a0) * t
            pts.append((rf * math.cos(a), rf * math.sin(a)))
    return pts


def spur_gear(m, z, width, bore=0.0, hub_r=0.0, hub_w=0.0, lightening=0):
    """Spur gear in the XZ plane (axis along +Y), width from y=0 to y=width."""
    pts = involute_gear_profile(m, z)
    g = cq.Workplane("XZ").polyline(pts).close().extrude(-width)
    if lightening:
        rf = m * z / 2 - 1.25 * m
        rr = (rf * 0.62)
        hole_r = rf * 0.17
        for k in range(lightening):
            a = 2 * math.pi * k / lightening + math.pi / lightening
            h = cyl(hole_r, width + 2, "y", (rr * math.cos(a), -1, rr * math.sin(a)))
            g = g.cut(h)
    if hub_r > 0 and hub_w > width:
        g = g.union(cyl(hub_r, hub_w, "y", (0, -(hub_w - width) / 2, 0)))
    if bore > 0:
        g = g.cut(cyl(bore, max(width, hub_w) + 10, "y", (0, -max(hub_w, width), 0)))
    return g


def rack(m, n_teeth, width, base_h, plane_axis="x"):
    """Straight rack along +X, teeth pointing +Z, width along Y (0..width)."""
    p = math.pi * m
    h = 2.25 * m                                  # whole depth
    a = h * math.tan(math.radians(20))            # flank run
    top = p / 2 - a                               # tooth thickness ~p/2 at mid-depth
    gap = p - (2 * a + top)
    pts = [(0, -base_h), (0, 0)]
    for k in range(n_teeth):
        x0 = k * p + gap / 2
        pts += [(x0, 0), (x0 + a, h), (x0 + a + top, h), (x0 + 2 * a + top, 0)]
    pts += [(n_teeth * p, 0), (n_teeth * p, -base_h)]
    return cq.Workplane("XZ").polyline(pts).close().extrude(-width)


def sprocket(r_root, z, tooth_h, width):
    """Simple track drive sprocket (rounded teeth) in XZ plane, axis +Y."""
    pts = []
    for k in range(z):
        c = 2 * math.pi * k / z
        w = math.pi / z
        for a, r in ((c - w * 0.95, r_root), (c - w * 0.45, r_root + tooth_h * 0.85),
                     (c - w * 0.18, r_root + tooth_h), (c + w * 0.18, r_root + tooth_h),
                     (c + w * 0.45, r_root + tooth_h * 0.85), (c + w * 0.95, r_root)):
            pts.append((r * math.cos(a), r * math.sin(a)))
    return cq.Workplane("XZ").polyline(pts).close().extrude(-width)


# ---------------------------------------------------------------- transforms
def T(x=0.0, y=0.0, z=0.0):
    M = np.eye(4)
    M[:3, 3] = (x, y, z)
    return M


def R(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    if axis == "x":
        M[1:3, 1:3] = [[c, -s], [s, c]]
    elif axis == "y":
        M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, s, -s, c
    else:
        M[0:2, 0:2] = [[c, -s], [s, c]]
    return M


def chain(*Ms):
    out = np.eye(4)
    for M in Ms:
        out = out @ M
    return out


def lerp(a, b, t):
    return a + (b - a) * t
