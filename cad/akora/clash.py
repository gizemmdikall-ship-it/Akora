"""Interference check: boolean common volume between every pair of placed solids whose
bounding boxes overlap. Pairs where one side is a fastener/axle/pin passing through a
plate are reported separately (bolt holes are not drilled in every plate)."""
import itertools

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from .export import placed, to_loc

FASTENER_HINT = ()   # after drill.py every remaining overlap is reported as a real clash
# designed contacts / nests: (substring A, substring B)
ALLOWED = [
    ("TRACK_LINK", "TRACK_LINK"),               # neighbouring links share hinge pins
    ("COVER_GASKET_EPDM", "TOP_COVER"),          # 2 mm (25 %) designed gasket squeeze by the cover lip
]


def _bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b)
    return np.array(b.Get())


def _vol(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    return g.Mass()


def check(parts, cfg, min_vol=200.0, shrink=0.3):
    items = []
    for p, M in placed(parts, cfg):
        s = p.shape.moved(to_loc(M)).wrapped
        bb = _bbox(s)
        items.append((p, s, bb))
    real, through = [], []
    n = len(items)
    bbs = np.array([it[2] for it in items])
    for i in range(n):
        lo, hi = bbs[i, :3] + shrink, bbs[i, 3:] - shrink
        cand = np.where(np.all(bbs[i + 1:, :3] < hi, axis=1) & np.all(bbs[i + 1:, 3:] > lo, axis=1))[0] + i + 1
        for j in cand:
            a, b = items[i][0], items[j][0]
            if any((x in a.name and y in b.name) or (x in b.name and y in a.name) for x, y in ALLOWED):
                continue
            op = BRepAlgoAPI_Common(items[i][1], items[j][1])
            if not op.IsDone():
                continue
            v = _vol(op.Shape())
            if v > min_vol:
                is_f = False
                (through if is_f else real).append((a.name, b.name, round(v / 1000.0, 2)))
    return real, through
