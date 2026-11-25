"""Automatic hole / pocket generation.

Pins, axles, bolts, cables, sensors ... (penetrators) that pass through a plate are
subtracted from that plate, but only when the two parts keep the same relative
position in every configuration (rigidly connected, or the penetrator is an axle that
only spins about its own axis). The result: real bores, clearance holes and pockets in
the B-rep instead of overlapping solids.
"""
import re

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common

from .clash import _bbox, _vol
from .kinematics import frames
from .export import to_loc

PEN_RANK = [  # higher in the list = stronger penetrator
    r"AXLE", r"(^|_)PIN(_|$)", r"SHAFT", r"BOLT", r"STUD", r"SCREW", r"PLUNGER", r"PIVOT",
    r"BALL_JOINT", r"SPACER", r"CONNECTOR", r"EMERGENCY_STOP", r"ULTRASONIC", r"PRESSURE_SENSOR",
    r"WATER_LEVEL_PROBE", r"ANTENNA", r"CABLE_", r"^TRACK_MOTOR_[LR]$", r"RECEPTACLE", r"BEARING", r"NUT",
]
NEVER_HOST = re.compile(r"TRACK_LINK")

STATES = [
    dict(j1=0, j2=0, rs=0, fs=0, legs=0, rails=0, cover="on"),
    dict(j1=1, j2=1, rs=1, fs=1, legs=1, rails=1, cover="on"),
    dict(j1=0.5, j2=0.3, rs=0.7, fs=0.4, legs=0.6, rails=0.3, cover="on"),
]


def rank(name):
    for i, pat in enumerate(PEN_RANK):
        if re.search(pat, name):
            return len(PEN_RANK) - i
    return 0


def drill(parts, min_vol=5.0, verbose=False):
    Fs = [frames(s) for s in STATES]
    world = []
    for p in parts:
        Ms = [F[p.frame] @ p.M for F in Fs]
        world.append(Ms)
    # candidate pairs from state 0 and 1 bounding boxes
    bbs = []
    for p, Ms in zip(parts, world):
        bbs.append([_bbox(p.shape.moved(to_loc(M)).wrapped) for M in Ms[:2]])
    n = len(parts)
    cuts = 0
    for i in range(n):
        ri = rank(parts[i].name)
        for j in range(n):
            if i == j:
                continue
            rj = rank(parts[j].name)
            if rj <= ri or NEVER_HOST.search(parts[i].name):
                continue            # j must out-rank i to cut it (i is host)
            hit = False
            for s in range(2):
                a, b = bbs[i][s], bbs[j][s]
                if np.all(a[:3] < b[3:]) and np.all(b[:3] < a[3:]):
                    hit = True
            if not hit:
                continue
            host, pen = parts[i], parts[j]
            # relative pose of the penetrator in the host frame, per state
            rel = [np.linalg.inv(world[i][s]) @ world[j][s] for s in range(3)]
            static = all(np.allclose(rel[0], r, atol=1e-6) for r in rel[1:])
            if not static:
                bb = [_bbox(pen.shape.moved(to_loc(r)).wrapped) for r in rel]
                static = all(np.allclose(bb[0], b, atol=0.6) for b in bb[1:])
            if not static:
                continue
            tool = pen.shape.moved(to_loc(rel[0]))
            op = BRepAlgoAPI_Common(host.shape.wrapped, tool.wrapped)
            if not op.IsDone() or _vol(op.Shape()) < min_vol:
                continue
            host.shape = host.shape.cut(tool)
            cuts += 1
            if verbose:
                print("  drill", host.name, "<-", pen.name)
    return cuts
