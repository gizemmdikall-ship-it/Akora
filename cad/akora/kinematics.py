"""Kinematic frames and the 15+ model configurations.

State values are 0 = folded / stowed, 1 = deployed.
  j1    : P1 -> P2 geared hinge (deck level, P2 folds over P1 upside-down)
  j2    : P2 -> P3 geared hinge (underside level, Z-fold)
  rs    : rear stair (nested step frames -> staircase)
  fs    : front ramp-stair (folded on P3 -> resting on the ground)
  legs  : support legs under the end of P3 (fold + telescope)
  rails : guardrails (fold up, then telescope)
"""
import numpy as np

from . import params as P
from .geom import R, T, chain, lerp
from .model import FRONT_PIVOT_DX, LEG_HINGE_Z, REAR_TREAD_D

CD = P.HINGE_CD


def _clamp(v):
    return max(0.0, min(1.0, v))


def frames(st):
    F = {"world": np.eye(4)}
    F["cover_seal"] = np.eye(4)
    F["cover"] = T(0, -2700, -(P.HULL_TOP - P.LIP_LEN)) if st.get("cover") == "off" else np.eye(4)

    # ---- platform chain
    F["P1"] = T(P.P1_X0, 0, P.DECK_Z)
    phi1 = 90.0 * (1 - st["j1"])
    e = P.J1_AXIS_DZ
    F["J1_LINK"] = chain(F["P1"], T(P.P1_LEN, 0, e), R("y", -phi1))
    F["P2_AXIS"] = chain(F["J1_LINK"], T(CD, 0, 0), R("y", -phi1))
    F["P2"] = chain(F["P2_AXIS"], T(0, 0, -e))
    phi2 = 90.0 * (1 - st["j2"])
    F["J2_LINK"] = chain(F["P2"], T(P.P2_LEN, 0, -P.FRAME_H), R("y", phi2))
    F["P3_AXIS"] = chain(F["J2_LINK"], T(CD, 0, 0), R("y", phi2))
    F["P3"] = chain(F["P3_AXIS"], T(0, 0, P.FRAME_H))

    # ---- front stair
    th = lerp(P.FRONT_STAIR_ANGLE_STOWED, P.FRONT_STAIR_ANGLE_DEPLOYED, st["fs"])
    F["FS"] = chain(F["P3"], T(P.P3_LEN + FRONT_PIVOT_DX, 0, P.FRONT_PIVOT_DZ), R("y", th))

    # ---- support legs: fold first, then telescope down
    lf = _clamp(st["legs"] * 2)
    le = _clamp(st["legs"] * 2 - 1)
    hinge_z = LEG_HINGE_Z
    ext = (P.DECK_Z + hinge_z) - 590          # inner travel needed to put the pad on the ground
    for sy, tag in ((1, "L"), (-1, "R")):
        F[f"LEG_{tag}"] = chain(F["P3"], T(P.P3_LEN - 110, sy * 350, hinge_z), R("y", 90 * (1 - lf)))
        F[f"LEG_{tag}_INNER"] = chain(F[f"LEG_{tag}"], T(0, 0, -ext * le))

    # ---- railings: fold up first, then telescope
    rf = _clamp(st["rails"] * 2)
    rt = _clamp(st["rails"] * 2 - 1)
    for mod in ("P1", "P2", "P3"):
        for sy, tag in ((1, "L"), (-1, "R")):
            hz = P.RAIL_HINGE_Z[0] if sy > 0 else P.RAIL_HINGE_Z[1]
            ang = 90.0 * (1 - rf) * (1 if sy > 0 else -1)
            F[f"RAIL_{mod}_{tag}"] = chain(F[mod], T(0, sy * P.RAIL_SPACING / 2, hz), R("x", ang))
            # top surface of the top rail lands exactly at DEPLOYED_HEIGHT on both sides
            ext = (P.RAIL_HEIGHT - P.RAIL_R) - hz - 420
            F[f"RAIL_{mod}_{tag}_UP"] = chain(F[f"RAIL_{mod}_{tag}"], T(0, 0, ext * rt))

    # ---- rear landing + nested step frames
    F["LANDING"] = T(P.REAR_LANDING_X0 + 15, 0, P.DECK_Z)
    rs = st["rs"]
    for k in range(4):
        D = np.array([P.REAR_LANDING_X0 - REAR_TREAD_D * k, P.DECK_Z - 180 * (k + 1)])
        Sx = np.array([P.P1_X0 - 60 - 45 * k, P.DECK_Z + 65 + 45 * k])
        W = np.array([D[0], Sx[1]])
        if rs <= 0.5:
            p = Sx + (W - Sx) * (rs / 0.5)
        else:
            p = W + (D - W) * ((rs - 0.5) / 0.5)
        F[f"RS_{k}"] = T(p[0], 0, p[1])
    F["RS_FEET"] = chain(F["RS_3"], T(0, 0, 35 + 120 * (1 - _clamp(rs * 4 - 3))))
    return F


BASE = dict(j1=0, j2=0, rs=0, fs=0, legs=0, rails=0, cover="on")
DEPLOYED = dict(j1=1, j2=1, rs=1, fs=1, legs=1, rails=1, cover="on")

BODY_ONLY = ("01_MAIN_BODY", "02_TRACK_SYSTEM", "07_ELECTRONICS", "08_POWER_SYSTEM", "09_FASTENERS")
PLATFORM_ONLY = ("03_PLATFORM_SYSTEM", "04_STAIR_SYSTEM", "05_GUARDRAIL_SYSTEM", "06_SUPPORT_SYSTEM", "09_FASTENERS")


def _cfg(name, title, layers=None, hide_groups=(), explode=False, **kw):
    st = dict(BASE)
    st.update(kw)
    return dict(name=name, title=title, state=st, layers=layers, hide_groups=set(hide_groups), explode=explode)


CONFIGS = [
    _cfg("C01_CLOSED_ROBOT", "Kapalı AKORA (taşıma konumu)"),
    _cfg("C02_TOP_COVER_CLOSED", "Üst kapağı kapalı gövde (platform paketi sökülü)", layers=BODY_ONLY),
    _cfg("C03_TOP_COVER_REMOVED", "Üst kapak tamamen çıkarılmış (kapak yanda, yerde)", layers=BODY_ONLY, cover="off"),
    _cfg("C04_INTERNALS_VISIBLE", "İç mekanizma görünür (kapak, batarya kapağı, sol palet sökülü)",
         layers=BODY_ONLY, cover="off", hide_groups=("cover", "internal_lid", "track_L")),
    _cfg("C05_TRACK_DETAIL", "Palet mekanizması detayı", layers=("02_TRACK_SYSTEM",)),
    _cfg("C06_PLATFORM_FOLDED", "Platform mekanizması katlanmış", layers=PLATFORM_ONLY),
    _cfg("C07_FIRST_PLATFORM_OPEN", "İlk platform (P2) açıldı: dikey konum, P3 hâlâ katlı", j1=0.5),
    _cfg("C08_SECOND_PLATFORM_OPEN", "İkinci platform (P3) açılmış", j1=1, j2=1),
    _cfg("C09_STAIRS_OPEN", "Merdivenler açılmış", j1=1, j2=1, rs=1, fs=1),
    _cfg("C10_RAILINGS_OPEN", "Korkuluklar açılmış", j1=1, j2=1, rs=1, fs=1, rails=1),
    _cfg("C11_SUPPORT_LEGS_OPEN", "Destek ayakları açılmış", j1=1, j2=1, rs=1, fs=1, legs=1),
    _cfg("C12_FULLY_DEPLOYED", "Tamamen açılmış tahliye robotu", **{k: v for k, v in DEPLOYED.items()}),
    _cfg("C13a_OPENING_STAGE_01", "Açılma 1: P2 dikey kule, P3 yatay (PDF açılım adımı 2)", j1=0.5, j2=0.5),
    _cfg("C13b_OPENING_STAGE_02", "Açılma 2: platformlar yataya iniyor", j1=0.85, j2=0.8),
    _cfg("C13c_OPENING_STAGE_03", "Açılma 3: merdivenler ve ayaklar açılıyor", j1=1, j2=1, rs=0.5, fs=0.5, legs=0.5),
    _cfg("C14a_CLOSING_STAGE_01", "Kapanma 1: korkuluklar katlanıyor", j1=1, j2=1, rs=1, fs=1, legs=1, rails=0.4),
    _cfg("C14b_CLOSING_STAGE_02", "Kapanma 2: ayaklar toplanıyor, merdivenler kalkıyor", j1=1, j2=1, rs=0.6, fs=0.4, legs=0.3),
    _cfg("C14c_CLOSING_STAGE_03", "Kapanma 3: platformlar katlanıyor", j1=0.3, j2=0.25),
    _cfg("C15_EXPLODED", "Patlatılmış montaj perspektifi", explode=True, **{k: v for k, v in DEPLOYED.items()}),
]

# explode offsets per group (mm) - along assembly directions
EXPLODE = {
    "hull": (0, 0, 0), "frame": (0, 0, 350), "internal": (0, 0, 650), "internal_lid": (0, 0, 900),
    "gasket": (0, 0, 1150), "cover": (0, 0, 1400), "panel_rear": (-450, 0, 0),
    "track_L": (0, 900, 0), "track_R": (0, -900, 0),
    "P1": (0, 0, 2000), "J1_A": (0, 0, 2000), "J1_L": (0, 0, 2000), "J1_B": (0, 0, 2000),
    "P2": (0, 0, 2000), "J2_A": (0, 0, 2000), "J2_L": (0, 0, 2000), "J2_B": (0, 0, 2000), "P3": (0, 0, 2000),
    "rail_P1": (0, 0, 2700), "rail_P2": (0, 0, 2700), "rail_P3": (0, 0, 2700),
    "landing": (-200, 0, 2000), "rear_stair": (-700, 0, 2000), "front_stair": (700, 0, 2000),
    "legs": (0, 0, 1500),
}
