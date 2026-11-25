"""AKORA build pipeline.

    python build.py            # everything
    python build.py --fast     # skip STEP + 3DM (GLB + renders + reports only)

Outputs (cad/output):
    step/AKORA_<config>.step.zip   NURBS B-rep STEP AP214, unit = cm, assembly tree = layer tree
    rhino/AKORA_master.3dm         Rhino 8 file, unit = cm, layers 01..11, all configurations
    glb/AKORA_<config>.glb         render / web-viewer meshes (cm, Y-up)
    reports/                       BOM, clash report, solid validity, dimension check
    renders/                       technical product renders
"""
import csv
import json
import math
import os
import subprocess
import sys
import time
import zipfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from akora import clash, drill, export, model  # noqa: E402
from akora import params as P  # noqa: E402
from akora.kinematics import CONFIGS, frames  # noqa: E402
from akora.materials import MATERIALS  # noqa: E402

OUT = os.path.join(HERE, "output")
DENSITY = {  # g/cm3 for mass estimate
    "polymer_black": 1.36, "polymer_dark": 1.36, "orange": 7.85, "rubber": 1.2, "antislip": 2.7,
    "frame_black": 2.7, "brushed": 7.9, "stainless": 7.9, "aluminum": 2.7, "steel_dark": 7.85,
    "silver_cap": 2.7, "epdm": 1.1, "pcb": 1.9, "battery": 2.3, "glass": 1.2, "reflector": 1.2,
    "red": 1.3, "cable": 1.8, "led": 1.2,
}
ORANGE_ALU = ("GEAR", "LINK_PLATE", "PEDESTAL", "BRACKET", "RAIL_HINGE", "COLLAR", "TOP_BEAM", "SWING_ARM",
              "FOOT_PLATE", "MOUNT", "SEAT", "KNUCKLE", "PIVOT_PLATE", "TENSIONER", "OUTER_RING")


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def ensure(*d):
    p = os.path.join(OUT, *d)
    os.makedirs(p, exist_ok=True)
    return p


# ------------------------------------------------------------------ reports
def solid_report(parts):
    from OCP.BRepCheck import BRepCheck_Analyzer
    rows = []
    for p in parts:
        sh = p.shape
        solids = sh.Solids()
        closed = all(s.isValid() for s in solids)
        rows.append((p.name, len(solids), BRepCheck_Analyzer(sh.wrapped).IsValid(), closed))
    return rows


def bom(parts, path):
    from akora.clash import _vol
    groups = {}
    for p in parts:
        base = p.name
        key = (p.layer, p.mat, round(_vol(p.shape.wrapped) / 1000.0, 1))
        groups.setdefault(key, []).append(p.name)
    rows = []
    total = 0.0
    for (layer, mat, vol_cm3), names in sorted(groups.items()):
        dens = DENSITY[mat]
        if mat == "orange" and any(k in names[0] for k in ORANGE_ALU):
            dens = 2.7 if any(k in names[0] for k in ("RAIL_HINGE", "COLLAR", "KNUCKLE", "PEDESTAL", "SEAT", "FOOT_PLATE")) else 7.85
        mass = vol_cm3 * dens / 1000.0
        total += mass * len(names)
        rows.append(["/".join(layer), names[0] if len(names) == 1 else names[0] + f" (+{len(names) - 1})",
                     len(names), MATERIALS[mat][4], vol_cm3, round(mass, 3), round(mass * len(names), 3)])
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Katman", "Parça", "Adet", "Malzeme", "Birim hacim (cm3)", "Birim kütle (kg)", "Toplam kütle (kg)"])
        w.writerows(rows)
        w.writerow([])
        w.writerow(["TOPLAM (tahmini, iç boşluklar dahil hesaplanan katı hacimden)", "", sum(r[2] for r in rows), "", "", "", round(total, 1)])
    return total, len(rows)


def dimension_check(parts):
    """Measure the as-modelled overall dimensions in the key configurations (cm)."""
    out = {}
    for name in ("C01_CLOSED_ROBOT", "C12_FULLY_DEPLOYED"):
        cfg = next(c for c in CONFIGS if c["name"] == name)
        lo = np.array([1e9] * 3)
        hi = -lo
        for p, M in export.placed(parts, cfg):
            v, f = export.world_mesh(p, M)
            if len(v):
                lo = np.minimum(lo, v.min(0))
                hi = np.maximum(hi, v.max(0))
        out[name] = dict(length=round((hi[0] - lo[0]) / 10, 1), width=round((hi[1] - lo[1]) / 10, 1),
                         height=round(hi[2] / 10, 1), x_min=round(lo[0] / 10, 1), x_max=round(hi[0] / 10, 1))
    F = frames(next(c for c in CONFIGS if c["name"] == "C12_FULLY_DEPLOYED")["state"])
    out["deck_height_cm"] = round(F["P2"][2, 3] / 10, 1)
    out["P3_deck_height_cm"] = round(F["P3"][2, 3] / 10, 1)
    return out


# ------------------------------------------------------------------ main
def main():
    fast = "--fast" in sys.argv
    t0 = time.time()
    log("building parts")
    parts = model.build_all()
    log(len(parts), "parts; drilling holes / pockets")
    n_cuts = drill.drill(parts)
    log("cuts:", n_cuts)
    rep = ensure("reports")

    # --- validity
    sr = solid_report(parts)
    bad = [r for r in sr if not r[2]]
    log("invalid shapes:", [b[0] for b in bad])

    # --- GLB per configuration
    gdir = ensure("glb")
    for c in CONFIGS:
        export.write_glb(parts, c, os.path.join(gdir, f"AKORA_{c['name']}.glb"))
    log("glb done")

    # --- clash report (all configurations + 41-sample opening sweep), 4 worker processes
    import multiprocessing as mp
    global _PARTS
    _PARTS = parts
    jobs = [c for c in CONFIGS if not c["explode"]]
    jobs += [dict(name=f"SWEEP_{i:02d}", title=str(i / 40), state=opening_path(i / 40), layers=None,
                  hide_groups=set(), explode=False) for i in range(41)]
    with mp.get_context("fork").Pool(4) as pool:
        res = pool.map(_clash_job, jobs)
    clash_rows, sweep = [], []
    for c, (real, thr) in zip(jobs, res):
        if c["name"].startswith("SWEEP"):
            sweep.append((round(float(c["title"]), 3), c["state"], len(real), real[:5]))
        else:
            clash_rows.append((c["name"], c["title"], len(real), len(thr), real[:20], thr[:20]))
            log("clash", c["name"], len(real), len(thr))
    log("sweep done, max clashes:", max(s[2] for s in sweep))
    write_clash_md(os.path.join(rep, "clash_report.md"), clash_rows, sweep)

    # --- BOM + dimensions
    total_mass, n_rows = bom(parts, os.path.join(rep, "BOM_parca_listesi.csv"))
    dims = dimension_check(parts)
    dims["estimated_mass_kg"] = round(total_mass, 1)
    dims["part_count"] = len(parts)
    dims["drilled_features"] = n_cuts
    dims["invalid_shapes"] = [b[0] for b in bad]
    with open(os.path.join(rep, "model_summary.json"), "w", encoding="utf-8") as f:
        json.dump(dims, f, indent=2, ensure_ascii=False)
    with open(os.path.join(rep, "solid_validity.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["part", "solids", "BRepCheck_valid", "all_solids_valid"])
        w.writerows(sr)
    log("summary", dims)

    if not fast:
        # --- STEP per configuration (zipped)
        sdir = ensure("step")
        tmp = os.path.join(sdir, "_tmp.step")
        for c in CONFIGS:
            export.write_step(parts, c, tmp)
            zp = os.path.join(sdir, f"AKORA_{c['name']}.step.zip")
            with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
                z.write(tmp, f"AKORA_{c['name']}.step")
            log("step", c["name"], round(os.path.getsize(zp) / 1e6, 1), "MB zip")
        os.remove(tmp)
        # --- Rhino master
        rdir = ensure("rhino")
        main_cfg = next(c for c in CONFIGS if c["name"] == "C01_CLOSED_ROBOT")
        asm = []
        for c in CONFIGS:
            if c["explode"]:
                continue
            c = dict(c)
            c["rhino_layer"] = c["name"]
            asm.append(c)
        exploded = next(c for c in CONFIGS if c["explode"])
        export.write_3dm(parts, os.path.join(rdir, "AKORA_master.3dm"), main_cfg, asm, exploded)
        log("3dm", round(os.path.getsize(os.path.join(rdir, "AKORA_master.3dm")) / 1e6, 1), "MB")

    log("done in", round(time.time() - t0), "s")


_PARTS = None


def _clash_job(cfg):
    return clash.check(_PARTS, cfg)


def opening_path(t):
    """Continuous opening sequence used for the kinematic sweep (0 = closed, 1 = deployed)."""
    def seg(a, b):
        return max(0.0, min(1.0, (t - a) / (b - a)))
    j1 = 0.5 * seg(0.0, 0.2) + 0.5 * seg(0.35, 0.55)
    j2 = 0.5 * seg(0.2, 0.35) + 0.5 * seg(0.35, 0.55)
    return dict(j1=j1, j2=j2, rs=seg(0.55, 0.7), fs=seg(0.55, 0.7), legs=seg(0.7, 0.85), rails=seg(0.85, 1.0), cover="on")


def write_clash_md(path, rows, sweep):
    L = ["# AKORA çakışma (interference) raporu", "",
         "Yöntem: her konfigürasyonda yerleştirilmiş tüm katı çiftleri için sınır kutusu ön elemesi, ardından "
         "OpenCASCADE Boolean `Common` hacmi. 0,2 cm³ üzeri kesişim **çakışma** sayılır. "
         "Bağlantı elemanlarının geçtiği delikler `drill.py` ile gerçek delik olarak modellenmiştir. "
         "Tasarım gereği izinli tek temas: üst kapak dudağının EPDM contayı 2 mm (%25) sıkıştırması.", "",
         "| Konfigürasyon | Açıklama | Çakışma | Bağlantı elemanı geçişi |", "|---|---|---:|---:|"]
    for name, title, n, nt, real, thr in rows:
        L.append(f"| {name} | {title} | {n} | {nt} |")
    L += ["", "## Açılma hareketi taraması (41 ara konum)", "",
          "| t | j1 | j2 | arka merd. | ön merd. | ayak | korkuluk | çakışma |", "|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for t, st, n, real in sweep:
        L.append(f"| {t:.3f} | {st['j1']:.2f} | {st['j2']:.2f} | {st['rs']:.2f} | {st['fs']:.2f} | {st['legs']:.2f} | {st['rails']:.2f} | {n} |")
    det = [(n, r) for n, _, c, _, r, _ in rows if c] + [(f"sweep t={t}", r) for t, _, c, r in sweep if c]
    if det:
        L += ["", "## Ayrıntı"]
        for n, r in det:
            L.append(f"- **{n}**: " + "; ".join(f"{a} × {b} ({v} cm³)" for a, b, v in r))
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
