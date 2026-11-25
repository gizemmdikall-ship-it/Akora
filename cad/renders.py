"""Technical product renders (three.js / headless Chromium) + stage boards.

    python renders.py      (after build.py has produced output/glb)
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from akora import drill, export, model  # noqa: E402
from akora.kinematics import CONFIGS  # noqa: E402
from akora.model import Part  # noqa: E402

OUT = os.path.join(HERE, "output")
GLB = os.path.join(OUT, "glb")
REN = os.path.join(OUT, "renders")
THREE = os.environ.get("THREE_DIR", os.path.join(HERE, "tools", "node_modules", "three"))


def g(x, y, z):
    """world cm (X fwd, Y left, Z up) -> glTF cm (Y up)."""
    return [x, z, -y]


def job(name, glb, cam, target, fov=30, w=2000, h=1250, **kw):
    d = dict(glb=glb, out=os.path.join(REN, name + ".png"), cam=g(*cam), target=g(*target), fov=fov, w=w, h=h)
    d.update(kw)
    return d


def section_glb():
    """Cut-away of the top-cover interlock through a quarter-turn fastener (assembled + lifted)."""
    import cadquery as cq
    import trimesh
    from trimesh.visual.material import PBRMaterial
    parts = model.build_all()
    drill.drill(parts)
    cfg = dict(name="SEC", state=dict(j1=0, j2=0, rs=0, fs=0, legs=0, rails=0, cover="on"), layers=None,
               hide_groups=set(), explode=False)
    studs = [p for p in parts if p.name.startswith("COVER_QUARTER_TURN_STUD_")]
    stud = max(studs, key=lambda p: abs(p.shape.BoundingBox().ymin + p.shape.BoundingBox().ymax)
               - abs((p.shape.BoundingBox().xmin + p.shape.BoundingBox().xmax) / 2 - 850) / 1000)
    sid = stud.name.rsplit("_", 1)[1]
    bb = stud.shape.BoundingBox()
    xs, ys = (bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2
    side = 1 if ys > 0 else -1
    keep = ("BODY_UPPER_TUB", "COVER_SEAT_RING_FEMALE", "COVER_GASKET_EPDM", "TOP_COVER_SHELL",
            "COVER_QUARTER_TURN_STUD_" + sid, "COVER_RECEPTACLE_" + sid, "BODY_JOINT_BEAD")
    pal = {"BODY_UPPER_TUB": (0.18, 0.18, 0.2), "BODY_JOINT_BEAD": (0.3, 0.3, 0.32),
           "COVER_SEAT_RING_FEMALE": (0.55, 0.56, 0.6), "COVER_GASKET_EPDM": (0.85, 0.12, 0.08),
           "TOP_COVER_SHELL": (0.05, 0.05, 0.055), "COVER_QUARTER_TURN_STUD_" + sid: (0.8, 0.8, 0.82),
           "COVER_RECEPTACLE_" + sid: (0.95, 0.42, 0.05)}
    y0, y1 = (ys - 140, ys + 60) if side > 0 else (ys - 60, ys + 140)
    cut = cq.Workplane("XY").box(160, y1 - y0, 220, centered=False).translate((xs - 160, y0, 600)).val()
    sc = trimesh.Scene()
    for p, M in export.placed(parts, cfg):
        if not any(p.name.startswith(k) for k in keep):
            continue
        sh = p.shape.moved(export.to_loc(M)).intersect(cut)
        for k, dz in ((0, 0), (1, 60)):          # 0: assembled, 1: cover lifted 60 mm
            lift = dz if (k == 1 and p.frame == "cover") else 0
            sh2 = sh.translate(cq.Vector(0, 0, lift)).translate(cq.Vector(0, side * 260 * k, 0))
            vs, ts = sh2.tessellate(0.2, 0.15)
            if not ts:
                continue
            v = np.array([(q.x, q.y, q.z) for q in vs]) / 10.0
            m = trimesh.Trimesh(v, np.array(ts), process=False)
            rgb = pal[next(kk for kk in keep if p.name.startswith(kk))]
            m.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
                baseColorFactor=[*(c ** 2.2 for c in rgb), 1], metallicFactor=0.2, roughnessFactor=0.5, doubleSided=True))
            m.apply_transform(np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float))
            sc.add_geometry(m)
    sc.export(os.path.join(GLB, "AKORA_SECTION_COVER_INTERLOCK.glb"))
    return xs / 10, ys / 10, side


def main():
    os.makedirs(REN, exist_ok=True)
    xs, ys, side = section_glb()
    G = lambda n: f"glb/AKORA_{n}.glb"
    jobs = [
        job("01_kapali_on_perspektif", G("C01_CLOSED_ROBOT"), (430, 310, 190), (85, 0, 72), 30),
        job("02_kapali_yan_perspektif", G("C01_CLOSED_ROBOT"), (95, 560, 120), (85, 0, 78), 30),
        job("03_kapali_ust_perspektif", G("C01_CLOSED_ROBOT"), (330, 280, 520), (85, 0, 62), 30),
        job("04_tam_acik_genel_perspektif", G("C12_FULLY_DEPLOYED"), (650, 660, 380), (230, 0, 60), 34),
        job("05_platform_disli_mekanizmasi_yakin", G("C12_FULLY_DEPLOYED"), (212, 160, 130), (163, 50, 92), 30),
        job("06_palet_sistemi_yakin", G("C02_TOP_COVER_CLOSED"), (175, 300, 85), (90, 90, 33), 30),
        job("07_katli_merdiven_sistemi_yakin", G("C01_CLOSED_ROBOT"), (-150, 190, 215), (25, 0, 112), 30),
        job("08_korkuluk_mekanizmasi_yakin", G("C14a_CLOSING_STAGE_01"), (310, 250, 230), (232, 0, 112), 30),
        job("09_destek_ayaklari_yakin", G("C12_FULLY_DEPLOYED"), (510, 240, 95), (405, 15, 42), 30),
        job("10_ust_kapak_gecmeli_birlesim_kesit", "glb/AKORA_SECTION_COVER_INTERLOCK.glb",
            (xs + 70, ys + side * 4, 92), (xs - 2, ys + side * 13, 71), 30, noGround=True, exposure=1.3),
        job("11_ust_kapak_cikarilmis_ic_yerlesim", G("C04_INTERNALS_VISIBLE"), (250, 230, 330), (85, 0, 40), 32),
        job("12_patlatilmis_montaj_perspektifi", G("C15_EXPLODED"), (760, 920, 560), (230, 0, 170), 36,
            lines=explode_lines()),
    ]
    stage_names = ["C01_CLOSED_ROBOT", "C07_FIRST_PLATFORM_OPEN", "C13a_OPENING_STAGE_01", "C13b_OPENING_STAGE_02",
                   "C08_SECOND_PLATFORM_OPEN", "C13c_OPENING_STAGE_03", "C09_STAIRS_OPEN", "C11_SUPPORT_LEGS_OPEN",
                   "C12_FULLY_DEPLOYED", "C14a_CLOSING_STAGE_01", "C14b_CLOSING_STAGE_02", "C14c_CLOSING_STAGE_03",
                   "C10_RAILINGS_OPEN", "C06_PLATFORM_FOLDED", "C03_TOP_COVER_REMOVED", "C05_TRACK_DETAIL",
                   "C02_TOP_COVER_CLOSED"]
    for n in stage_names:
        jobs.append(job("stage_" + n, G(n), (520, 860, 340), (215, 0, 95), 32, w=1400, h=900))
    only = sys.argv[sys.argv.index("--only") + 1].split(",") if "--only" in sys.argv else None
    if only:
        jobs = [j for j in jobs if any(o in j["out"] for o in only)]
    with open(os.path.join(OUT, "render_jobs.json"), "w") as f:
        json.dump(jobs, f)
    subprocess.run(["node", os.path.join(HERE, "tools", "render.mjs"), os.path.join(OUT, "render_jobs.json"), THREE, OUT],
                   check=True)
    if not only or any("10_ust" in o for o in only):
        legend_section()
    boards()


def explode_lines():
    """Dashed assembly axes for the exploded view (glTF cm)."""
    L = []
    for (x, y) in ((10, 50), (160, -50), (10, -50), (160, 50)):
        L.append([g(x, y, 70), g(x, y, 70 + 140)])            # hull -> cover
    for sy in (1, -1):
        L.append([g(85, sy * 63, 35), g(85, sy * 155, 35)])    # hull -> tracks
    for x in (60, 150):
        L.append([g(x, 38, 80), g(x, 38, 280)])               # pedestals -> platform
    return L


def legend_section():
    from PIL import Image, ImageDraw, ImageFont
    FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    f = ImageFont.truetype(FONT, 30)
    fb = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 36)
    pth = os.path.join(REN, "10_ust_kapak_gecmeli_birlesim_kesit.png")
    im = Image.open(pth).convert("RGB")
    d = ImageDraw.Draw(im)
    d.text((40, 30), "Üst kapak geçmeli birleşimi - kesit (sol: takılı, sağ: kapak 60 mm kaldırılmış)", fill=(20, 20, 20), font=fb)
    items = [((13, 13, 14), "Üst kapak (6 mm cidar) + erkek dudak 9 mm"),
             ((140, 143, 153), "Dişi oturma halkası: 12 mm kanal, 18 mm derinlik"),
             ((217, 31, 20), "EPDM conta 10x8 mm - dudak 2 mm (%25) sıkıştırır"),
             ((204, 204, 209), "Çeyrek tur (captive) bağlantı pimi, 10 adet"),
             ((242, 107, 13), "Paslanmaz alıcı (receptacle)"),
             ((46, 46, 51), "Gövde üst cidarı (8 mm)")]
    y = im.height - 60 * len(items) - 40
    d.rectangle([30, y - 20, 1010, im.height - 25], fill=(255, 255, 255), outline=(180, 180, 180))
    for c, t in items:
        d.rectangle([50, y, 90, y + 36], fill=c, outline=(90, 90, 90))
        d.text((110, y + 2), t, fill=(25, 25, 25), font=f)
        y += 60
    d.text((40, 90), "Kapak çevresel dudakla kendini hizalar; sızdırmazlık (conta) ve sabitleme (çeyrek tur pimleri, "
           "4 hizalama pimi) ayrı işlevlerdir.", fill=(40, 40, 40), font=f)
    im.save(pth)


def boards():
    from PIL import Image, ImageDraw, ImageFont
    FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(FONT, 30)
    titles = {c["name"]: c["title"] for c in CONFIGS}

    def board(names, out, heading):
        ims = [Image.open(os.path.join(REN, "stage_" + n + ".png")).convert("RGB") for n in names]
        w, h = ims[0].size
        cols = 3
        rows = (len(ims) + cols - 1) // cols
        B = Image.new("RGB", (cols * w, rows * h + 90), (245, 246, 248))
        d = ImageDraw.Draw(B)
        d.text((30, 25), heading, fill=(20, 20, 20), font=ImageFont.truetype(FONT, 44))
        for i, (n, im) in enumerate(zip(names, ims)):
            x, y = (i % cols) * w, (i // cols) * h + 90
            B.paste(im, (x, y))
            d.rectangle([x + 20, y + 20, x + 60, y + 60], fill=(242, 92, 10))
            d.text((x + 30, y + 22), str(i + 1), fill=(255, 255, 255), font=font)
            d.text((x + 75, y + 24), titles[n], fill=(25, 25, 25), font=font)
        B.save(os.path.join(REN, out), optimize=True)

    board(["C01_CLOSED_ROBOT", "C07_FIRST_PLATFORM_OPEN", "C13a_OPENING_STAGE_01", "C13b_OPENING_STAGE_02",
           "C08_SECOND_PLATFORM_OPEN", "C13c_OPENING_STAGE_03", "C09_STAIRS_OPEN", "C11_SUPPORT_LEGS_OPEN",
           "C12_FULLY_DEPLOYED"], "13_acilma_asamalari.png", "AKORA - Açılma aşamaları (aynı geometri, kinematik konumlar)")
    board(["C12_FULLY_DEPLOYED", "C14a_CLOSING_STAGE_01", "C14b_CLOSING_STAGE_02", "C08_SECOND_PLATFORM_OPEN",
           "C14c_CLOSING_STAGE_03", "C01_CLOSED_ROBOT"], "14_kapanma_asamalari.png", "AKORA - Kapanma aşamaları (ters sıra)")


if __name__ == "__main__":
    main()
