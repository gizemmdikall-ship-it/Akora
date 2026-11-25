"""Post: captions, titles, dimension lines, end card; 12 -> 24 fps interpolation; mux soundtrack.

    python anim/blender/post.py            -> output/film/AKORA_tanitim_filmi.mp4
    python anim/blender/post.py preview N  -> one composited frame for checking
"""
import glob
import json
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
FILM = os.path.join(ROOT, "output", "film")
FRAMES = os.path.join(FILM, "frames")
COMP = os.path.join(FILM, "comp")
FPS, N = 12, 1380
W, H = 1280, 720
F = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
ORANGE = (242, 92, 10)
SCENES = [(0, 12, "01", "Afet senaryosu"), (12, 25, "02", "AKORA"), (25, 50, "03", "Açılım mekanizması"),
          (50, 65, "04", "Ölçüler"), (65, 85, "05", "Güvenli tahliye"), (85, 100, "06", "Kapanma"), (100, 111, "07", "AKORA")]


def font(p, s):
    return ImageFont.truetype(p, s)


def wrap(d, text, f, width):
    words, lines, cur = text.split(), [], ""
    for w in words:
        test = (cur + " " + w).strip()
        if d.textlength(test, font=f) <= width:
            cur = test
        else:
            lines.append(cur)
            cur = w
    lines.append(cur)
    return lines


def fade(t, a, b, f=0.4):
    return max(0.0, min(1.0, (t - a) / f, (b - t) / f))


class Post:
    def __init__(self):
        self.caps = json.load(open(os.path.join(FILM, "audio", "captions.json"), encoding="utf-8"))
        self.dims = {}
        for p in glob.glob(os.path.join(FILM, "overlay_*.json")):
            self.dims.update({int(k): v for k, v in json.load(open(p)).items()})
        self.fc, self.fs, self.flogo, self.fsc = font(F, 23), font(FB, 15), font(FB, 30), font(F, 15)

    def compose(self, i):
        t = i / FPS
        src = os.path.join(FRAMES, f"f_{i:05d}.jpg")
        if t >= 111 or not os.path.exists(src):
            return self.end_card(t)
        im = Image.open(src).convert("RGBA")
        ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        # logo
        a = int(255 * fade(t, 1.0, 110.6, 0.8))
        d.text((32, 24), "AKORA", font=self.flogo, fill=(255, 255, 255, a))
        d.rectangle([33, 62, 88, 65], fill=(*ORANGE, a))
        d.text((32, 70), "Afet ve İnsan Tahliye Robotu", font=self.fsc, fill=(235, 235, 235, a))
        # scene title (top right)
        for s0, s1, num, name in SCENES:
            if s0 <= t < s1:
                al = int(255 * fade(t, s0 + 0.2, s1 - 0.1, 0.5))
                txt = f"{num}  {name}"
                tw = d.textlength(txt, font=self.fs)
                d.rectangle([W - 40 - tw - 16, 26, W - 32, 56], fill=(10, 12, 15, int(al * 0.6)))
                d.rectangle([W - 40 - tw - 16, 26, W - 40 - tw - 12, 56], fill=(*ORANGE, al))
                d.text((W - 32 - tw - 8, 33), txt, font=self.fs, fill=(255, 255, 255, al))
        # dimension lines (scene 4)
        if i in self.dims:
            for dm in self.dims[i]:
                (ax, ay), (bx, by) = dm["a"], dm["b"]
                A, B = (ax * W, (1 - ay) * H), (bx * W, (1 - by) * H)
                col = (255, 255, 255, 235)
                d.line([A, B], fill=col, width=2)
                vx, vy = B[0] - A[0], B[1] - A[1]
                L = max(1.0, (vx * vx + vy * vy) ** 0.5)
                nx, ny = -vy / L * 9, vx / L * 9
                for P in (A, B):
                    d.line([(P[0] - nx, P[1] - ny), (P[0] + nx, P[1] + ny)], fill=col, width=2)
                mx, my = (A[0] + B[0]) / 2, (A[1] + B[1]) / 2
                tw = d.textlength(dm["label"], font=self.fs)
                ox, oy = (0, 16) if abs(vx) > abs(vy) else (tw / 2 + 18, 0)
                d.rectangle([mx + ox - tw / 2 - 8, my + oy - 11, mx + ox + tw / 2 + 8, my + oy + 11], fill=(10, 12, 15, 190))
                d.text((mx + ox - tw / 2, my + oy - 9), dm["label"], font=self.fs, fill=(255, 255, 255, 255))
        # caption (synced with narration)
        for c in self.caps:
            if c["start"] - 0.2 <= t < c["end"] + 0.3:
                al = fade(t, c["start"] - 0.2, c["end"] + 0.3, 0.3)
                lines = wrap(d, c["text"], self.fc, 980)
                lh = 31
                hbox = lh * len(lines) + 22
                y0 = H - 46 - hbox
                widest = max(d.textlength(L, font=self.fc) for L in lines)
                d.rounded_rectangle([W / 2 - widest / 2 - 22, y0, W / 2 + widest / 2 + 22, y0 + hbox], 8,
                                    fill=(8, 10, 12, int(175 * al)))
                for k, L in enumerate(lines):
                    tw = d.textlength(L, font=self.fc)
                    d.text((W / 2 - tw / 2, y0 + 11 + k * lh), L, font=self.fc, fill=(255, 255, 255, int(255 * al)))
        im = Image.alpha_composite(im, ov).convert("RGB")
        if t < 1.0:                                   # fade in from black
            im = Image.blend(Image.new("RGB", im.size), im, t)
        return im

    def end_card(self, t):
        im = Image.new("RGB", (W, H), (6, 7, 9))
        d = ImageDraw.Draw(im)
        a = fade(t, 111.2, 200, 1.0)
        c = lambda rgb: tuple(int(v * a) for v in rgb)
        fl = font(FB, 96)
        tw = d.textlength("AKORA", font=fl)
        d.text((W / 2 - tw / 2 + 6, 250), "AKORA", font=fl, fill=c((255, 255, 255)))
        d.rectangle([W / 2 - 60, 370, W / 2 + 60, 376], fill=c(ORANGE))
        fs = font(F, 30)
        s = "Afet anında güvenli geçiş için."
        tw = d.textlength(s, font=fs)
        d.text((W / 2 - tw / 2, 400), s, font=fs, fill=c((225, 225, 225)))
        f2 = font(F, 17)
        s2 = "Afet ve İnsan Tahliye Robotu  ·  6 m tahliye yolu  ·  90 cm yürüyüş yüksekliği  ·  100 cm korkuluk"
        tw = d.textlength(s2, font=f2)
        d.text((W / 2 - tw / 2, 470), s2, font=f2, fill=c((150, 150, 150)))
        return im


def main():
    p = Post()
    if len(sys.argv) > 2 and sys.argv[1] == "preview":
        for k in sys.argv[2].split(","):
            p.compose(int(k)).save(os.path.join(FILM, f"preview_{k}.jpg"), quality=92)
        return
    os.makedirs(COMP, exist_ok=True)
    for i in range(N):
        p.compose(i).save(os.path.join(COMP, f"c_{i:05d}.jpg"), quality=95)
    out = os.path.join(FILM, "AKORA_tanitim_filmi.mp4")
    vf = "minterpolate=fps=24:mi_mode=mci:mc_mode=aobmc:me_mode=bidir:vsbmc=1:scd=fdiff:scd_threshold=8"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(FPS), "-i", os.path.join(COMP, "c_%05d.jpg"),
                    "-i", os.path.join(FILM, "audio", "film_audio.wav"), "-vf", vf, "-c:v", "libx264", "-preset", "slow",
                    "-crf", "18", "-pix_fmt", "yuv420p", "-c:a", "aac", "-b:a", "192k", "-shortest", "-movflags", "+faststart", out],
                   check=True)
    print("written", out)


if __name__ == "__main__":
    main()
