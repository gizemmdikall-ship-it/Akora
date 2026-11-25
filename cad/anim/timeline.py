"""Narration + timeline for the AKORA product animation.

Synthesises each narration line with Piper (Turkish voice), measures it, and lays out
the step windows so every motion fits inside the sentence that explains it.

    python anim/timeline.py  ->  output/animation/timeline.json + narration/*.wav + narration.wav
"""
import json
import os
import subprocess
import sys
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "output", "animation")
VOICE = os.environ.get("AKORA_VOICE", os.path.join(ROOT, "anim", "voices", "tr_TR-fettah-medium.onnx"))

# (key, caption number, caption title, caption text (digits), spoken text (words), minimum seconds for the motion)
STEPS = [
    ("intro", "", "AKORA", "Sel yolu ikiye böldü. Karşı kıyıda mahsur kalan insanlar var. AKORA göreve çıkıyor.",
     "Sel, yolu ikiye böldü. Karşı kıyıda mahsur kalan insanlar var. Akora, afet ve insan tahliye robotu, göreve çıkıyor.", 6.0),
    ("drive", "01", "Paletli hareket", "Çift paletli şasi, ıslak ve molozlu zeminde uzaktan kumandayla su kenarına kadar ilerler.",
     "Çift paletli şasi, ıslak ve molozlu zeminde, uzaktan kumandayla su kenarına kadar ilerler.", 6.0),
    ("scan", "02", "Sensörlerle analiz", "Su seviyesi ve mesafe sensörleri ile kamera, su yatağının genişliğini ve karşı kıyıyı ölçer; açılım konumu belirlenir.",
     "Su seviyesi ve mesafe sensörleri ile kamera, su yatağının genişliğini ve karşı kıyıyı ölçer. Açılım konumu belirlenir.", 4.0),
    ("open", "03", "Platform açılımı", "Ø20 cm senkron dişli menteşeler: ikinci platform dikey konuma kalkar, üçüncü platform yataya döner, iki modül birlikte 90 cm yürüyüş seviyesine iner.",
     "Platform açılıyor. Yirmi santimetre çaplı senkron dişli menteşeler sayesinde ikinci platform dikey konuma kalkar, üçüncü platform yataya döner, ve iki modül birlikte doksan santimetrelik yürüyüş seviyesine iner.", 10.0),
    ("stairs", "04", "Merdivenler", "Arkadaki katlanır basamaklar ve öndeki 120 cm rampa merdiven zemine iner.",
     "Arkadaki katlanır basamaklar ve öndeki yüz yirmi santimetrelik rampa merdiven zemine iner.", 4.5),
    ("legs", "05", "Destek ayakları", "Teleskopik destek ayakları karşı kıyıya basar, yükü zemine aktarır ve platformu dengeler.",
     "Teleskopik destek ayakları karşı kıyıya basar, yükü zemine aktarır ve platformu dengeler.", 3.5),
    ("rails", "06", "Korkuluklar", "100 cm yüksekliğindeki reflektörlü korkuluklar kalkar ve kilitlenir. 6 m uzunluğunda güvenli tahliye yolu hazır.",
     "Yüz santimetre yüksekliğindeki reflektörlü korkuluklar kalkar ve kilitlenir. Altı metre uzunluğunda güvenli bir tahliye yolu hazır.", 3.5),
    ("evac", "07", "İnsan tahliyesi", "Mahsur kalanlar suya girmeden, kaymaz platform üzerinden güvenli bölgeye geçer.",
     "Mahsur kalanlar, suya girmeden, kaymaz platform üzerinden güvenli bölgeye geçer.", 0.0),
    ("evac2", "07", "İnsan tahliyesi", "Kaymaz yüzey ve iki yandaki korkuluklar; çocuk ve yaşlı dahil herkes için güvenli geçiş.",
     "Kaymaz yüzey ve iki yandaki korkuluklar, çocuk ve yaşlılar dahil herkes için güvenli bir geçiş sağlar.", 0.0),
    ("close", "08", "Kapanma", "Tahliye tamamlanınca korkuluklar, ayaklar, merdivenler ve platformlar ters sırayla katlanır; robot taşıma konumuna döner.",
     "Tahliye tamamlanınca korkuluklar, ayaklar, merdivenler ve platformlar ters sırayla katlanır. Robot yeniden taşıma konumuna döner.", 10.0),
    ("end", "", "AKORA", "Sel ve taşkında güvenli tahliye yolu.", "Akora. Sel ve taşkında, güvenli tahliye yolu.", 5.0),
]
EVAC_TOTAL = 19.0          # people walking from the far bank to the rescue truck
LEAD, TAIL = 0.35, 0.9     # silence before / after each sentence


def wav_len(p):
    with wave.open(p) as w:
        return w.getnframes() / w.getframerate()


def main():
    nd = os.path.join(OUT, "narration")
    os.makedirs(nd, exist_ok=True)
    t = 0.0
    tl = []
    for key, num, title, cap, spoken, mn in STEPS:
        wp = os.path.join(nd, f"{key}.wav")
        subprocess.run([sys.executable, "-m", "piper", "-m", VOICE, "-f", wp, "--length-scale", "1.04",
                        "--sentence-silence", "0.25"], input=spoken.encode(), check=True, capture_output=True)
        d = wav_len(wp)
        dur = max(mn, LEAD + d + TAIL)
        tl.append(dict(key=key, num=num, title=title, text=cap, start=round(t, 3), end=round(t + dur, 3),
                       voice_start=round(t + LEAD, 3), voice_len=round(d, 3), wav=f"narration/{key}.wav"))
        t += dur
    # the evacuation needs enough time for the walk: stretch evac2
    ev = [s for s in tl if s["key"] in ("evac", "evac2")]
    need = EVAC_TOTAL - (ev[-1]["end"] - ev[0]["start"])
    if need > 0:
        i = tl.index(ev[-1])
        tl[i]["end"] += need
        for s in tl[i + 1:]:
            for k in ("start", "end", "voice_start"):
                s[k] += need
    total = tl[-1]["end"]
    with open(os.path.join(OUT, "timeline.json"), "w", encoding="utf-8") as f:
        json.dump(dict(total=round(total, 3), steps=tl), f, ensure_ascii=False, indent=1)
    # mix narration onto a rain bed
    inputs, filt = [], []
    for i, s in enumerate(tl):
        inputs += ["-i", os.path.join(OUT, s["wav"])]
        ms = int(s["voice_start"] * 1000)
        filt.append(f"[{i}:a]aresample=48000,adelay={ms}|{ms},volume=1.6[v{i}]")
    n = len(tl)
    filt.append(f"anoisesrc=color=pink:amplitude=0.09:duration={total:.2f}:sample_rate=48000,lowpass=f=4200,highpass=f=180[rain]")
    filt.append("".join(f"[v{i}]" for i in range(n)) + f"[rain]amix=inputs={n + 1}:normalize=0:duration=longest,"
                f"atrim=0:{total:.2f},afade=t=in:d=1.5,afade=t=out:st={total - 2:.2f}:d=2,alimiter=limit=0.95[out]")
    subprocess.run(["ffmpeg", "-v", "error", "-y", *inputs, "-filter_complex", ";".join(filt), "-map", "[out]",
                    "-ac", "2", os.path.join(OUT, "narration.wav")], check=True)
    for s in tl:
        print(f"{s['start']:6.1f}-{s['end']:6.1f}  {s['key']:7s} voice {s['voice_len']:.1f}s")
    print("total", round(total, 1), "s")


if __name__ == "__main__":
    main()
