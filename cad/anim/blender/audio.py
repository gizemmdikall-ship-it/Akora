"""Film soundtrack: Turkish narration (Piper), procedural music bed and mechanism SFX synced to film.py.

    python anim/blender/audio.py  ->  output/film/audio/film_audio.wav (+ narration.json for captions)
"""
import json
import os
import subprocess
import sys
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, "output", "film", "audio")
VOICE = os.path.join(ROOT, "anim", "voices", "tr_TR-fettah-medium.onnx")
SR = 48000
DUR = 115.0

# (start s, caption text, spoken text)
LINES = [
    (1.0, "Afetler sonrasında en büyük sorunlardan biri, güvenli ulaşım ve tahliye yollarının ortadan kalkmasıdır. Özellikle sel felaketlerinde, insanların güvenli bölgelere ulaşması ciddi bir zorluğa dönüşür.", None),
    (13.0, "AKORA, afet bölgelerinde tahliye sürecine destek olmak için geliştirilen modüler bir arama-kurtarma robotudur. Paletli hareket sistemi sayesinde zorlu zeminlerde ilerleyerek ihtiyaç duyulan noktaya ulaşmayı hedefler.",
     "Akora, afet bölgelerinde tahliye sürecine destek olmak için geliştirilen modüler bir arama kurtarma robotudur. Paletli hareket sistemi sayesinde zorlu zeminlerde ilerleyerek ihtiyaç duyulan noktaya ulaşmayı hedefler."),
    (26.0, "AKORA’nın en önemli özelliklerinden biri, gövdesinde kompakt biçimde taşıdığı açılabilir tahliye mekanizmasıdır. Çok parçalı platform sistemi, ihtiyaç duyulan noktada kontrollü biçimde açılarak geçici bir geçiş alanı oluşturur.",
     "Akoranın en önemli özelliklerinden biri, gövdesinde kompakt biçimde taşıdığı açılabilir tahliye mekanizmasıdır. Çok parçalı platform sistemi, ihtiyaç duyulan noktada kontrollü biçimde açılarak geçici bir geçiş alanı oluşturur."),
    (40.5, "Ø20 cm dişli menteşeler platformları sırayla döndürür; merdivenler zemine iner, destek ayakları ve korkuluklar kilitlenir.",
     "Yirmi santimetre çaplı dişli menteşeler platformları sırayla döndürür. Merdivenler zemine iner, destek ayakları ve korkuluklar kilitlenir."),
    (51.0, "Kompakt taşıma formundaki robot, açıldığında 6 m uzunluğunda bir geçiş oluşturur: 90 cm yürüyüş yüksekliği ve 100 cm korkuluklarla.",
     "Kompakt taşıma formundaki robot, açıldığında altı metre uzunluğunda bir geçiş oluşturur. Doksan santimetre yürüyüş yüksekliği ve yüz santimetrelik korkuluklarla."),
    (66.0, "Destek ayakları ve korkuluk sistemiyle oluşturulan geçiş alanı, kurtarma ekiplerinin ve afetzedelerin daha kontrollü hareket etmesine yardımcı olur. AKORA, afet sonrası tahliye çalışmalarına esnek ve taşınabilir bir çözüm sunmayı amaçlar.",
     "Destek ayakları ve korkuluk sistemiyle oluşturulan geçiş alanı, kurtarma ekiplerinin ve afetzedelerin daha kontrollü hareket etmesine yardımcı olur. Akora, afet sonrası tahliye çalışmalarına esnek ve taşınabilir bir çözüm sunmayı amaçlar."),
    (86.0, "Görev tamamlandığında mekanizma yeniden katlanarak robotun kompakt taşıma formuna dönmesini sağlar. Böylece AKORA farklı görev noktalarına yeniden yönlendirilebilir.",
     "Görev tamamlandığında mekanizma yeniden katlanarak robotun kompakt taşıma formuna dönmesini sağlar. Böylece Akora farklı görev noktalarına yeniden yönlendirilebilir."),
    (108.0, "AKORA – Afet anında güvenli geçiş için.", "Akora. Afet anında, güvenli geçiş için."),
]

# mechanism events (start, end, kind) - same timings as film.py mech_state
MECH = [(28.3, 31.8, "slide"), (32.3, 36.6, "gear"), (37.3, 40.7, "gear"), (41.2, 43.8, "gear"), (44.2, 45.8, "pivot"),
        (46.2, 47.8, "tele"), (48.1, 49.8, "rail"), (85.3, 87.7, "rail"), (88.3, 90.7, "tele"), (91.3, 93.7, "slide"),
        (94.3, 96.4, "gear"), (96.6, 97.8, "gear"), (97.9, 99.7, "gear")]
DRIVE = [(16.0, 19.0, 0.5), (19.0, 24.4, 1.0)]
STREET = [(0, 12), (19, 50), (65, 100)]


def t2i(t):
    return int(round(t * SR))


def tts(text, path):
    subprocess.run([sys.executable, "-m", "piper", "-m", VOICE, "-f", path, "--length-scale", "0.88", "--sentence-silence", "0.25"],
                   input=text.encode(), check=True, capture_output=True)
    with wave.open(path) as w:
        sr = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768
    # resample to SR (linear)
    n = int(len(x) * SR / sr)
    return np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x)


def lowpass(x, fc):
    a = np.exp(-2 * np.pi * fc / SR)
    y = np.empty_like(x)
    acc = 0.0
    # vectorised one-pole via cumulative trick is unstable; use simple loop on decimated signal
    for i in range(len(x)):
        acc = (1 - a) * x[i] + a * acc
        y[i] = acc
    return y


def lp_fft(x, fc, slope=4):
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / SR)
    X /= (1 + (f / fc) ** (2 * slope)) ** 0.5
    return np.fft.irfft(X, len(x))


def hp_fft(x, fc):
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / SR)
    X *= (f / fc) ** 2 / (1 + (f / fc) ** 2)
    return np.fft.irfft(X, len(x))


def env_ad(n, a, d):
    e = np.ones(n)
    na, nd = int(a * SR), int(d * SR)
    if na:
        e[:na] = np.linspace(0, 1, na)
    if nd:
        e[-nd:] *= np.linspace(1, 0, nd)
    return e


def reverb(x, secs=2.5, mix=0.35, seed=1):
    rng = np.random.default_rng(seed)
    n = int(secs * SR)
    ir = rng.standard_normal(n) * np.exp(-np.linspace(0, 7, n))
    ir = lp_fft(ir, 4000)
    wet = np.fft.irfft(np.fft.rfft(x, len(x) + n) * np.fft.rfft(ir, len(x) + n))[: len(x)]
    wet /= np.max(np.abs(wet)) + 1e-9
    return (1 - mix) * x + mix * wet * np.max(np.abs(x))


def music():
    N = t2i(DUR)
    t = np.arange(N) / SR
    out = np.zeros(N)
    chords = [(57, 60, 64), (53, 57, 60), (48, 52, 55), (55, 59, 62)]   # Am F C G
    bar = 7.2
    for k in range(int(DUR / bar) + 1):
        c = chords[k % 4]
        a, b = t2i(k * bar), min(N, t2i((k + 1) * bar + 1.5))
        tt = t[a:b] - t[a]
        e = env_ad(b - a, 2.5, 2.5)
        for m in c:
            f = 440 * 2 ** ((m - 69) / 12)
            for det in (-0.12, 0.0, 0.12):
                out[a:b] += e * 0.05 * np.sin(2 * np.pi * f * (1 + det / 100) * tt + det)
            out[a:b] += e * 0.02 * np.sin(2 * np.pi * f / 2 * tt)
    pulse = np.zeros(N)                                    # soft pulse for the mechanism / technology feel
    for k in range(int(DUR / 0.6)):
        i = t2i(k * 0.6)
        n = min(N - i, t2i(0.25))
        pulse[i:i + n] += 0.035 * np.sin(2 * np.pi * 110 * np.arange(n) / SR) * np.exp(-np.arange(n) / SR * 18)
    gate = np.zeros(N)
    gate[t2i(25):t2i(50)] = 1; gate[t2i(85):t2i(100)] = 1
    gate = lp_fft(gate, 2)
    out = out + pulse * gate
    out = reverb(lp_fft(out, 3500), 3.0, 0.45)
    out *= env_ad(N, 3.0, 4.0)
    return out / (np.max(np.abs(out)) + 1e-9) * 0.22


def rain():
    N = t2i(DUR)
    rng = np.random.default_rng(7)
    x = hp_fft(lp_fft(rng.standard_normal(N), 6000), 250)
    drops = np.zeros(N)
    for i in rng.integers(0, N, 9000):
        n = min(N - i, 400)
        drops[i:i + n] += rng.uniform(0.2, 1) * np.exp(-np.arange(n) / 40) * np.sin(np.arange(n) * rng.uniform(0.5, 1.5))
    x = x / np.max(np.abs(x)) * 0.09 + drops / np.max(np.abs(drops)) * 0.05
    flow = lp_fft(rng.standard_normal(N), 500)
    x += flow / np.max(np.abs(flow)) * 0.06
    g = np.zeros(N)
    for a, b in STREET:
        g[t2i(a):t2i(b)] = 1
    return x * lp_fft(g, 3)


def mech():
    N = t2i(DUR)
    rng = np.random.default_rng(11)
    x = np.zeros(N)

    def motor(a, b, f0, amp, wobble=0.05):
        i, j = t2i(a), t2i(b)
        tt = np.arange(j - i) / SR
        ramp = np.minimum(1, np.minimum(tt / 0.35, (tt[-1] - tt) / 0.35 + 1e-9))
        f = f0 * (0.75 + 0.25 * ramp) * (1 + wobble * np.sin(2 * np.pi * 3 * tt))
        ph = 2 * np.pi * np.cumsum(f) / SR
        s = sum(np.sin(k * ph) / k for k in range(1, 9))
        s = lp_fft(s, 1800) * ramp * amp
        x[i:j] += s

    def clicks(a, b, rate, amp):
        for tc in np.arange(a, b, 1 / rate):
            i = t2i(tc + rng.uniform(-0.004, 0.004))
            n = min(N - i, 900)
            x[i:i + n] += amp * rng.uniform(0.6, 1) * rng.standard_normal(n) * np.exp(-np.arange(n) / 120)

    def clunk(tc, amp):
        i = t2i(tc)
        n = min(N - i, t2i(0.5))
        tt = np.arange(n) / SR
        x[i:i + n] += amp * (np.sin(2 * np.pi * 70 * tt) * np.exp(-tt * 14) + 0.4 * np.sin(2 * np.pi * 1850 * tt) * np.exp(-tt * 30)
                             + 0.5 * rng.standard_normal(n) * np.exp(-tt * 60))

    for a, b, kind in MECH:
        if kind == "gear":
            motor(a, b, 95, 0.16); clicks(a + 0.2, b - 0.2, 22, 0.05); clunk(b, 0.35)
        elif kind == "slide":
            motor(a, b, 140, 0.1); clicks(a, b, 6, 0.06); clunk(b, 0.25)
        elif kind == "pivot":
            motor(a, b, 120, 0.11); clunk(b, 0.35)
        elif kind == "tele":
            motor(a, b, 180, 0.08, 0.02); clunk((a + b) / 2, 0.15); clunk(b, 0.3)
        elif kind == "rail":
            clicks(a, b, 3, 0.08)
            for k in range(6):
                clunk(b - 0.25 + 0.04 * k, 0.09)
    for a, b, lvl in DRIVE:
        motor(a, b, 60, 0.13 * lvl, 0.08); clicks(a, b, 14, 0.03 * lvl)
    return reverb(x, 0.6, 0.15, 3)


def main():
    os.makedirs(OUT, exist_ok=True)
    N = t2i(DUR)
    voice = np.zeros(N)
    caps = []
    prev_end = 0.0
    for k, (t0, cap, spoken) in enumerate(LINES):
        v = tts(spoken or cap, os.path.join(OUT, f"line_{k}.wav"))
        t0 = max(t0 - 0.5 if k == 0 else t0, prev_end + 0.35)
        prev_end = t0 + len(v) / SR
        i = t2i(t0)
        n = min(len(v), N - i)
        voice[i:i + n] += v[:n] * 0.95
        caps.append(dict(start=t0, end=t0 + len(v) / SR + 0.4, text=cap))
        print(f"line {k}: {t0:6.1f}s  {len(v) / SR:5.1f}s")
    # duck music + rain under the voice
    vol = lp_fft(np.abs(voice), 3)
    duck = 1 - 0.55 * np.clip(vol / (np.max(vol) + 1e-9) * 6, 0, 1)
    mix = voice + music() * duck + rain() * (0.6 + 0.4 * duck) + mech() * 0.9
    mix *= 0.89 / np.max(np.abs(mix))
    pcm = (np.clip(mix, -1, 1) * 32767).astype(np.int16)
    stereo = np.c_[pcm, pcm].reshape(-1)
    with wave.open(os.path.join(OUT, "film_audio.wav"), "w") as w:
        w.setnchannels(2); w.setsampwidth(2); w.setframerate(SR); w.writeframes(stereo.tobytes())
    with open(os.path.join(OUT, "captions.json"), "w", encoding="utf-8") as f:
        json.dump(caps, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
