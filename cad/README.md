# AKORA — 1:1 Parametrik Endüstriyel CAD Modeli

Afet ve sel tahliye robotu AKORA'nın gerçek ölçekli, düzenlenebilir ve üretim mantığına göre kurulmuş 3D CAD modeli.

- **Ölçek:** 1/1, **birim:** santimetre (STEP ve Rhino dosyaları cm olarak yazılır).
- **Geometri:** OpenCASCADE çekirdeğinde NURBS B-rep kapalı katılar. Hiçbir parça tek parça mesh değildir.
- **Referanslar:** `reference/` klasöründe. Kaynaklar teknik çizim (`teknik_cizim.jpg`, PDF poster ölçeği 1/40), video kareleri ve çizgi render.

> Model kod olarak (CadQuery / Python) tanımlıdır. Her ölçü `akora/params.py` içinde tek yerde durur. Bir ölçü değiştirilip `python build.py` çalıştırıldığında bütün konfigürasyonlar, STEP/3DM dosyaları, çakışma raporu ve renderlar yeniden üretilir.

## Çıktılar

| Dosya | İçerik |
|---|---|
| `output/rhino/AKORA_master.3dm` | Rhino 8 dosyası, birim cm. 01–11 katman ağacı. Her parça ayrı, isimli ve malzemeli obje. 10_ASSEMBLY altında 18 konfigürasyon, 11_EXPLODED_VIEW altında patlatılmış görünüş var. |
| `output/step/AKORA_<konfig>.step.zip` | Her konfigürasyon için **NURBS katı** STEP AP214, birim cm. Montaj ağacı katman ağacıyla aynıdır. Rhino'da *File → Open/Import* ile açılır ve düzenlenebilir polysurface olarak gelir. |
| `output/glb/` | Render ve web görüntüleme için GLB. |
| `output/renders/` | 14 teknik render ile açılma ve kapanma panoları. |
| `output/reports/` | Parça listesi (BOM), çakışma raporu, katı geçerlilik tablosu ve ölçü özeti. |

### Rhino ile çalışma notu
`rhino3dm` kütüphanesi genel NURBS katı (trimmed B-rep) yazamaz. Bu yüzden `.3dm` içindeki objeler aynı katı modelden üretilmiş yüksek çözünürlüklü **mesh'lerdir**. Her obje isim, malzeme ve katman bilgisini taşır. Düzenlenebilir **NURBS polysurface** için STEP dosyasını Rhino'da açın. Gerekirse *Save As .3dm* ile tam NURBS 3DM elde edilir. İki dosya aynı geometriden üretildiği için birebir örtüşür.

## Katman yapısı
01_MAIN_BODY · 02_TRACK_SYSTEM (LEFT_TRACK / RIGHT_TRACK altında TRACK_FRAME, TRACK_LINKS, DRIVE_SPROCKETS, IDLER_WHEELS, ROAD_WHEELS, TRACK_MOTORS, TRACK_AXLES, BEARINGS) · 03_PLATFORM_SYSTEM · 04_STAIR_SYSTEM · 05_GUARDRAIL_SYSTEM · 06_SUPPORT_SYSTEM · 07_ELECTRONICS · 08_POWER_SYSTEM · 09_FASTENERS · 10_ASSEMBLY · 11_EXPLODED_VIEW. Alt katmanlar talimattaki adlarla birebir aynıdır.

(Ayrıntılı ölçü, mekanizma ve sonuç bölümleri derleme sonrası aşağıya eklenir.)
