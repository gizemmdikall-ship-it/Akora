# AKORA çakışma (interference) raporu

Yöntem: her konfigürasyonda yerleştirilmiş tüm katı çiftleri için sınır kutusu ön elemesi, ardından OpenCASCADE Boolean `Common` hacmi. 0,2 cm³ üzeri kesişim **çakışma** sayılır. Bağlantı elemanlarının geçtiği delikler `drill.py` ile gerçek delik olarak modellenmiştir. Tasarım gereği izinli tek temas: üst kapak dudağının EPDM contayı 2 mm (%25) sıkıştırması.

| Konfigürasyon | Açıklama | Çakışma | Bağlantı elemanı geçişi |
|---|---|---:|---:|
| C01_CLOSED_ROBOT | Kapalı AKORA (taşıma konumu) | 0 | 0 |
| C02_TOP_COVER_CLOSED | Üst kapağı kapalı gövde (platform paketi sökülü) | 0 | 0 |
| C03_TOP_COVER_REMOVED | Üst kapak tamamen çıkarılmış (kapak yanda, yerde) | 0 | 0 |
| C04_INTERNALS_VISIBLE | İç mekanizma görünür (kapak, batarya kapağı, sol palet sökülü) | 0 | 0 |
| C05_TRACK_DETAIL | Palet mekanizması detayı | 0 | 0 |
| C06_PLATFORM_FOLDED | Platform mekanizması katlanmış | 0 | 0 |
| C07_FIRST_PLATFORM_OPEN | İlk platform (P2) açıldı: dikey konum, P3 hâlâ katlı | 0 | 0 |
| C08_SECOND_PLATFORM_OPEN | İkinci platform (P3) açılmış | 0 | 0 |
| C09_STAIRS_OPEN | Merdivenler açılmış | 0 | 0 |
| C10_RAILINGS_OPEN | Korkuluklar açılmış | 0 | 0 |
| C11_SUPPORT_LEGS_OPEN | Destek ayakları açılmış | 0 | 0 |
| C12_FULLY_DEPLOYED | Tamamen açılmış tahliye robotu | 0 | 0 |
| C13a_OPENING_STAGE_01 | Açılma 1: P2 dikey kule, P3 yatay (PDF açılım adımı 2) | 0 | 0 |
| C13b_OPENING_STAGE_02 | Açılma 2: platformlar yataya iniyor | 0 | 0 |
| C13c_OPENING_STAGE_03 | Açılma 3: merdivenler ve ayaklar açılıyor | 0 | 0 |
| C14a_CLOSING_STAGE_01 | Kapanma 1: korkuluklar katlanıyor | 0 | 0 |
| C14b_CLOSING_STAGE_02 | Kapanma 2: ayaklar toplanıyor, merdivenler kalkıyor | 0 | 0 |
| C14c_CLOSING_STAGE_03 | Kapanma 3: platformlar katlanıyor | 0 | 0 |

## Açılma hareketi taraması (41 ara konum)

| t | j1 | j2 | arka merd. | ön merd. | ayak | korkuluk | çakışma |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.000 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.025 | 0.06 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.050 | 0.12 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.075 | 0.19 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.100 | 0.25 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.125 | 0.31 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.150 | 0.37 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.175 | 0.44 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.200 | 0.50 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.225 | 0.50 | 0.08 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.250 | 0.50 | 0.17 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.275 | 0.50 | 0.25 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.300 | 0.50 | 0.33 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.325 | 0.50 | 0.42 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.350 | 0.50 | 0.50 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.375 | 0.56 | 0.56 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.400 | 0.62 | 0.62 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.425 | 0.69 | 0.69 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.450 | 0.75 | 0.75 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.475 | 0.81 | 0.81 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.500 | 0.88 | 0.88 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.525 | 0.94 | 0.94 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.550 | 1.00 | 1.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 |
| 0.575 | 1.00 | 1.00 | 0.17 | 0.17 | 0.00 | 0.00 | 0 |
| 0.600 | 1.00 | 1.00 | 0.33 | 0.33 | 0.00 | 0.00 | 0 |
| 0.625 | 1.00 | 1.00 | 0.50 | 0.50 | 0.00 | 0.00 | 0 |
| 0.650 | 1.00 | 1.00 | 0.67 | 0.67 | 0.00 | 0.00 | 0 |
| 0.675 | 1.00 | 1.00 | 0.83 | 0.83 | 0.00 | 0.00 | 0 |
| 0.700 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 | 0 |
| 0.725 | 1.00 | 1.00 | 1.00 | 1.00 | 0.17 | 0.00 | 0 |
| 0.750 | 1.00 | 1.00 | 1.00 | 1.00 | 0.33 | 0.00 | 0 |
| 0.775 | 1.00 | 1.00 | 1.00 | 1.00 | 0.50 | 0.00 | 0 |
| 0.800 | 1.00 | 1.00 | 1.00 | 1.00 | 0.67 | 0.00 | 2 |
| 0.825 | 1.00 | 1.00 | 1.00 | 1.00 | 0.83 | 0.00 | 2 |
| 0.850 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0 |
| 0.875 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.17 | 0 |
| 0.900 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.33 | 0 |
| 0.925 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.50 | 0 |
| 0.950 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.67 | 0 |
| 0.975 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.83 | 0 |
| 1.000 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0 |

## Ayrıntı
- **sweep t=0.8**: SUPPORT_LEG_INDEX_PIN_L × SUPPORT_LEG_INNER_L (1.61 cm³); SUPPORT_LEG_INDEX_PIN_R × SUPPORT_LEG_INNER_R (1.61 cm³)
- **sweep t=0.825**: SUPPORT_LEG_INDEX_PIN_L × SUPPORT_LEG_INNER_L (1.61 cm³); SUPPORT_LEG_INDEX_PIN_R × SUPPORT_LEG_INNER_R (1.61 cm³)
