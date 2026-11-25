"""Material library: name -> (RGB 0..1, metallic, roughness, opacity, description)."""

MATERIALS = {
    "polymer_black":  ((0.045, 0.045, 0.05), 0.0, 0.55, 1.0, "Mat siyah teknik polimer (PA6-GF30 / rotomoulded PE)"),
    "polymer_dark":   ((0.12, 0.12, 0.13), 0.0, 0.6, 1.0, "Antrasit teknik polimer"),
    "orange":         ((0.95, 0.36, 0.04), 0.25, 0.42, 1.0, "Turuncu toz boyalı çelik / alüminyum (RAL 2004)"),
    "rubber":         ((0.025, 0.025, 0.028), 0.0, 0.92, 1.0, "Siyah aşınmaya dayanıklı kauçuk"),
    "antislip":       ((0.07, 0.07, 0.075), 0.55, 0.62, 1.0, "Siyah kaymaz kabartmalı alüminyum (anodize)"),
    "frame_black":    ((0.09, 0.09, 0.1), 0.6, 0.45, 1.0, "Siyah anodize alüminyum profil"),
    "brushed":        ((0.80, 0.81, 0.82), 1.0, 0.28, 1.0, "Fırçalanmış paslanmaz boru (korkuluk)"),
    "stainless":      ((0.66, 0.67, 0.69), 1.0, 0.22, 1.0, "Paslanmaz çelik A4 bağlantı elemanı"),
    "aluminum":       ((0.74, 0.75, 0.77), 1.0, 0.38, 1.0, "Alüminyum alaşım 6061-T6 (iç şasi)"),
    "steel_dark":     ((0.22, 0.22, 0.24), 0.9, 0.4, 1.0, "Sertleştirilmiş çelik (zincir dişlisi, mil)"),
    "silver_cap":     ((0.72, 0.73, 0.75), 1.0, 0.3, 1.0, "Alüminyum jant kapağı"),
    "epdm":           ((0.16, 0.16, 0.17), 0.0, 0.95, 1.0, "EPDM elastomer conta"),
    "pcb":            ((0.04, 0.32, 0.16), 0.1, 0.5, 1.0, "Elektronik kart"),
    "battery":        ((0.20, 0.32, 0.55), 0.3, 0.5, 1.0, "LiFePO4 batarya modülü"),
    "glass":          ((0.55, 0.65, 0.75), 0.0, 0.05, 0.35, "Polikarbonat şeffaf kapak / lens"),
    "reflector":      ((1.0, 0.55, 0.0), 0.0, 0.2, 1.0, "Turuncu reflektör"),
    "red":            ((0.80, 0.05, 0.04), 0.0, 0.4, 1.0, "Acil durdurma / kırmızı kablo"),
    "cable":          ((0.05, 0.05, 0.05), 0.0, 0.6, 1.0, "Kablo kılıfı"),
    "led":            ((0.95, 0.95, 0.90), 0.0, 0.1, 1.0, "LED far"),
}
