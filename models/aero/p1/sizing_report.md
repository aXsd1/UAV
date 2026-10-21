# P1 boyutlandırma raporu

`tools/aerogen` tarafından `p1.toml` dosyasından üretildi; elle düzenlemeyin.
Değerler el hesabıdır (pervane kapalı gövde aerodinamiği, kanallı momentum teorisi).

| Kontrol | Değer | Hedef | Durum |
|---|---|---|---|
| Kütle toplamı | 12.00 kg | 12.00 kg | GEÇTİ |
| Ağırlık merkezi istasyonu | 0.469 m | bilgi | bilgi |
| Statik marj (seyir) | 10.3 % | 8–12 % | GEÇTİ |
| Seyirde gereken CL | 0.801 | ≈ 0.80 (doküman) | GEÇTİ |
| Seyir hücum açısı | 8.3° | < stall 15° | GEÇTİ |
| Seyir L/D | 8.8 | ≈ 9 (doküman) | GEÇTİ |
| Seyir elektrik gücü | 635 W | ≈ 480 W (doküman) | bilgi |
| Askı devri | 8452 rpm | < 10000 rpm | GEÇTİ |
| Askı mil gücü | 3129 W | ≈ 2200 W (doküman) | KALDI |
| Askı elektrik gücü | 3681 W | bilgi | bilgi |
| T/W=1.4 için mil gücü | 5183 W | ≤ motor tepe 4400 W | KALDI |
| Kanal çıkış hızı (askı) | 37.6 m/s | bilgi | bilgi |
| Görev enerjisi | 757 Wh (askı 123 + seyir 635) | ≤ kullanılabilir 604 Wh | KALDI |
| Ulaşılabilir seyir süresi | 45 dk | ≥ 60 dk | KALDI |
| Batarya akımı (T/W tepe, nominal gerilim) | 137 A @ 44.4 V | ≤ 640 A (40C) | GEÇTİ |
| Vane yunuslama momenti vs 8 m/s yan rüzgâr | 6.1 N·m (kol 0.19 m) | > 1,5 × 6.2 N·m rüzgâr momenti | KALDI |
| Vane sapma momenti vs 8 m/s yan rüzgâr | 6.1 N·m | > 1,5 × 3.1 N·m rüzgâr momenti | GEÇTİ |
| İtki ekseni etrafında moment (±%10 devir farkı) | 0.71 N·m | bilgi | bilgi |

Notlar:
- Rüzgâr momenti kontrolü kanal momentum sürüklemesini içermez; gerçek yük daha büyüktür.
- Askı gücü = ideal kanallı güç / FM × (1 + koaksiyel kayıp); FM ve kayıp config'ten.
