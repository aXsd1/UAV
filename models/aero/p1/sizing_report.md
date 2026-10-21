# P1 boyutlandırma raporu

`tools/aerogen` tarafından `p1.toml` dosyasından üretildi; elle düzenlemeyin.
Değerler el hesabıdır (pervane kapalı gövde aerodinamiği, kanallı momentum teorisi).

| Kontrol | Değer | Hedef | Durum |
|---|---|---|---|
| Kütle toplamı | 12.70 kg | 12.70 kg | GEÇTİ |
| Ağırlık merkezi istasyonu | 0.460 m | bilgi | bilgi |
| Statik marj (seyir) | 10.0 % | 8–12 % | GEÇTİ |
| Tutunma hızı (CLmax) | 17.4 m/s (CLmax 0.90) | seyir 22 m/s ≥ 1.25 × tutunma | GEÇTİ |
| Seyirde gereken CL | 0.570 | ≈ 0.57 (doküman) | GEÇTİ |
| Seyir hücum açısı | 7.0° | < stall 15° | GEÇTİ |
| Seyir trim elevonu (momentum sürüklemesi dahil) | +0.0° | |δe| ≤ 5° (azami 25°) | GEÇTİ |
| Seyir L/D | 9.6 | ≈ 9 (doküman) | GEÇTİ |
| Seyir elektrik gücü | 699 W | ≈ 700 W (doküman) | bilgi |
| Askı devri | 8452 rpm | < 10000 rpm | GEÇTİ |
| Askı mil gücü | 3407 W | ≈ 3400 W (doküman) | GEÇTİ |
| Askı elektrik gücü | 4008 W | bilgi | bilgi |
| T/W=1.4 için mil gücü | 5643 W | ≤ motor tepe 6000 W | GEÇTİ |
| Kanal çıkış hızı (askı) | 38.7 m/s | bilgi | bilgi |
| Görev enerjisi | 600 Wh (askı 134 + seyir 466) | ≤ kullanılabilir 604 Wh | GEÇTİ |
| Ulaşılabilir seyir süresi | 40 dk | ≥ 40 dk | GEÇTİ |
| Batarya akımı (T/W tepe, nominal gerilim) | 150 A @ 44.4 V | ≤ 640 A (40C) | GEÇTİ |
| Vane yunuslama momenti vs 8 m/s yan rüzgâr | 6.2 N·m (kol 0.18 m) | > 1,5 × 7.8 N·m rüzgâr momenti | KALDI |
| Vane sapma momenti vs 8 m/s yan rüzgâr | 6.2 N·m | > 1,5 × 3.2 N·m rüzgâr momenti | GEÇTİ |
| Askıda 8 m/s yan rüzgâr (kanat açıklığı rüzgârda) yatışı vs devrilme açısı | 18.2° / 28.2° (ayak 0.45 m, AM yüksekliği 0.84 m) | devrilme ≥ yatış + 5° | GEÇTİ |
| Yerde 8 m/s rüzgâr, motorlar kapalı | 56 / 10 N·m = 5.82 | ≥ 1.5 | GEÇTİ |
| İtki ekseni etrafında moment (±%10 devir farkı) | 0.77 N·m | bilgi | bilgi |

Notlar:
- Rüzgâr momenti kontrolü kanal momentum sürüklemesini içermez; gerçek yük daha büyüktür.
- Askı gücü = ideal kanallı güç / FM × (1 + koaksiyel kayıp); FM ve kayıp config'ten.
