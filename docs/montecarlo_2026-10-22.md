# Monte Carlo: geçiş, 597 koşu (2026-10-21/22)

SITL (PX4 v1.17 + Gazebo, düz dünya), 30 m'de ön geçiş, 25 s seyir, geri geçiş, iniş.
Rüzgâr 0–8 m/s, yön rastgele, türbülans σ = 0,1 × rüzgâr. 2 paralel, 4× hız (aynı tohum
2× ve 4× hızda aynı sonucu veriyor). Gece koşusu 1.000 için başladı, 597'de durdu
(süreçler Claude uygulamasının süreç ağacındaydı); Erin 597'yi yeterli buldu.
Ham sonuçlar: `runs/montecarlo/*_transition/results.jsonl` (repoda değil),
analiz betiği çıktısı `runs/montecarlo/transition_analysis.json`.

## Sonuç

| | Sayı |
|---|---|
| Koşu (düzenek hatası hariç) | 597 |
| Geçti | 472 (%79) |
| Ön geçiş irtifa kaybı ≥ 5 m (araç kaybı yok) | 114 |
| **Araç kaybı** (yere çakılma veya inişte devrilme) | **6 (%1)** |
| Ön geçiş süresi | hepsi < 5 s (en çok 4,13 s) |
| Ön geçiş irtifa kaybı | medyan 3,9 m, %95 6,3 m |

Dokümandaki "1.000 rastgele geçişte sıfır kayıp" hedefi karşılanmıyor.

### Rüzgâr hızına göre

| Rüzgâr | Kalan | Medyan irtifa kaybı |
|---|---|---|
| 0–2 m/s | 2 / 165 | 3,0 m |
| 2–4 m/s | 9 / 154 | 3,6 m |
| 4–6 m/s | 38 / 139 | 4,3 m |
| 6–8 m/s | 76 / 139 | 5,2 m |

### Rüzgârın uçuş yönüne göre (rüzgâr ≥ 4 m/s)

| | Kalan | Medyan kayıp |
|---|---|---|
| Karşıdan (±45°) | 1 / 4 | 4,1 m |
| Yandan (45–135°) | 102 / 260 | 4,6 m |
| Arkadan (> 135°) | 11 / 14 | 5,8 m |

Geçişlerin neredeyse hepsi yan rüzgârda: weathervane (WV_AXIS 1) kanat açıklığını
rüzgâra çevirdiği için burun rüzgâra dik bakıyor. Karşıdan ve arkadan rüzgâr örneği az.

### Araç kayıpları

| Koşu | Rüzgâr | Ne oldu |
|---|---|---|
| 423 | 4,0 m/s yandan | Geçişten sonra yatay uçuşta yere çakıldı (20 m/s, 35 m irtifa kaybı) |
| 412 | 1,8 m/s | Failsafe alçalmasına geçti, 8 m/s yatay hızla yere değip devrildi |
| 455 | 0,3 m/s | Yumuşak temas (0,3 m/s yatay, 2,8° yatış), ardından devrildi |
| 69, 414, 557 | 6,4–8,0 m/s | İnişte devrildi |

455 ve 412 sakin havada: temas sonrası devrilme rüzgârdan bağımsız bir sorun olabilir
(yer temas modeli, motor duruş torku); incelenmeli.

## Öneriler

1. **Geçişi rüzgâra karşı yapmak**: geçişten önce burnu rüzgâra çevirmek. Hava hızı
   daha çabuk oluşur; arkadan rüzgâr en kötü durum.
2. **Geçiş bitiş hızını yükseltmek**: VT_ARSP_TRANS şu an 19,1 m/s (1,1 × tutunma);
   ~21 m/s, FW'ye geçişte tutunma payını artırır.
3. **Geçiş eğimini yumuşatmak**: VT_F_TRANS_DUR 3 → 4 s; burun daha yavaş iner.
4. **Temas sonrası devrilmeyi incelemek**: 455'in kaydıyla, temas anından sonraki
   kuvvetler ve motor/vane durumu.
5. **423'ü incelemek**: tek FW çakılması; geçiş sonrası hız/hücum açısı kaybı.

Her değişiklik, 6–8 m/s rüzgârda ~100 koşuluk bir alt kümeyle ölçülmeli.
