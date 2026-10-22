# Geçiş düzeltmeleri ve ölçüm (2026-10-22)

`montecarlo_2026-10-22.md`'deki 597 koşunun ardından yapılan düzeltmeler ve 6–8 m/s
rüzgârda yeni ayarlarla yapılan ölçüm. Erin 46 geçerli koşuyu yeterli buldu; set 100'e
tamamlanmadı, eski ayarlı karşılaştırma seti çalıştırılmadı.

## 1. Başlangıç: 597 koşu (eski ayarlar)

| | |
|---|---|
| Geçti | 472 / 597 (%79) |
| Ön geçiş irtifa kaybı ≥ 5 m | 114 |
| Araç kaybı | 6 (423, 412, 455, 69, 414, 557) |
| Ön geçiş irtifa kaybı | medyan 3,9 m, %95 6,3 m |
| **6–8 m/s alt kümesi** | **76 / 139 kaldı (%55 kaldı, %45 geçti), medyan kayıp 5,2 m** |

Kayıplar rüzgârla artıyordu; geçişlerin çoğu yan rüzgârda yapılıyordu (weathervane
kanat açıklığını rüzgâra çeviriyor).

## 2. Bulunan simülatör hatası: hava hızı sensörü

Gazebo'nun air_speed sensörü, uçuş dinamiği eklentisinin kendi içinde uyguladığı rüzgârı
görmüyordu; PX4'e **yer hızını** hava hızı diye veriyordu. Sonuç: rüzgâra karşı geçişte
ölçülen hız geç yükseliyor, rüzgâr arkadan/yandan gelince erken yükseliyordu; geçişin
bitiş anı ve FW'ye devirdeki gerçek tutunma payı yanlıştı. İlk rüzgâra karşı denemede
geçiş hiç tamamlanmadı, hata böyle ortaya çıktı.

Düzeltme: eklenti hava hızını kendisi yayımlıyor (burun ekseni boyunca hava hızından
½ρu² diferansiyel basınç + 0,5 Pa gürültü). Gazebo sensörü kaldırıldı.

**Önemli:** 597 koşunun tamamı bu hatayla yapıldı. Bu yüzden aşağıdaki öncesi/sonrası
farkı, ayar değişiklikleri ile simülatör düzeltmesinin toplam etkisidir; ikisi ayrılmadı.

## 3. Kayıp nedenleri (kayıt incelemesi)

| Koşu | Neden | Düzeltme |
|---|---|---|
| 455, 69, 557 (414 benzer) | İnişte irtifa tahmini 0,6–1 m yüksek kaldı; PX4 yere değdiğini anlamadı, araç bacakları üzerindeyken kontrol etmeye devam etti, vane'ler doyuma gitti → devrilme | Kuyruğa aşağı bakan lidar, EKF2_RNG_CTRL 1 |
| 423 | FW devrinde yunuslama hızı kaçışı (−2,3 → +4,6 rad/s) → quad-chute → çakılma | Daha yavaş geçiş (4 s), daha yüksek devir hızı |
| 412 | Failsafe alçalmasına geçti, 8 m/s yatay hızla yere değdi | **Açık**, nedeni bulunmadı |

## 4. Uygulanan düzeltmeler

1. **Rüzgâra karşı geçiş:** havada asılıyken aracın eğilme yönünden (itki vektörünün
   yatay bileşeni) rüzgâr yönü tahmin ediliyor (5 ölçüm ortalaması), burun MAV_CMD_DO_REPOSITION
   ile rüzgâra çevriliyor; dönüş ve geçiş süresince WV_EN 0.
2. **Geçiş bitiş hızı:** VT_ARSP_TRANS 19,1 → 20,9 m/s (1,1 → 1,2 × tutunma hızı).
3. **Geçiş süresi:** VT_F_TRANS_DUR 3 → 4 s.
4. **İniş lidarı:** kuyruk ucunda gpu_lidar (bacak uçlarının 0,15 m üstünde),
   EKF2_RNG_CTRL 1, EKF2_MIN_RNG 0,15, EKF2_RNG_A_HMAX 10. 7 m/s rüzgârda tek deneme: dik indi,
   mesafe sensörü geçerli.

Ayrıca hava hızı sensörü düzeltmesi (bölüm 2).

## 5. Ölçüm: 6–8 m/s rüzgâr, yeni ayarlar

Tohum 3000'den başlayarak 48 koşu başladı; 2'si düzenek hatası, sayılmadı (biri PX4'ün başlarken
bellek hatasıyla çıkması, biri kalkışta 30 m'ye ulaşılamaması; ikincisi ayrıca
incelenmedi), **46 geçerli koşu**. Rüzgâr 6,0–7,9 m/s, yön rastgele, türbülans σ = 0,1 × rüzgâr.

| | Önce (597'nin 6–8 m/s kısmı) | Sonra (yeni ayarlar + düzeltilmiş sim) |
|---|---|---|
| Koşu | 139 | 46 |
| Geçti | 63 (%45) | **44 (%96)** |
| Ön geçiş irtifa kaybı ≥ 5 m | 76 | 2 (5,2 m ve 5,6 m) |
| Ön geçiş irtifa kaybı, medyan | 5,2 m | **3,7 m** |
| Ön geçiş irtifa kaybı, %95 / en çok | — | 4,9 m / 5,6 m |
| Ön geçiş süresi, medyan / %95 / en çok | — | 4,3 / 4,6 / 4,9 s (hepsi < 5 s) |
| Araç kaybı | 3 (69, 414, 557; inişte devrilme) | **0** |
| İniş | — | hepsi dik; son yatış en çok 1,0°, alçalma hızı en çok 2,8 m/s |

Kalan 2 koşu yalnızca 5 m irtifa kaybı sınırını hafif aştı; geçiş, dönüş ve iniş başarılı.

### Rüzgâra karşı dönüş ne kadar işe yaradı

| Dönüş durumu | Koşu | Medyan kayıp | En çok |
|---|---|---|---|
| 20 s içinde ±10°'ye döndü | 19 | 4,1 m | 5,6 m |
| ±10°'ye ulaşamadı (dönüş kısmen) | 20 | 2,8 m | 4,1 m |
| Eğilme küçük, yön tahmin edilmedi | 7 | 3,6 m | 4,7 m |

Rüzgâr yönü tahmini medyan 11° hatalı (en çok 104°). Tam rüzgâra dönen koşular daha fazla
irtifa kaybetti; iki 5 m aşımı da bunlarda. Yorum: tam karşı rüzgârda hava hızı çabuk
oluşuyor, geçiş daha erken (≈3,5 s) bitiyor ve burun daha hızlı iniyor. İyileşmenin büyük
kısmı büyük olasılıkla hava hızı düzeltmesinden, 4 s geçişten ve 20,9 m/s devir hızından
geliyor; rüzgâra tam dönmek tek başına yarar göstermedi. Ayırmak için eski ayar seti gerekir.

## 6. Açık kalanlar

- **412:** failsafe alçalmasının nedeni bulunmadı.
- **Eski ayar + düzeltilmiş sim karşılaştırması yapılmadı**: iyileşmenin ne kadarının
  ayarlardan, ne kadarının sensör düzeltmesinden geldiği bilinmiyor. `MC_LEGACY=1` anahtarı
  hazır (aynı tohumlarla çalıştırılabilir).
- **Tek motor arızası** ve diğer senaryolar (link kaybı, GNSS kaybı, şehir inişi, havada
  asılı rüzgâr) Monte Carlo ile koşulmadı.
- Rüzgâra karşı dönüş: tahmin hatası (en çok 104°) ve tam karşı rüzgârda artan kayıp;
  geçiş açısı (ör. rüzgâra 30–45° açıyla) ayrıca denenebilir.
- Ölçüm 46 koşu; "1.000 geçişte sıfır kayıp" hedefi için daha büyük set gerekir.

Ham sonuçlar: `runs/montecarlo/20261022T184720Z_transition/results.jsonl` (repoda değil).
