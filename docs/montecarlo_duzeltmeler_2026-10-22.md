# Geçiş düzeltmeleri ve Monte Carlo ölçümleri (2026-10-22/23)

`montecarlo_2026-10-22.md`'deki 597 koşunun ardından yapılan düzeltmeler, 6–8 m/s rüzgârda ilk
ölçüm (46 koşu, 22 Ekim) ve 23 Ekim'deki büyük set: yeni ayarlarla 1.000 geçiş, eski ayarlarla
100 geçiş ve diğer 5 senaryodan 50'şer koşu (toplam 1.350 koşu).

**Okurken dikkat:** 23 Ekim 08:54'ten sonra simülasyon bozuldu (IMU verisinde boşluklar, bölüm 7).
1.000 geçişin 467'si, eski ayar setinin tamamı ve diğer 5 senaryonun tamamı bu dönemde koştu.
Bu dönemdeki araç kayıplarının çoğu büyük olasılıkla simülasyon kaynaklı; temiz dönem rakamları
ayrıca verildi.

## 1. Başlangıç: 597 koşu (eski ayarlar)

| | |
|---|---|
| Geçti | 472 / 597 (%79) |
| Ön geçiş irtifa kaybı ≥ 5 m | 114 |
| Araç kaybı | 6 (423, 412, 455, 69, 414, 557) |
| Ön geçiş irtifa kaybı | medyan 3,9 m, %95 6,3 m |
| **6–8 m/s alt kümesi** | **76 / 139 kaldı (%45 geçti), medyan kayıp 5,2 m** |

Kayıplar rüzgârla artıyordu; geçişlerin çoğu yan rüzgârda yapılıyordu (weathervane
kanat açıklığını rüzgâra çeviriyor).

## 2. Bulunan simülatör hatası: hava hızı sensörü

Gazebo'nun air_speed sensörü, uçuş dinamiği eklentisinin kendi içinde uyguladığı rüzgârı
görmüyordu; PX4'e **yer hızını** hava hızı diye veriyordu. Geçişin bitiş anı ve FW'ye devirdeki
tutunma payı bu yüzden yanlıştı. Düzeltme: eklenti hava hızını kendisi yayımlıyor (burun ekseni
boyunca ½ρu² diferansiyel basınç + 0,5 Pa gürültü). 597 koşunun tamamı bu hatayla yapıldı.

## 3. Kayıp nedenleri (597 koşunun kayıtları)

| Koşu | Neden | Düzeltme |
|---|---|---|
| 455, 69, 557 (414 benzer) | İnişte irtifa tahmini 0,6–1 m yüksek kaldı; PX4 yere değdiğini anlamadı, vane'ler doyuma gitti → devrilme | Kuyruğa aşağı bakan lidar, EKF2_RNG_CTRL 1 |
| 423 | FW devrinde yunuslama hızı kaçışı (−2,3 → +4,6 rad/s) → quad-chute → çakılma | Daha yavaş geçiş (4 s), daha yüksek devir hızı |
| 412 | Failsafe alçalmasına geçti, 8 m/s yatay hızla yere değdi | **Açık** |

## 4. Uygulanan düzeltmeler

1. **Rüzgâra karşı geçiş:** havada asılıyken eğilme yönünden rüzgâr yönü tahmin ediliyor
   (5 ölçüm ortalaması), burun MAV_CMD_DO_REPOSITION ile rüzgâra çevriliyor; dönüş ve geçiş
   süresince WV_EN 0.
2. **Geçiş bitiş hızı:** VT_ARSP_TRANS 19,1 → 20,9 m/s (1,1 → 1,2 × tutunma hızı).
3. **Geçiş süresi:** VT_F_TRANS_DUR 3 → 4 s.
4. **İniş lidarı:** kuyruk ucunda gpu_lidar, EKF2_RNG_CTRL 1, EKF2_MIN_RNG 0,15, EKF2_RNG_A_HMAX 10.

Ayrıca hava hızı sensörü düzeltmesi (bölüm 2).

## 5. İlk ölçüm: 6–8 m/s, yeni ayarlar, 46 koşu (22 Ekim, temiz)

| | Önce (597'nin 6–8 m/s kısmı) | Sonra |
|---|---|---|
| Koşu | 139 | 46 (48 başladı, 2 düzenek hatası) |
| Geçti | 63 (%45) | **44 (%96)** |
| Ön geçiş irtifa kaybı ≥ 5 m | 76 | 2 (5,2 m ve 5,6 m) |
| Ön geçiş irtifa kaybı, medyan / %95 / en çok | 5,2 m / — / — | **3,7 / 4,9 / 5,6 m** |
| Ön geçiş süresi, medyan / %95 / en çok | — | 4,3 / 4,6 / 4,9 s |
| Araç kaybı | 3 | **0** |

## 6. 23 Ekim seti: düzenek ve kesintiler

- 2 paralel, 4× hız, kamerasız model, düz "default" dünya; şehir inişi Karabük'te (Esri dokulu
  güncel sürüm). Her koşu bitince sonucu diske yazıldı; bekçi (watchdog.ps1) uygulamadan bağımsız
  çalıştı, durunca temizleyip kaldığı yerden sürdürdü.
- **Kesintiler:** bekçi 10 kez otomatik yeniden başlattı (06:01 WSL yeniden başlaması; 13:22–17:19
  arası 9 kez, çoğu C: diskinin dolmasıyla "No space left on device"); 2 kez elle başlatıldı
  (08:54 /mnt/c G/Ç hatası sonrası betik düzeltilince; 17:36 disk boşaltılınca). Bir kez (07:03)
  betik hatası yüzünden set yanlışlıkla "bitti" işaretlenip durdu; düzeltildi (dd952ef).
- Düzenek hatası (sim açılmadı vb.) olan koşular aynı tohumla yeniden koşuldu; aşağıdaki
  sayılarda her koşu bir kez sayılıyor.

## 7. Simülasyon bozulması (08:54 sonrası)

08:54'ten sonra koşulan uçuş kayıtlarında IMU örnekleri arasında boşluklar var:

| | 08:54 öncesi | 08:54 sonrası |
|---|---|---|
| IMU örnek aralığı %99 | 4 ms | 8 ms |
| En büyük boşluk | 20–24 ms | 32–120 ms |
| Uçuş başına > 20 ms boşluk | 0–1 | 4–11 |

Bu dönemdeki kayıpların çoğu, sakin hava dahil, kararlı askıdan ya da iniş alçalmasından 1–3 s
içinde yatışın 5°'den 60°'nin üstüne fırlaması; aerodinamik bir yükleniş yok. Bir askı koşusunda
EKF konum tahmini ~1 km kaydı (araç gerçekte yerinde). Kesin neden doğrulanmadı; büyük olasılıkla
aynı saatlerde açık olan başka Gazebo oturumlarıyla (Karabük dünyası, arayüz) işlemci paylaşımı ve
disk dolması. Bu dönemin araç kaybı sayıları gerçek değerden yüksek olmalı.

## 8. 1.000 geçiş, yeni ayarlar, 0–8 m/s rüzgâr (tohum 4000)

| | Tümü | Temiz dönem | Bozuk dönem |
|---|---|---|---|
| Koşu | 1.000 | 533 | 467 |
| Geçti | **960 (%96,0)** | 521 (%97,7) | 439 (%94,0) |
| Ön geçiş irtifa kaybı ≥ 5 m | 17 | 6 | 11 |
| Ön geçiş irtifa kaybı, medyan / %95 / en çok | 3,2 / 4,6 / 31,6 m | 3,2 / 4,6 / 6,3 m | 3,3 / 4,6 / 31,6 m |
| Ön geçiş süresi, medyan / %95 / en çok | 4,4 / 4,7 / 6,3 s | 4,4 / 4,7 / 5,2 s | 4,4 / 4,7 / 6,3 s |
| Ön geçiş ≥ 5 s | 7 | | |
| Araç kaybı (devrilme / çakılma) | 14 | **2** | 12 |
| İniş algılanmadı (araç dik) | 4 | 2 | 2 |

Kalma nedenleri (40): ön geçiş tamamlanmadı/yavaş 11, irtifa kaybı ≥ 5 m 14, geri geçiş/iniş 15.

### Rüzgâr hızına göre (temiz dönem)

| Rüzgâr | Koşu | Kalan | Medyan kayıp | Araç kaybı |
|---|---|---|---|---|
| 0–2 m/s | 149 | 2 | 2,9 m | 0 (2 iniş algılanmadı) |
| 2–4 m/s | 131 | 0 | 3,1 m | 0 |
| 4–6 m/s | 132 | 0 | 3,5 m | 0 |
| 6–8 m/s | 121 | 10 | 3,9 m | 2 |

Bozuk dönemde aynı aralıklar: 4/113, 1/126, 9/123, 14/105 kaldı.

### Araç kayıpları

- **Temiz dönem (2):**
  - #155 (7,1 m/s): FW seyirde (30 m) 18 s boyunca yere doğru alçaldı, hava hızı 22'den 41 m/s'ye
    çıktı; burun aşağı bir dalış, irtifa kontrolü kaybı (597'deki #423'e benzer). Neden açık.
  - #231 (7,7 m/s): inişte yere değerken devrildi.
- **Bozuk dönem (12):**
  - #846, 876, 975: askıda (30 m) 1–3 s içinde ani yatış, quad-chute.
  - #854, 887, 910, 938, 940, 948, 972: iniş alçalmasında ani yatış.
  - #957, 969: FW seyirde 2–4 s içinde yere dalış.
  - #950: ön geçişte 31,6 m irtifa kaybı, quad-chute, yere çarpma.

  Bunlarda IMU boşlukları, "Accel #0 fail: TIMEOUT" ve "ekf2 missing data" uyarıları var.
- Geçiş ve seyir 30 m'de yapılıyor; FW'deki her irtifa kaybı doğrudan yere çarpma demek.
- Kalkışta devrilme: #292 ve #375 (7,6–8,0 m/s) ilk denemelerde kalkışta yatıp failsafe ile
  kapandı, yeniden koşulunca geçti; tekrarlanabilir değil.

### Rüzgâra karşı dönüş (1.000 koşu)

| Dönüş | Koşu | Medyan kayıp | ≥ 5 m (6–8 m/s içinde) |
|---|---|---|---|
| ±10°'ye döndü | 276 | 3,3 m | 2 / 99 |
| ±10°'ye ulaşamadı | 211 | 3,7 m | 10 / 84 |
| Yön tahmin edilmedi (eğilme küçük, çoğu düşük rüzgâr) | 509 | 3,0 m | 1 / 40 |

1.000 koşuda tam dönüş yarar gösteriyor (46 koşuluk ilk ölçümdeki ters sonuç küçük örneklemdendi).
Yön tahmini hatası medyan 26°, %95'te ~177° (ara sıra tersi yön); iyileştirilmeli.

## 9. Eski ayar + düzeltilmiş simülatör: 100 koşu, 6–8 m/s (tohum 3000, MC_LEGACY=1)

Eski ayar (19,1 m/s, 3 s, dönüş yok, lidar kapalı), düzeltilmiş hava hızı sensörü. Tamamı bozuk
dönemde koştu; adil karşılaştırma aynı dönemdeki yeni ayarlı 6–8 m/s koşuları.

| | Eski 597 (6–8, sensör hatalı) | **Eski ayar, düzeltilmiş sim** | Yeni ayar, 22 Ekim (46, temiz) | Yeni ayar, 1.000'in 6–8'i (bozuk dönem, 105) |
|---|---|---|---|---|
| Geçti | %45 | **%64** | %96 | %87 |
| Ön geçiş kaybı ≥ 5 m | %55 | **%28** | %4 | %7 |
| Medyan kayıp | 5,2 m | **4,8 m** | 3,7 m | 3,9 m |
| %95 kayıp | — | 5,9 m | 4,9 m | 5,2 m |
| Ön geçiş süresi medyan | — | 3,6 s | 4,3 s | 4,3 s |
| Araç kaybı | 3 / 139 | **11 / 100** | 0 / 46 | 7 / 105 |

Aynı 46 tohumla eşleştirince eski ayar 39/46 koşuda daha fazla irtifa kaybetti (medyan +1,0 m),
geçen koşu 33'e karşı 44. Yani: hava hızı sensörü düzeltmesi tek başına 6–8 m/s'de geçme oranını
%45'ten ~%64'e çıkarıyor; ayar değişiklikleri bunu %87–96'ya taşıyor. Eski ayarın 11 kaybının
dağılımı:
- 5'i inişte (#21, 22, 36, 38, 51; lidar yok).
- 2'si askıda (#58, 87).
- 3'ü FW seyirde ya da geri geçişte (#24, 59, 82).
- 1'i ön geçişte 30 m irtifa kaybıyla (#83).

Bozuk dönem bunları da şişirmiş olabilir.

## 10. Diğer senaryolar (50'şer koşu, tohum 5000, hepsi bozuk dönemde)

| Senaryo | Sonuç |
|---|---|
| **Askıda rüzgâr** (0–12 m/s, 10 m) | 39/50 geçti. 8 m/s üstündeki 6 koşunun tamamı (9,3–11,4 m/s) yerinde durdu ama **inişte devrildi**. 4 koşuda "konum hatası" 140–690 m: EKF konum tahmini kaydı, araç dik indi (bozulma). Medyan konum tutma hatası 0,27 m. |
| **Eyleyici arızası** (motor 0/1, servo 0/1 takılı) | **50/50 kontrolsüz düşüş.** 2 motorlu tasarımda tek motor kaybı ve tek vane servosunun takılması askıda kurtarılamıyor. Beklenen bir tasarım sınırı; yedeklilik ya da paraşüt gerekiyor. |
| **Bağlantı kaybı** (FW'de, RTL) | **50/50 geçti.** Hepsi RTL'ye geçti, eve döndü, medyan iniş hatası 5 cm. |
| **GNSS kaybı** (60 s, askıda) | Kriter yok. 60 s'de gerçek sürüklenme medyan 58 m, %95 150 m, en çok 231 m; 49'unda failsafe devreye girdi. GNSS'siz konum tutma yok. |
| **Şehir inişi** (Karabük, 2–8 m/s, türbülans 0,25 × rüzgâr) | 17/50 geçti (iniş hatası medyan 0,12 m). **32 koşuda araç kalkışta devrildi** (3,5–7,7 m/s; attitude failure → failsafe → disarm). Geçenler 2,2–5,5 m/s. 1 koşu indi ama 170 m uzağa. |

## 11. "Compass needs calibration - Land now!" ve failsafe'ler

- **Pusula mesajı:** 1.000 geçişte 6 koşu (%0,6; #329 ve #579 dahil), eski ayar setinde 4,
  askıda rüzgârda 1.
- **Nedeni:** mesaj, yatışın ya da yunuslamanın ~90°'ye yaklaştığı anlarda çıkıyor: ya geçiş
  sırasında (#579 ön geçişte; #329 geri geçişten hemen sonra; ikisinde de 5–6,5 m/s ani alçalma ve
  yön tahmininde 50–130°'lik sıçrama), ya da araç devrilirken/devrildikten sonra (#854, eski ayar
  #21, #22). Kuyruk üstü araçta burun dikken manyetik yön tanımsızlaşıyor; EKF pusula ölçümünü
  tutarsız bulup arıza bayrağı kaldırıyor. Devrilme vakalarında mesaj sonuç, sebep değil. Test uçuşundaki
  tepeye çarpma da aynı türden. Önerilen deneme (doğrulanmadı): EKF2_MAG_TYPE'ı yalnız yön (1)
  yapmak ya da dik duruşta pusula füzyonunu sınırlamak.
- **Failsafe:** 1.000 geçişte 6 koşu (5'inde quad-chute; hepsi bozuk dönemde), eski ayar setinde 8.
  Bağlantı kaybı (50) ve GNSS kaybındaki (49) failsafe'ler beklenen davranış.

## 12. Açık kalanlar

1. **Simülasyon bozulmasının nedeni** ve bozuk dönem koşularının (467 geçiş, eski ayar seti, diğer
   senaryolar) temiz koşullarda tekrarı.
2. **Kalkış/inişte yerde devrilme:** şehirde 3,5 m/s, düzde 7+ m/s, askıda 9+ m/s rüzgârda. İniş
   takımı açıklığı (0,45 m), yere değince motor kesme mantığı ve rüzgârı kanat yüzüne değil
   kenarına alan kalkış yönü incelenmeli.
3. **Eyleyici arızası:** tek motor/servo kaybı kurtarılamıyor; tasarım kararı gerekli.
4. **GNSS'siz seyir:** 60 s'de 58 m (medyan) sürüklenme; optik akış ya da görsel konumlama yok.
5. Rüzgâr yönü tahmini (medyan 26° hata, ara sıra ters yön).
6. Pusula (bölüm 11) ve #412.
7. **FW seyirde dalış (#155, temiz dönem):** irtifa kontrolü kaybı; 30 m seyir irtifası pay
   bırakmıyor.
8. "1.000 geçişte sıfır kayıp" hedefi: temiz dönemde 533 geçişte 2 kayıp (7+ m/s'de: biri FW
   dalışı, biri inişte devrilme); hedef henüz karşılanmıyor.

Ham sonuçlar: `runs/montecarlo/*/results.jsonl` (repoda değil); bu rapordaki rakamlar
`python -m tools.montecarlo.report_sets` çıktısından (`runs/montecarlo/sets_2026-10-23.json`).
