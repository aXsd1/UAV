# Monte Carlo uçuş kayıtlarındaki PX4 uyarıları (2026-10-23)

21–23 Ekim'deki 61 setin 2045 uçuş kaydı (.ulg) silinmeden önce `tools/montecarlo/compact_logs.py`
ile özetlendi. Her setin `ulog_summary.jsonl` dosyasında koşu başına süre ve WARNING ve üstü
bütün PX4 mesajları duruyor. Bu tabloları ve aşağıdaki sayıları o dosyalardan üretmek için .ulg gerekmez.
Silinen: ~80 GB (`runs/` 76 GB'tan 0,6 GB'a indi). Kalan .ulg'ler: simülasyon thread'inin seçtiği 4 örnek
ve son urban_landing seti (`20261023T143858Z_urban_landing`).

Sayılar "o mesajı en az bir kez yazan koşu sayısı"dır; rakamlar mesaj içinde `#` ile birleştirildi.

| Senaryo | Koşu | Geçti / kaldı | En sık mesajlar (koşu, kalanlar arasında) |
|---|---|---|---|
| transition | 1817 | 1568 / 208 (41 bilinmiyor) | ESC failure detected 1068 (161); `command # unsupported` 289 (13); Accel TIMEOUT 85 (5); Attitude failure (roll) 76 (35); Failsafe activated 25 (17); Compass needs calibration 16 (9); Quad-chute triggered 11 (11) |
| hover_wind | 53 | 41 / 11 | ESC failure 31 (8); Attitude failure (roll) 12 (11); Accel TIMEOUT 6 (4); High Accelerometer Bias 5 (5) |
| actuator_failure | 52 | 1 / 51 | Attitude failure (roll) 49 (48); ESC failure 32 (31); Attitude failure (pitch) 23 (23) |
| link_loss | 51 | 51 / 0 | No connection to the GCS 51; Failsafe: Hold 51 (beklenen) |
| gnss_loss | 51 | sonuç satırında yok | Failsafe: blind land 23; invalid setpoints 23; heading estimate not stable 11; 24 kayıt yarım |
| urban_landing | 21 | 5 / 0 (16 bilinmiyor) | height estimate not stable 12; Failsafe activated 10; 4 kayıt yarım |

Gözlemler (çıkarım, kayıtların zaman serisine bakılmadı):

- **Pusula ("Compass needs calibration - Land now!")**: 20 koşuda (%1). 19'u geçiş, 1'i askı rüzgâr
  senaryosunda. 10'u kalan koşularda. İlk mesaj ortanca 76. saniyede, uçuşun ~%60'ında geliyor. Kalkışta
  değil uçuş sırasında çıkıyor, büyük olasılıkla sert manevra ya da çarpma sonrasında. Üç güne
  dağılmış (21: 6, 22: 3, 23: 11), yeni Karabük dünyasına özgü değil.
- **"ESC failure detected"** koşuların yarısından çoğunda ve geçenlerde de var. Kayıt sonunda motorlar
  durunca yazılan bir ön uçuş kontrolü gibi görünüyor; sonuçla ilgisi zayıf.
- **"Attitude failure (roll)"** kalan koşularda belirgin: geçişte 76 koşunun 35'i, askı rüzgârda 12'nin 11'i
  kalmış. Araç kaybının izi olarak işe yarar.
- **Quad-chute** 11 geçiş koşusunda tetiklenmiş, hepsi kalmış.

Yeniden üretmek için (WSL'de, depo kökünde):

```bash
python3 -m tools.montecarlo.compact_logs --delete              # yeni setleri özetle, geçen koşuların .ulg'sini sil
```
