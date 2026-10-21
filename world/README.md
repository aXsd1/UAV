# Dünya üretimi

`tools/worldgen.py` (WSL içinde çalışır) Copernicus DEM GLO-30 yükseklik verisi
ve OpenStreetMap bina/yol verisinden bir Gazebo dünyası üretir:

```bash
python3 tools/worldgen.py fetch --bbox 41.145 32.580 41.235 32.700
python3 tools/worldgen.py build --bbox 41.145 32.580 41.235 32.700 --origin <enlem> <boylam> --name karabuk
WORLD=karabuk bash px4/run_sitl.sh
```

- `fetch`: DEM penceresi AWS'deki bulut uyumlu GeoTIFF'ten yalnız gereken baytlar
  okunarak, OSM verisi Overpass API'den indirilir (`world/data/`, repoya girmez).
- `build`: arazi üçgen ağı (321 × 321, ~31 m; Gazebo heightmap'inde DART çarpışma
  yüzeyi görselden ~9 m yukarıda kaldığı için ağ kullanılıyor), tüm binalar için tek OBJ ağ (yükseklik OSM
  etiketinden, yoksa 9 m), yollar için görsel şeritler ve dünya SDF'i
  (`world/build/<ad>/`, repoya girmez). `spherical_coordinates` Gazebo
  orijinini gerçek konuma koyar; PX4'ün GNSS'i gerçek koordinatları görür.

Varsayılan bölge: Karabük çevresi, 10 × 10 km (şehir merkezi, vadiler, açık arazi).
Kalkış noktası 41,1639 / 32,6443 (275 m, eğim < 1°, en yakın bina 645 m).
Durum (2026-10-21): DEM ve 3313 bina indirildi; yollar Overpass yedek sunucusu
zaman aşımına uğradığı için henüz yok.
Test sahası seçilince kendi fotogrametri uçuşumuzla (OpenDroneMap, 2–5 cm) dijital
ikiz eklenecek.

Lisans: harita verisi © OpenStreetMap katkıcıları (ODbL); yükseklik verisi
© DLR e.V. 2010–2014 ve © Airbus Defence and Space GmbH 2014–2018, Copernicus
programı tarafından sağlanmıştır.
