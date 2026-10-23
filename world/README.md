# Dünya üretimi

`tools/worldgen.py` (WSL içinde çalışır) Copernicus DEM GLO-30 yükseklik verisi
ve OpenStreetMap bina/yol verisinden bir Gazebo dünyası üretir:

```bash
python3 tools/worldgen.py fetch --bbox 41.145 32.580 41.235 32.700 --imagery esri
python3 tools/worldgen.py build --bbox 41.145 32.580 41.235 32.700 --origin <enlem> <boylam> --name karabuk
WORLD=karabuk bash px4/run_sitl.sh
```

- `fetch`: DEM penceresi AWS'deki bulut uyumlu GeoTIFF'ten yalnız gereken baytlar
  okunarak, OSM binaları ve yol/akarsuları Overpass API'den (ana sunucu yanıt vermezse
  yedeklerden), zemin dokusu `--imagery` ile seçilen kaynaktan karo karo indirilir
  (`world/data/`, repoya girmez). Tek parça için `--only dem|buildings|features|imagery`.
  - `esri` (kullanılan): Esri World Imagery, zoom 17, 1980 karo, ~0,5–1 m. Proje sahibinin
    kararı (2026-10-23); Esri'nin koşulları indirilen karoların Esri ürünleri dışında
    kullanılmasına izin vermiyor, bu yüzden veri ve üretilen dokular repoya girmez.
  - `sentinel2` (varsayılan): Sentinel-2 cloudless 2016 (EOX, CC BY 4.0), zoom 14, 10 m.
- `build`: arazi üçgen ağı (321 × 321, ~31 m; Gazebo heightmap'inde DART çarpışma
  yüzeyi görselden ~9 m yukarıda kaldığı için ağ kullanılıyor) ve üstüne kaplanan uydu dokusu
  (Esri'de görsel ağ 5 × 5 parçaya bölünüp her parçaya 2048 px doku, ~1 m/piksel; Sentinel'de
  tek doku), tüm binalar için tek OBJ ağ (yükseklik OSM etiketinden, yoksa 9 m), yollar ve
  akarsular için araziyi izleyen görsel şeritler (yalnız Sentinel'de; Esri görüntüsünde yollar
  zaten görünüyor) ve dünya SDF'i
  (`world/build/<ad>/`, repoya girmez). `spherical_coordinates` Gazebo
  orijinini gerçek konuma koyar; PX4'ün GNSS'i gerçek koordinatları görür.

Varsayılan bölge: Karabük çevresi, 10 × 10 km (şehir merkezi, vadiler, açık arazi).
Kalkış noktası 41,1639 / 32,6443 (275 m, eğim < 1°, en yakın bina 645 m).
Durum (2026-10-23): DEM, Esri uydu dokusu ve 3313 bina.
Test sahası seçilince kendi fotogrametri uçuşumuzla (OpenDroneMap, 2–5 cm) dijital
ikiz eklenecek.

Lisans: harita verisi © OpenStreetMap katkıcıları (ODbL); Esri dokusu: Source: Esri, Maxar,
Earthstar Geographics, and the GIS User Community (koşullar yukarıda); Sentinel dokusu Sentinel-2 cloudless
2016 by EOX IT Services GmbH (Copernicus Sentinel 2016 verisinden değiştirilmiş), CC BY 4.0
(sonraki yılların mozaikleri ticari olmayan lisanslı olduğu için 2016 kullanılıyor); yükseklik verisi
© DLR e.V. 2010–2014 ve © Airbus Defence and Space GmbH 2014–2018, Copernicus
programı tarafından sağlanmıştır.
