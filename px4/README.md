# PX4 entegrasyonu

PX4 v1.17.0, WSL2 Ubuntu 24.04 içinde `~/uav/PX4-Autopilot` dizininde
(`uav-p1` dalı). Kurulum: `tools/setup/wsl_setup.sh`, sonra `px4/install.sh`.

| Dosya | İşlev |
|---|---|
| `patches/0001-control-allocator-tailsitter-hover-surfaces.patch` | Yeni `CA_TS_MC_SV` parametresi: tailsitter'da ilk N kontrol yüzeyi (vane'ler) askı matrisinde rotorlarla birlikte ayrılır; koaksiyel rotorlarda tork farkıyla yaw açılır. Stok PX4'te tailsitter askıda yalnız rotorlarla kontrol edilir. |
| `airframes/4050_gz_vtol_p1` | SITL airframe; `python -m tools.px4gen` ile config'ten üretilir. |
| `install.sh` | Yamayı uygular, airframe ve Gazebo modelini üretip kaydeder, eklentiyi derler ve test eder, PX4 SITL'i derler. |
| `run_sitl.sh` | Gazebo penceresiyle SITL (`HEADLESS=1` ile pencerisiz). |
| `sitl_hover_check.sh` | Pencerisiz duman testi: kalkış, 10 m'de bekleme, iniş; `WIND=8` ile rüzgâr. |

Aktüatörler: motor 1 üst (CCW), motor 2 alt (CW); servo 1 vane yunuslama,
servo 2 vane sapma, servo 3–4 sol ve sağ elevon.

Çerçeve notu: PX4 tailsitter'da gövde ekseni askı (çok rotorlu) çerçevesidir.
Sabit kanat FRD'den dönüşüm: x_mc = z_fw (karın), y_mc = y_fw, z_mc = −x_fw (burun).
