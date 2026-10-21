# VTOL kuyruk üstü İHA (V-BAT benzeri)

Pistsiz dikey kalkıp yatay uçuşa geçen, kanal içinde zıt dönen iki pervaneyle
itilen ve kanal çıkışındaki vane'lerle itki vektörlemesi yapan otonom
tail-sitter İHA. Önce 12 kg'lık elektrikli prototip (P1), sonra 55 kg'lık seri
hibrit (P2). P1 donanım tabanı: Cube Orange+ uçuş kartı, 2 × 6S 16000 mAh LiPo (seri, 12S).

Dayanak dokümanlar:
- [Tasarım dokümanı](https://claude.ai/code/artifact/04c4a23b-eb41-4dac-b864-000eb6f73aec)
- [Simülasyon dokümanı](https://claude.ai/code/artifact/0a49e0f5-c96e-4a9c-9f7d-1928f0232544)

## Dizin yapısı

| Yol | İçerik | Durum |
|---|---|---|
| `config/p1.toml` | P1 parametrelerinin **tek kaynağı** | var |
| `tools/aerogen/` | El hesabı aero katsayı üreteci + boyutlandırma kontrolleri | var |
| `models/aero/p1/` | Üretilmiş tablolar (Gazebo ve Isaac ikisi de okur) | üretildi |
| `tests/` | Birim testleri (`unittest`, ek paket gerekmez) | var |
| `docs/` | Kurallar (eksenler, birimler, tablo formatı) ve karar kaydı | var |
| `tools/dynamics/` | Referans 6-DOF kuvvet modeli (Python) | var |
| `sim/gazebo/vtol_dynamics/` | C++ 6-DOF Gazebo Harmonic eklentisi | var, SITL'de uçtu |
| `sim/isaac/` | Isaac Lab ortamı ve PPO eğitimi | plan |
| `px4/` | PX4 yaması, airframe, kurulum ve SITL betikleri | var |
| `ros2_ws/` | ROS 2 Jazzy çalışma alanı (otonomi paketleri) | boş |
| `world/` | DEM GLO-30 + OSM'den dünya üretimi | plan |

## Hızlı başlangıç

Python 3.11+ ve numpy yeterli.

```bash
python -m tools.aerogen
```

```bash
python -m unittest discover -s tests -t .
```

İlk komut `config/p1.toml`'dan `models/aero/p1/` tablolarını ve
`sizing_report.md`'yi üretir, kontrolleri ekrana basar. Config değişince
tabloları yeniden üretip birlikte commit edin; `test_tables_uptodate`
eskimiş tabloyu yakalar.

## Akış

```
config/p1.toml ──► tools/aerogen ──► models/aero/p1/*.csv + aero.json
                                          │
                     ┌────────────────────┴───────────────────┐
                     ▼                                        ▼
       sim/gazebo/vtol_dynamics (C++)            sim/isaac (PyTorch, toplu)
       + PX4 SITL + ROS 2                        + PPO eğitimi
```

## SITL (WSL2 Ubuntu 24.04)

Adım adım çalıştırma rehberi: [docs/CALISTIRMA.md](docs/CALISTIRMA.md).

Bir kez, WSL içinde: `tools/setup/wsl_setup.sh` (PX4, Gazebo Harmonic, ROS 2 Jazzy,
XRCE-DDS Agent), sonra `px4/install.sh`. Uçurmak için `px4/run_sitl.sh`;
pencerisiz duman testi için `px4/sitl_hover_check.sh` (`WIND=8` ile rüzgârlı).

Durum (2026-10-21): 10 m askı ±5 cm, 0,5°. 8 m/s rüzgârda (3 s'de artan) en çok
0,8 m, sonra ±0,35 m; rüzgârda iniş dik kalıyor. Ani 8 m/s basamakta 1,7 m anlık sapma.
Tam görev (kalkış, ön geçiş, seyir, geri geçiş, iniş) rüzgârsız ve 5 m/s yan rüzgârda
geçiyor: ön geçiş 3,7–3,8 s, 2–3 m irtifa kaybı.
