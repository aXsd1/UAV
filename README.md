# VTOL kuyruk üstü İHA (V-BAT benzeri)

Pistsiz dikey kalkıp yatay uçuşa geçen, kanal içinde zıt dönen iki pervaneyle
itilen ve kanal çıkışındaki vane'lerle itki vektörlemesi yapan otonom
tail-sitter İHA. Önce 12 kg'lık elektrikli prototip (P1), sonra 55 kg'lık seri
hibrit (P2).

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
| `sim/gazebo/vtol_dynamics/` | C++ 6-DOF Gazebo Harmonic eklentisi | arayüz tanımı |
| `sim/isaac/` | Isaac Lab ortamı ve PPO eğitimi | plan |
| `px4/` | Tailsitter airframe ve control allocation yapılandırması | plan |
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

Sonraki adımlar (simülasyon dokümanı uygulama sırası 3. madde): Gazebo eklentisi,
PX4 tailsitter yapılandırması, ilk SITL askısı. PX4 kaynak kodu, Gazebo,
ROS 2, Isaac Sim ve Docker kurulumu ağır olduğu için onay beklemektedir.
