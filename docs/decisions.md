# Karar kaydı

Kısa kayıtlar; gerekçenin ayrıntısı tasarım ve simülasyon dokümanlarında.

| # | Tarih | Karar | Gerekçe |
|---|---|---|---|
| 1 | 2026-10-11 | Kanal içinde koaksiyel, zıt dönen 2 pervane, her biri ayrı motor | Tork dengesi, itki ekseni etrafında dönüş devir farkıyla; stator kanadı yok |
| 2 | 2026-10-11 | İtki vektörleme kanal çıkışındaki 2 eksenli vane'lerle, + elevonlar | Erin'in seçimi |
| 3 | 2026-10-11 | Uçuş yazılımı PX4 (tailsitter, özel control allocation) | BSD lisans, esnek control allocator, uXRCE-DDS |
| 4 | 2026-10-11 | ROS 2 Jazzy + uXRCE-DDS | Otonomi kodu simülasyonda ve araçta aynı |
| 5 | 2026-10-11 | Gazebo Harmonic SITL + kendi C++ 6-DOF eklentisi; RL için Isaac Lab (PPO) | İkisi aynı aero tablolarını okur |
| 6 | 2026-10-21 | Parametrelerin tek kaynağı `config/p1.toml` (TOML) | Python 3.11+ `tomllib` ile ek paket gerekmez; C++ tarafında toml++ (tek başlık dosyası) |
| 7 | 2026-10-21 | Testler stdlib `unittest` | Ek kurulum yok; pytest de aynı testleri çalıştırır |
| 8 | 2026-10-21 | Üretilmiş tablolar repoda tutulur | Simülatörler Python'a bağımlı olmaz; eskime testle yakalanır |
| 9 | 2026-10-21 | Kanat çeyrek veteri istasyon 0,48 m (VARSAYIM) | Kanaldaki 2,6 kg burunda; kanat 0,62 m'deyken statik marj ~%40 çıktı, 0,48 m'de %10 |
| 10 | 2026-10-21 | Uçuş kontrolcüsü CubePilot Cube Orange+ (Pixhawk 6X yerine) | Erin'in elindeki/örnek projedeki kart; PX4 hedefi `cubepilot_cubeorangeplus`, üçlü IMU |
| 11 | 2026-10-21 | Batarya 2 × Profuse 6S 16000 mAh 40C LiPo, seri (12S, 44,4 V, 710 Wh, ~4,0 kg); P1 MTOW 12,7 kg, dayanıklılık hedefi 40–45 dk | Erin'in kararı. 0,6 kg pay korundu; batarya 0,47 m'de (statik marj %10,1) |
| 12 | 2026-10-21 | Motor tepe gücü 2 × 3,0 kW (2,2 kW yerine) | T/W 1,4 için ~5,6 kW mil gücü gerekiyordu. Motor kütlesi artışı (+0,2 kg VARSAYIM) paydan: pay 0,4 kg; batarya 0,49 m (statik marj %10,4) |

## Açık sorular

- Test sahası seçilmedi; varsayılan 10 × 10 km açık alan + şehir kenarı.
- Tasarım dokümanındaki askı gücü (~2,2 kW) FM ve koaksiyel kaybı içermiyor
  gibi görünüyor; üretecin hesabı 12,7 kg ile ~3,4 kW mil gücü. Bkz. `models/aero/p1/sizing_report.md`.
- Ulaşılabilir seyir tam 40 dk (604 Wh kullanılabilir); hedefin alt sınırında, yedek enerji payı yok. Paket kütlesi tartılınca güncellenmeli.
- Askıda vane yunuslama yetkisi 8 m/s yan rüzgârda sınırda (vane-AM kolu 0,19 m).
