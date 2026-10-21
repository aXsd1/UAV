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
| 13 | 2026-10-21 | Geliştirme ortamı WSL2 Ubuntu 24.04: PX4 v1.17.0, Gazebo Harmonic (gz-sim 8.15), ROS 2 Jazzy, Micro XRCE-DDS Agent 2.4.3 | Erin'in onayı; yönetici izni gerekmedi |
| 14 | 2026-10-21 | PX4 yaması `CA_TS_MC_SV`: vane'ler askı kontrol matrisinde | Stok tailsitter askıda yalnız rotor kullanır; koaksiyel rotorlar yunuslama/yuvarlanma üretemez |
| 15 | 2026-10-21 | Askı açı hızı kazançları P 2,5 / I 0,8 / D 0,04 | Atalet ~1 kg·m², vane yetkisi ~5 rad/s²; P 0,5 ile açı döngüsü salınıma girdi |
| 16 | 2026-10-21 | İniş ayağı yarıçapı 0,45 m (kanatçık ucu 0,32 m yerine), +0,1 kg paydan; batarya 0,48 m | 0,32 m'de devrilme açısı 21°, rüzgârda yatış 23° → SITL'de temasta devrildi. 0,45 m: devrilme 28,7°, yerde güvenlik 5,8 |
| 17 | 2026-10-21 | PX4 yaması `WV_AXIS=1`: weathervane kanat açıklığını rüzgâra çevirir (yakın taraf, en çok 90° dönüş) | Stok weathervane karnı rüzgâra çevirir (kanat rüzgâra dik, 29° yatış); yanlamasına 18° hesap, SITL'de ~6° |
| 18 | 2026-10-21 | α tabloları yalnız x–z düzlemindeki dinamik basınçla (q cos²β) uygulanır | Tam yan rüzgârda α tanımsızlaşıyor; eski model küçük gürültüyle 87 N sıçrıyordu (regresyon testi eklendi) |
| 19 | 2026-10-21 | Kanat: reflex profil, açıklık 1,7 m, alan 0,75 m²; seyir 21,8 m/s (1,25 × tutunma 17,4 m/s); kuyruk açısı −4,4°; kanat +0,25 kg paydan (pay 0,05 kg) | Erin'in seçimi. 1,5 m / 0,6 m² kanatta tutunma 18,5 m/s, 20 m/s seyir güvensizdi (SITL'de geçişten sonra tutunma kaybı). Yeni kanatla 40 dk |
| 20 | 2026-10-21 | FW'de elevonlar yalnız yuvarlanma; yunuslama vane'lerden (VT_FW_DIFTHR_EN 5), yatış sınırı 25°, bekleme yarıçapı 150 m, FW asgari gaz 0,10 | Kanat aerodinamik merkezi AM'ye ~1 cm: elevon yunuslama yerine taşımayı değiştiriyor, burun kaldırırken kanat taşıması çöküyordu |
| 21 | 2026-10-21 | Yatay kuyruk kanatçıklarına flaplı elevatör (veterin %30'u, tek servo, ±25°), FW yunuslama elevatörden; vane'ler FW'de yalnız sapma; pay 0 (servo 0,05 kg) | Elevonlar AM'ye çok yakın: aynı yunuslama için elevatör 7,5 kat daha az taşıma kaybettiriyor. SITL'de ön geçiş irtifa kaybı 15–18 m → 2–3 m |

## Açık sorular

- Kütle payı sıfır: herhangi bir ekleme MTOW'u artırır. Bileşenler tartılınca güncellenmeli.

- Ani 8 m/s rüzgâr basamağında 1,7 m anlık sapma (6 s'de toparlanıyor). 3 s'de artan rüzgârda en çok 0,8 m (weathervane dönüşü sırasında), sonra ±0,35 m.
- Weathervane rüzgârda ±10° avlanıyor; kazanç ayarı gerekebilir.
- Askıda yan rüzgâr yükünü kanal momentum sürüklemesi belirliyor (~3,2 kg/s × rüzgâr hızı).

- Test sahası seçilmedi; varsayılan 10 × 10 km açık alan + şehir kenarı.
- Ulaşılabilir seyir tam 40 dk (604 Wh kullanılabilir); hedefin alt sınırında, yedek enerji payı yok. Paket kütlesi tartılınca güncellenmeli.
- Askıda vane yunuslama yetkisi 8 m/s yan rüzgârda sınırda (vane-AM kolu 0,19 m).
