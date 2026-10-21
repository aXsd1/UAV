# Kurallar: eksenler, birimler, tablo formatı

Gazebo eklentisi, Isaac Lab ortamı ve PX4 yapılandırması aynı kuralları
kullanır. Bunlardan biri değişirse `models/aero/*/aero.json` içindeki
`format_version` artırılır.

## Eksenler

- **Gövde ekseni:** sabit kanat FRD. `x` burun yönü, `y` sağ kanat, `z` aşağı.
  Askıda araç dik durur, `x` yukarı bakar (PX4 tailsitter da gövde eksenini
  sabit kanat yönelimine göre tanımlar).
- **İstasyon** (`station_m`): burundan kuyruğa doğru metre. Gövde `x` = `s_ref − s`.
- **Hücum açısı** α = atan2(w, u), **yana kayma** β = asin(v / V), gövde
  eksenindeki hava hızı bileşenleriyle.
- Askıda "itki ekseni etrafında dönüş" gövde `x` ekseni etrafındaki dönüştür
  (sabit kanatta yuvarlanma). İki pervanenin devir farkıyla üretilir.

## Birimler

SI. Tablolarda açılar derece (sütun adı `_deg` ile biter), kodun içinde
radyan. Devir tablo formüllerinde rev/s (`n`), config'te rpm.

## Tablolar (`models/aero/<ad>/`)

| Dosya | Sütunlar | Anlamı |
|---|---|---|
| `airframe_alpha.csv` | `alpha_deg, CL, CD, Cm` | Pervane kapalı gövde, β = 0, α ∈ [−180°, 180°], 1° adım. CL/CD rüzgâr ekseninde, Cm referans istasyonu etrafında, `S`·`c` ile boyutsuz. |
| `airframe_beta.csv` | `beta_deg, CY, Cl, Cn` | Pervane kapalı, α = 0, β ∈ [−180°, 180°]. CY gövde `y`, Cl/Cn referans istasyonu etrafında, `S`·`b` ile boyutsuz. |
| `rotor.csv` | `J, CT, CQ` | Pervane başına: `T = CT ρ n² D⁴`, `Q = CQ ρ n² D⁵`, `J = V_eksenel / (n D)`. Kanal dudağı itkisi dahil, ünitenin yarısı. |
| `vane.csv` | `delta_deg, side_force_ratio, axial_loss_ratio` | Vane sapmasına karşı jet yan kuvveti / ünite itkisi ve eksenel itki kaybı / ünite itkisi. |
| `aero.json` | — | Referans boyutlar, kütle/atalet, sönüm ve kontrol türevleri, rotor/vane parametreleri, askı kontrol etkinliği, seyir trim noktası, config SHA-256. |

Dinamik modelin sorumlulukları (tablolarda **olmayan** etkiler):

1. α ve β'nın birlikte etkisi: v0 tablolar ayrık; model `airframe_alpha`'yı
   α'ya, `airframe_beta`'yı β'ya göre ayrı ayrı uygular.
2. Momentleri referans istasyonundan gerçek AM'ye taşımak:
   `Cm_AM = Cm_ref − (s_AM − s_ref)/c · CZ`, `CZ = −CL cos α − CD sin α`.
3. Kanal momentum sürüklemesi (`ṁ · V_dik`) ve pervane akışının kanat ve
   elevonlar üzerindeki etkisi.
4. Motor birinci derece gecikmesi, servo hız sınırı ve ölü bölge
   (`aero.json` → `rotor`, `vanes`).
5. Yer etkisi, kuyruk ayağı teması, rüzgâr ve Dryden türbülansı.

## İşaret kuralları

- Elevon simetrik: iki firar kenarı aşağı pozitif → +CL, −Cm.
- Elevon diferansiyel: `(sol − sağ)/2`, pozitif sağa yuvarlar.
- Pervaneler (VARSAYIM): burundan bakınca üst pervane saat yönü tersine (CCW),
  alt pervane saat yönüne (CW) döner.
