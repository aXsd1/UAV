# vtol_dynamics — Gazebo Harmonic 6-DOF eklentisi (henüz yazılmadı)

Hazır LiftDrag ve MulticopterMotorModel eklentileri kanal içindeki koaksiyel
pervaneleri, vane'leri ve 0–90° geçiş aerodinamiğini modelleyemez. Bu eklenti
bir `gz::sim::System` olarak her adımda araç gövdesine toplam kuvvet ve moment
uygular.

## Girdi

- `models/aero/p1/*.csv` ve `aero.json` (format: `docs/conventions.md`).
- PX4'ten aktüatör komutları (gz transport): 2 motor, 2 vane ekseni, 2 elevon.

## Her adımda

1. Gövde eksenindeki hava hızı (araç hızı − rüzgâr) → V, α, β, p, q, r.
2. Gövde: `airframe_alpha` + `airframe_beta` + sönüm türevleri + elevon türevleri.
3. Pervaneler: motor gecikmesi → n₁, n₂; `rotor.csv` ile T, Q; tork farkı
   → itki ekseni etrafında moment; jiroskopik etki (I_rotor).
4. Kanal: momentum sürüklemesi `ṁ · V_dik`, çıkış hızı.
5. Vane'ler: servo hız sınırı ve ölü bölge → δ; `vane.csv` ile yan kuvvet ve
   eksenel kayıp, `exit_station_m`'de uygulanır.
6. Tüm momentler gerçek AM'ye taşınır (AM rastgeleleştirmesi desteklenir).

Python referansı (`tools/aerogen`) ile aynı sayıları verdiğini gösteren
birim testleri bu eklentiyle birlikte gelecek.
