# PX4 yapılandırması (plan)

PX4 kaynak kodu henüz klonlanmadı (onay bekliyor). Burada tutulacaklar:

- `airframes/`: P1 tailsitter airframe dosyası (SITL ve gerçek araç).
- Control allocation: 6 aktüatör.

| Aktüatör | Askıda etkisi | Seyirde etkisi |
|---|---|---|
| Motor 1 (üst, CCW) | itki, itki ekseni etrafında tork | itki |
| Motor 2 (alt, CW) | itki, ters tork | itki |
| Vane yunuslama | yunuslama momenti | yunuslama (jet varken) |
| Vane sapma | sapma momenti | sapma (jet varken) |
| Elevon sol | (pervane akışında) zayıf | yunuslama + yuvarlanma |
| Elevon sağ | (pervane akışında) zayıf | yunuslama + yuvarlanma |

Askı etkinlik sayıları `models/aero/p1/aero.json` → `hover_allocation`
içinde (vane radyanı başına moment, devir farkı başına moment). Bu sayılar
PX4'ün `CA_*` parametrelerine çevrilecek.
