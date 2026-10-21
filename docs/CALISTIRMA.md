# Simülasyonu çalıştırma rehberi

Bu adımlar 2026-10-21'de bu bilgisayarda (Windows 10 Pro, WSL2 Ubuntu 24.04,
PX4 v1.17.0, Gazebo Harmonic) denendi. "Denendi" yazmayanlar denenmedi.

## 1. Ubuntu'yu (WSL) root olarak açın

PX4 kurulumu root kullanıcısının klasöründe (`/root/uav`). Başlat menüsündeki
**Ubuntu 24.04** kısayolu artık `erina` kullanıcısıyla açılır ve bu kurulumu
göremez; o yüzden PowerShell'den root olarak açın:

```powershell
wsl -d Ubuntu-24.04 -u root
```

Terminal `root@...#` ile açılır.

## 2. Proje klasörüne gidin

```bash
cd /mnt/c/Users/erina/Desktop/Yeageth/workplace/UAV
```

Windows'taki `C:\Users\erina\Desktop\Yeageth\workplace\UAV` klasörüyle aynıdır.

## 3. Simülasyonu başlatın (Gazebo penceresiyle) — denendi

```bash
bash px4/run_sitl.sh
```

- İlk açılışta ~15 saniye bekleyin. Windows'ta **Gazebo Sim** penceresi açılır
  ve araç (dik duran gri gövde, mavi kanal, siyah kanat) zeminde görünür.
- Terminalde `pxh>` istemi çıkar ve `Ready for takeoff!` yazar.
- PX4 kodunda değişiklik varsa ilk açılışta derleme birkaç dakika sürebilir.

Tek komutla, doğrudan PowerShell'den de başlatılabilir:

```powershell
wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/Users/erina/Desktop/Yeageth/workplace/UAV/px4/run_sitl.sh
```

## 4. Aracı uçurun (`pxh>` istemine yazın)

| Komut | Ne olur | Durum |
|---|---|---|
| `commander takeoff` | Motorlar çalışır, araç dikey kalkar (varsayılan ~16 m) ve askıda bekler | denendi |
| `commander land` | Bulunduğu yere dikey iner ve motorları kapatır | denendi |
| `param set MIS_TAKEOFF_ALT 30` | Kalkış irtifasını 30 m yapar (`takeoff`'tan önce) | denendi |
| `commander transition` | Askıdayken: öne yatıp yatay uçuşa geçer (~3 s) ve bulunduğu noktanın çevresinde tur atar. Yatay uçuştayken: tekrar dik askıya döner (~4 s) | denendi (pencerisiz testte) |
| `commander mode posctl` | Konum tutma moduna geçer | denenmedi |
| `shutdown` | PX4'ü ve Gazebo'yu kapatır | — |

Geçiş denemek için önerilen sıra: `param set MIS_TAKEOFF_ALT 30` →
`commander takeoff` → kalkış bitince `commander transition` → 20–30 s sonra
`commander transition` → `commander land`.

Gazebo'da aracı takip etmek için sol paneldeki varlık listesinde
`vtol_p1_0`'a sağ tıklayıp **Follow** seçin.

## 5. Rüzgâr ekleyin — denendi

Simülasyon açıkken **ikinci bir terminal** açın (`wsl -d Ubuntu-24.04 -u root`) ve:

```bash
gz topic -t /model/vtol_p1_0/wind -m gz.msgs.Vector3d -p "x: -8"
```

Doğudan 8 m/s rüzgâr eser (`x`, `y`, `z` dünya ekseninde m/s). Kapatmak için:

```bash
gz topic -t /model/vtol_p1_0/wind -m gz.msgs.Vector3d -p "x: 0"
```

Araç rüzgârda kanat açıklığını rüzgâra çevirir (weathervane) ve hafifçe yatar.

## 6. Otomatik testler (pencerisiz) — denendi

Pencere açmadan uçuş yapıp sonucu yazdırırlar:

```bash
bash px4/sitl_hover_check.sh 20
WIND=8 WIND_RAMP=3 bash px4/sitl_hover_check.sh 30
bash px4/sitl_transition_check.sh 25
WIND=5 bash px4/sitl_transition_check.sh 25
```

Sonda `VERDICT: PASS` / `FAIL` yazar. Uçuş kayıtları (`.ulg`):
`/root/uav/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`.
Herhangi bir kaydı incelemek için: `python3 px4/analyze_flight.py <dosya.ulg>`.

## 7. QGroundControl (isteğe bağlı) — denenmedi

Harita, telemetri ve görev planlama için. Windows'a QGroundControl kurulursa,
WSL içindeki PX4'ün yayınını Windows'a yönlendirmek gerekir (WSL2 ayrı bir ağ
kullanır). `pxh>` istemine:

```
mavlink start -u 14560 -o 14550 -t <WINDOWS_IP>
```

`<WINDOWS_IP>`, Ubuntu terminalinde `ip route show default | awk '{print $3}'`
komutunun çıktısıdır. QGroundControl UDP 14550'yi otomatik dinler. Windows
güvenlik duvarı izin isteyebilir.

## Sorun giderme

| Belirti | Çözüm |
|---|---|
| Gazebo penceresi açılmıyor | `wsl --update` (PowerShell) ve WSL'yi yeniden başlatın: `wsl --shutdown` |
| `No such file or directory` / `~/uav` bulunamadı | Ubuntu `erina` kullanıcısıyla açılmış; `wsl -d Ubuntu-24.04 -u root` ile açın |
| `Ready for takeoff` gelmiyor | Önceki oturum açık kalmış olabilir: `pkill -x px4; pkill -f "gz sim"` ve tekrar başlatın |
| Config değişti (`config/p1.toml`) | Windows'ta `python -m tools.aerogen`, sonra Ubuntu'da `bash px4/install.sh` |
