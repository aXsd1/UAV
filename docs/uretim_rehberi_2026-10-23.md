# P1 Üretim ve Test Rehberi

Oct 23, 2026 · @Erin

P1, simülasyonda uçan 12,7 kg'lık elektrikli kuyruk üstü aracın gerçek kopyasıdır ve yaklaşık 5–6 ayda, malzeme olarak kabaca 7–10 bin USD ile üretilip ilk serbest uçuşa getirilebilir. Bu rehber sivil araştırma aracı içindir; silah veya askeri faydalı yük entegrasyonu kapsam dışıdır.

Sıra şöyledir: önce simülasyonun bulduğu tasarım açıkları kapatılır, sonra parçalar alınıp yapı üretilir, her alt sistem yerde ölçülür, ölçülen değerler `config/p1.toml`'a yazılıp simülasyon yeniden koşulur, ancak ondan sonra uçulur. Gerçek araç ile simülasyon aynı parametre dosyasından beslendiği sürece SITL'de geçen her senaryo gerçek uçuşun provası olur.

| Aşama | Süre (hafta) | Çıktı | Kapı |
| --- | --- | --- | --- |
| 0. Tasarım kararları | 1–2 | Paraşüt, pusula, iniş ayağı kararları; güncel p1.toml | Kütle bütçesi kapanıyor, SITL yeniden geçiyor |
| 1. Tedarik | 3–6 (paralel) | Bütün parçalar elde | Motor, ESC, batarya teslim |
| 2. Yapı üretimi | 4–6 | Gövde, kanal, kanat, kuyruk | Tartım ±%5 bütçe içinde |
| 3. İtki ve vane testi | 2–3 | Ölçülmüş itki, güç, vane momenti | Askı devri ve gücü hesaba ±%10 |
| 4. Entegrasyon ve HITL | 2 | Kablolu, yazılımı yüklü araç | Bütün eyleyiciler doğru yönde, HITL'de tam görev |
| 5. Gimbal standı ve bağlı askı | 2–3 | Ayarlı kontrolcü | 5 dk kararlı bağlı askı |
| 6. Serbest uçuş | 4–8 | Askı, geçiş, tam görev | 20 hatasız otonom görev |

İlk serbest uçuştan önce tasarım dokümanındaki kapı geçerlidir: simülasyonda 1.000 rüzgârlı geçişte sıfır kayıp. Bugün temiz dönemde 533 geçişte 2 kayıp var ve 467 koşu bozuk simülasyonda kaldı; bu yüzden üretim ve yer testleri başlayabilir, serbest uçuş bu kapıyı bekler.

## Üretimden önce kapatılacak kararlar

Monte Carlo koşuları beş açık bıraktı; bunlar parça siparişinden önce karara bağlanmalı, çünkü her biri kütleyi, ağırlık merkezini ya da iniş ayağı geometrisini değiştirir.

| Bulgu (simülasyon) | Gerçek araca etkisi | Önerilen karar |
| --- | --- | --- |
| Tek motor kaybı ya da tek vane servosunun takılması askıda kurtarılamıyor (50/50 kontrolsüz düşüş) | Tek bir arıza aracı ve altındakini kaybettirir | P1'e **paraşüt** (yay ya da gaz atımlı, \~0,6–0,8 kg) ve uçuş sonlandırma; ESC telemetrisiyle motor arızası algılama. Motor yedekliği P2'ye kalır |
| Kuyruk üstünde burun dikken pusula füzyonu tutarsızlaşıyor ("Compass needs calibration - Land now!") | Gerçek araçta 150 A'lik motor akımı manyetometreyi daha da bozar | İki GNSS anteniyle **GNSS yönü** (kanat uçlarında, \~1,5 m aralık); pusula yalnız yedek, motor kablolarından uzakta. SITL'de EKF2\_MAG\_TYPE seçenekleri denenmeli |
| Şehir senaryosunda 3,5 m/s üstü rüzgârda kalkışta devrilme (32/50); düzde 7+ m/s, askıda 9+ m/s inişte devrilme | Yerdeki her saniye en riskli an | İlk uçuşlarda kalkış/iniş rüzgâr sınırı **3 m/s**; kalkıştan önce kanat kenarı rüzgâra hizalanır; ayak yarıçapı 0,45 m'den büyütme SITL'de denenir; yere değince motor kesme mantığı doğrulanır |
| Kütle payı sıfır (12,7 kg'ın tamamı dağıtılmış) | Paraşüt, kablo, vida fazlası MTOW'u aşar | İlk uçuşlarda 1,0 kg'lık gimbal faydalı yükü yerine paraşüt taklır (ikisi de \~0,42 m istasyonunda); Jetson ilk uçuşlarda aynı kütlede balastla değiştirilebilir |
| Vane yunuslama momenti 8 m/s yan rüzgârda el hesabında yetmiyor (6,1 N·m, gereken 1,5 × 7,9 = 11,9 N·m) | Weathervane ile SITL'de 8 m/s'de askı tutuyor, ama pay yok | Vane'ler maksimum alanda yapılır ve momenti standda ölçülür; ölçüm gelene kadar askı rüzgâr sınırı 6 m/s |

Kararlar alınınca `config/p1.toml` güncellenir, `python -m tools.aerogen` ile tablolar yeniden üretilir ve geçiş, askı-rüzgâr, eyleyici arızası senaryoları temiz bir makinede yeniden koşulur. Paraşütün değerlendirmesi için eyleyici arızası senaryosuna "arızadan sonra paraşüt açılıyor ve araç < 6 m/s ile yere iniyor" başarı ölçütü eklenmeli.

## Araç ölçüleri

Üretim çizimleri aşağıdaki değerlerden yapılır; hepsi `config/p1.toml`'dan gelir ve simülasyon modeli (`tools/sdfgen.py`) aynı değerleri kullanır. İstasyonlar burun ucundan kuyruğa doğru metredir; "VARSAYIM" olanlar üretimde netleşip dosyaya geri yazılmalıdır.

| Parça | Ölçü | İstasyon (m) | Not |
| --- | --- | --- | --- |
| Gövde | 1,30 m boy, Ø 0,14 m | 0–1,30 | Silindirik, burunda kanal |
| Kanal | İç Ø 0,30 m, dış Ø \~0,34 m, boy 0,25 m | merkez 0,13, çıkış 0,28 | Çıkış alanı = disk alanı |
| Göbek | Ø 0,06 m | 0,08–0,16 | İki motoru taşır |
| Üst pervane | Ø 0,30 m, CCW (burundan bakınca) | 0,08 | En çok 10.000 rpm, uç hızı 157 m/s |
| Alt pervane | Ø 0,30 m, CW | 0,16 | Tork dengesi için hatvesi ayarlı |
| Vane'ler | 2 eksen (yunuslama, sapma), ±20° | \~0,29 | Kanal çıkışının hemen arkası |
| Kanat | Açıklık 1,70 m, veter 0,44 m, 0,75 m², ok açısı 0° | çeyrek veter 0,48 (hücum kenarı 0,37) | Reflex profil, Cm0 ≈ −0,02 |
| Elevonlar | Veterin %25'i, yarı açıklığın %20–95'i, ±25° | kanat firar kenarı | FW'de yalnız yuvarlanma |
| Kuyruk kanatçıkları | 4 adet, + düzeni, her biri 0,25 m × 0,18 m (0,045 m²) | çeyrek veter 1,15 | Simetrik profil |
| Yatay kanatçık açısı | −4,4° (hücum kenarı aşağı) |  | Seyir trimini sağlar |
| Elevatör | Yatay kanatçıkların veterinin %30'u, tek servo, ±25° | \~1,23 | FW'de yunuslama |
| İniş ayakları | 4 uç, yarıçap 0,45 m | 1,30 | Devrilme açısı \~28° |
| Ağırlık merkezi | Hedef 0,459 m (statik marj %10,3) | 0,459 | Batarya kızağıyla ayarlanır |
| İniş lidarı | Kuyrukta, aşağı bakan | ayak uçlarından 0,15 m yukarıda | EKF2\_RNG\_POS\_Z gerçek konuma göre |
| Burun kamerası | Burun ekseninde, ileri bakan | 0 | Askıda yukarı bakar |

Performans hedefleri (boyutlandırma raporu): MTOW 12,7 kg, tutunma 17,4 m/s, seyir 21,8 m/s, L/D \~9,6, askı devri \~8.450 rpm, askı mil gücü \~3,4 kW (elektrik \~4,0 kW), seyir elektrik gücü \~700 W, seyir süresi 40 dk.

## Malzeme listesi

Toplam malzeme kabaca 7–10 bin USD tutar; en pahalı kalemler haberleşme, yardımcı bilgisayar, paraşüt ve batarya. Aday parçalar örnektir: şartnameyi karşılayan eşdeğer alınabilir, fiyatlar hafızadan ve yaklaşıktır, siparişten önce güncel veri sayfası ve fiyat kontrol edilmelidir. Kütle sütunu p1.toml bütçesidir.

| Kalem | Şartname | Aday | Adet | Bütçe kütle | Yaklaşık fiyat (USD) |
| --- | --- | --- | --- | --- | --- |
| Motor | Dış rotorlu fırçasız, 12S, KV \~260–300, tepe ≥ 3 kW, sürekli ≥ 2,5 kW, dış çap ≤ 55 mm (göbeğe sığmalı) | T-Motor AT41xx ya da Scorpion HKIII-40xx sınıfı | 2 (+1 yedek) | itki toplamı 1,80 kg içinde | 150–250 / adet |
| ESC | 12S (tercihen 14S), sürekli ≥ 100 A, DShot ya da PWM, devir/akım/sıcaklık telemetrisi | T-Motor Flame 100A HV, Hobbywing Platinum HV 130A | 2 (+1) | itki içinde | 150–250 / adet |
| Pervane | Ø 300 mm kanallı fan pervanesi, 3–5 kanat, rijit (karbon ya da CNC), dengelenmiş; bir CCW, bir CW | Özel üretim (bkz. itki bölümü) | 2 takım (+2 yedek) | itki içinde | 100–400 / takım |
| Batarya | 6S 16.000 mAh 40C LiPo, seri → 12S, 710 Wh | Profuse 22,2 V 16000 mAh 40C | 2 takım (uçuş + şarjda) | 4,00 kg | 300–450 / paket |
| Şarj cihazı | 2 × 6S, ≥ 1 kW, denge şarjı | ISDT, SkyRC çift kanallı | 1 | – | 300–500 |
| Uçuş kartı | Üçlü IMU, PX4 desteği | CubePilot Cube Orange+ + taşıyıcı kart | 1 | aviyonik 0,80 kg içinde | 400–550 |
| GNSS | RTK'ya hazır, çift anten yön (moving baseline) desteği | CubePilot Here4 × 2 | 2 | aviyonik içinde | 250–350 / adet |
| Hava hızı | Dijital diferansiyel basınç sensörü + pitot | MS4525DO ya da DLVR tabanlı | 1 | aviyonik içinde | 50–120 |
| İniş lidarı | 0,1–12 m, ≥ 50 Hz, UART/I²C | Benewake TFmini-S / TFmini Plus | 1 | aviyonik içinde | 40–80 |
| Güç ölçer | 12S, ≥ 200 A, akım + gerilim çıkışı | Mauch HS-200-HV | 1 | aviyonik içinde | 80–120 |
| BEC | Uçuş kartı için 5,3 V ayrı hat; servolar için 8,4 V, ≥ 10 A sürekli | Mauch BEC, Castle 20A BEC | 2 | aviyonik içinde | 40–80 / adet |
| Servo (vane) | HV, dijital, metal dişli, ≥ 20 kg·cm, ≤ 0,10 s/60° (≥ 600°/s) | KST, Savox, MKS sınıfı | 2 (+1) | vane 0,40 kg içinde | 80–150 / adet |
| Servo (elevon, elevatör) | HV, dijital, ≥ 10 kg·cm, ince kanat tipi | KST X10 sınıfı | 3 (+1) | 0,05 kg + kanat içinde | 60–120 / adet |
| Kumanda + telemetri + video | Tek bağlantıda RC, MAVLink ve HD video; Cube ekosistemiyle uyumlu | CubePilot Herelink | 1 takım | aviyonik içinde | 1.000–1.400 |
| Yedek telemetri | 868 ya da 433 MHz SiK | Holybro SiK V3 | 1 çift | aviyonik içinde | 60–100 |
| Yardımcı bilgisayar | Jetson Orin NX 16 GB + taşıyıcı kart | Seeed reComputer, Holybro Pixhawk Jetson Baseboard | 1 | 0,70 kg | 900–1.300 |
| Kameralar | Burun kamerası (ileri), aşağı bakan global shutter (VIO) | Arducam/e-con global shutter | 2 | yardımcı içinde | 100–300 |
| Paraşüt sistemi | 12–14 kg için iniş hızı ≤ 6 m/s, yay ya da gaz atımlı, servo/PWM tetik | Fruity Chutes, Mars Parachutes, Opale sınıfı | 1 | 0,6–0,8 kg (faydalı yük yerine) | 500–1.000 |
| Karbon kumaş | 200 g/m² twill + UD şerit | – | \~10 m² | yapı 3,85 kg içinde | 300–500 |
| Cam elyaf | 80–160 g/m² | – | \~6 m² | yapı içinde | 60–120 |
| Epoksi | Laminasyon epoksisi + sertleştirici, vakum torbası, ayırıcı | – | \~5 kg | – | 150–300 |
| Köpük | XPS (kanat çekirdeği), Rohacell/PVC (sandviç perdeler) | – | – | yapı içinde | 80–200 |
| Karbon boru/çubuk | Kanat kirişi Ø 16–20 mm boru, kuyruk ayak çubukları | – | – | yapı içinde | 60–150 |
| 3B baskı malzemesi | PETG-CF ya da PA-CF (perdeler, braketler, kalıplar) | – | 2–3 kg | yapı içinde | 80–150 |
| Bağlantı elemanları | Alüminyum göbek, rulman, M3/M4 paslanmaz vida, rot başları | – | – | – | 100–200 |
| Güç kablolama | 8 AWG ana hat, 10 AWG ESC, XT90-S kıvılcımsız soket, düşük ESR kapasitör | – | – | aviyonik içinde | 80–150 |
| İtki standı | 300 N yük hücresi, tork ölçümü, akım/devir | Hazır (Tyto Robotics) ya da yük hücresi + HX711 ile kendi yapımı | 1 | – | 150 (kendi) – 3.000 (hazır) |
| Gimbal test standı | 3 eksen serbest mafsal, araca AM'den bağlanır | Kendi yapımı (alüminyum profil + kardan mafsal) | 1 | – | 150–300 |
| Atalet ve LiPo güvenliği | Hassas tartı (±1 g), LiPo çantası/kutusu, yangın söndürücü (D sınıfı ya da kum) | – | – | – | 150–300 |

## Gövde, kanat ve kuyruk üretimi

Yapının toplam bütçesi 3,85 kg'dır (gövde 1,50, kanal 0,60, kanat 1,25, kuyruk 0,50); her parça bitince tartılır ve %5'ten fazla aşım bir sonraki parçada telafi edilir. P1 için kalıp yerine çoğu yerde 3B baskı ve el serimi yeterlidir; P2'de dişi kalıba geçilir.

### Gövde (1,30 m, Ø 0,14 m)

1. Hazır karbon boru (Ø 140 mm, et kalınlığı \~1 mm) bulunursa kullanılır; bulunmazsa 3B baskı ya da torna köpükten erkek kalıp yapılır, iki yarım kabuk 2–3 kat 200 g/m² karbon ile vakum torbada serilir.
2. Perdeler (bulkhead) 3 mm PVC köpük üzeri karbon sandviçten ya da PA-CF baskıdan: kanal arka flanşı (\~0,28), batarya kızağı ön/arka (0,38–0,55), kanat bağlantısı (0,37 ve 0,81), aviyonik rafı (0,55–0,65), kuyruk bağlantısı (1,10 ve 1,28).
3. Batarya kızağı boylamasına ±4 cm kaydırılabilir olmalı; kızağın üzerine 1 cm aralıklı işaret konur. Ağırlık merkezi buradan ayarlanır.
4. Erişim kapakları: batarya (üstte, hızlı değişim), aviyonik, paraşüt çıkış kapağı. Paraşüt ağızı kanat ile kanal arasında, pervaneden uzak tarafa bakmalı; asklı gövde bağlantısı ağırlık merkezine yakın.
5. Kablo kanalı: kanal göbeğinden kuyruğa kadar bir güç kanalı ve ayrı bir sinyal kanalı (manyetik ve elektriksel gürültü için en az 5 cm aralık).

### Kanat (1,70 m × 0,44 m, reflex profil)

1. Profil: kuyruksuz uçaklar için reflex bir profil (ör. MH 45 / MH 60 ailesi), Cm0 ≈ −0,02, kalınlık \~%10. Profil seçildikten sonra katsayılar XFOIL ile kontrol edilip p1.toml'daki eğim ve sıfır taşıma açısı güncellenir.
2. Çekirdek: XPS köpük, sıcak telle iki yarı kanat kesilir (her biri 0,78 m + gövde geçişi).
3. Kiriş: çeyrek veter hattında Ø 16–20 mm karbon boru (gövdede geçmeli, kanatlar sökülebilir) + üst/alt karbon UD şerit kap.
4. Kaplama: 2 kat 80–110 g/m² cam elyaf, vakum torbada; firar kenarına UD karbon.
5. Elevonlar veterin arka %25'i, yarı açıklığın %20–95'i; ayrı menteşe yerine kaplamada kesik ve üst yüzeyde kumaş menteşe. Servo kanat içinde, kontrol kolu kapakla.
6. Kanat bağlantısı gövdeye iki pim + bir vida; askıda kanat düşey durduğu için yan yük taşır, pimler bunu karşılamalı.

### Kuyruk ve iniş ayakları

1. Dört kanatçık: 0,25 m × 0,18 m, simetrik profil (NACA 0010 gibi), cam/karbon kaplı köpük ya da 3 mm karbon sandviç levha.
2. Yatay kanatçıklar −4,4° açıyla takılır (hücum kenarı aşağı); açı bir 3B baskı açı şablonuyla kontrol edilir. Bu açı seyir trimini sağladığı için ±0,5° içinde olmalı.
3. Elevatör: yatay kanatçıkların arka %30'u, iki taraf tek bir tork borusuyla bağlı, tek servo.
4. İniş ayakları: dört karbon çubuk kanatçık köklerinden 0,45 m yarıçapa uzanır; uçlarda lastik ayak (darbe sönümü ve sürtünme için). Ayaklar inişte aracın bütün ağırlığını ve 1 m/s dikey teması taşımalı; sökülebilir olursa taşıma kolaylaşır.
5. Kuyruk ucunda lidar braketi: lidar ayak uçlarından 0,15 m yukarıda, doğrudan aşağı bakar; toz ve çimden korunmalı.

### Yapısal kontroller

- Kanat uçtan yükleme: 2,5 g eşdeğeri (\~310 N dağılımı) kum torbasıyla, kalıcı şekil değişimi yok.
- Kanal-gövde bağlantısı: 1,4 × ağırlık itkisi + titreşim; motorlar çalışırken bağlantıda çatlak ve gevşeme kontrolü.
- İniş ayağı: 0,3 m'den bırakma testi (boş gövde + balast).

## Kanal ve koaksiyel itki birimi

İtki biriminin başarısını üç şey belirler: pervane ucu ile kanal duvarı arasındaki boşluk, kanal giriş dudağının yuvarlaklığı ve iki pervanenin tork dengesi. Hesaptaki √2 kanal kazancı ancak bunlar tutarsa elde edilir; tutmazsa askı gücü 3,4 kW'ın üstüne çıkar ve 40 dk seyir hedefi kaybolur.

### Kanal

1. İç çap 300 mm, boy 250 mm. Pervane düzlemlerinde iç çap ±0,5 mm tolerans; pervane ucu boşluğu hedefi 1,0–1,5 mm (çapın %0,3–0,5'i).
2. Kalıp: torna ya da 3B baskı silindirik mandrel (yüzeyi zımparalanıp ayırıcı sürülür). 3–4 kat karbon twill, pervane düzlemlerinde 2 kat ek UD halka (yuvarlaklığı korur).
3. Giriş dudağı: yarıçap \~30–40 mm (çapın %10–13'ü), 3B baskı + cam kaplama. Kanallı fanda itkinin önemli bir kısmı dudak üzerindeki emme basıncından gelir; dudak keskin olursa kazanç kaybolur.
4. Çıkış: çıkış alanı disk alanına eşit (difüzor yok); vane'ler çıkışın hemen arkasındadır.
5. Pervane düzlemlerinde kanal duvarına Kevlar ya da ince Kevlar/karbon zırh şeridi: pervane kopmasında parçanın dışarı çıkmasını sınırlar.

### Göbek ve destek kolları

1. Göbek Ø 60 mm, iki motoru sırt sırta taşır: üst motor 0,08, alt motor 0,16 istasyonunda. Alüminyum (6061) torna gövde, motor flanşları her iki uçta.
2. 3 ya da 4 destek kolu göbeği kanal duvarına bağlar; kollar akış yönünde ince simetrik profil (stator gibi akışı çevirmez, yalnız taşır). Kollardan biri motor ve ESC kablolarını taşır.
3. Pervaneler arasındaki mesafe \~80 mm (çapın \~%25'i); alt pervanenin üst pervane izinden kaybı hesapta %6 alındı.

### Motorlar

- Askıda motor başına \~1,7 kW mil gücü ve \~8.450 rpm; tepe (T/W 1,4) \~2,8 kW. 12S (44,4 V nominal, yükte \~42 V) ile 10.000 rpm için KV \~260–300 seçilir; son KV pervane hatvesiyle itki standında kesinleşir.
- Motor dış çapı göbeğe sığmalı (≤ 55 mm) ya da göbek motor çapına göre büyütülür ve p1.toml'daki hub\_diameter\_m güncellenir.
- Motorlar kanal içindeki akışla soğur; yer testlerinde 2 dk tam askı gücünden sonra sargı sıcaklığı < 90 °C olmalı.

### Pervaneler

- Ø 300 mm, kanallı fan için uç veteri geniş, 3–5 kanatlı; üst pervane CCW, alt pervane CW (burundan bakınca).
- Seçenekler: (a) hazır 12" karbon pervanenin ucundan kanal çapına kırpılması ve dengelenmesi (hızlı, ilk testler için); (b) CNC'de alüminyum ya da kalıpta karbon özel fan (kalıcı çözüm). 3B baskı pervane 10.000 rpm'de güvenli değildir.
- Tork dengesi: iki pervane aynı devirde eşit tork verecek şekilde alt pervanenin hatvesi biraz yüksek seçilir. İtki standında iki pervane birlikte çalışırken net tork ölçülür; askı devrinde net tork ≈ 0 olmalı (devir farkıyla yapılan yuvarlanma kontrolü ±%10 devir farkında 0,77 N·m üretir; trim için bunun yarısından fazlası harcanmamalı).
- Her pervane statik ve dinamik dengelenir; titreşim uçuş kartı ivmeölçerinde 30 m/s²'yi geçmemeli.

### ESC'ler

- ESC'ler göbeğin hemen arkasında, kanal çıkış akışında soğuyacak yerde; motor-ESC kabloları kısa, batarya-ESC kablosu uzun olduğu için ESC girişine düşük ESR kapasitör eklenir.
- Devir telemetrisi PX4'e bağlanır: motor durması ve devir sapması arıza algılaması ve paraşüt kararı için kullanılır.

## İtki vektörleme vane'leri ve kontrol yüzeyleri

Askıda yunuslama ve yuvarlanma momentinin tamamı vane'lerden gelir; vane'lerin boşluğu, hızı ve yönü aracın havada kalıp kalmayacağını belirler. Simülasyondaki varsayımlar: ±20° sapma, 25°'de tutunma, jet sapması / vane sapması 0,8, servo hızı 300°/s, ölü bölge 0,5°.

### Vane geometrisi

1. İki bağımsız set: biri yunuslama (servo 0), biri sapma (servo 1; askıda bu eksen PX4'te MC yuvarlanmasıdır). Her set kanal çıkışı boyunca uzanan 2–3 paralel kanatçıktan oluşur ve tek servoyla birlikte döner (paralel bağlantı çubuğu).
2. Kanatçık veteri 50–60 mm, simetrik profil (NACA 0012), menteşe çeyrek veterde: basınç merkezi menteşeye yakın kalır, servo yükü küçük olur.
3. Malzeme: karbon sandviç ya da katı karbon; mil Ø 3–4 mm çelik, iki uçta rulman. Jet \~39 m/s ve 917 Pa dinamik basınç taşır; titreşimde kırılmamalı.
4. Moment hedefi: el hesabında vane'ler askı itkisinin \~16°'sini saptırıp \~34 N yan kuvvet ve 0,18 m kolla 6,1 N·m üretir. 8 m/s yan rüzgâr için istenen 11,9 N·m'ye vane alanı tek başına yetmez (jet sapması vane tutunmasıyla sınırlı). Bu yüzden vane'ler kanal çıkışını tam kaplayacak alanda yapılır, moment standda ölçülür ve rüzgâr sınırı ölçüme göre belirlenir.

### Servolar ve bağlantı

- Servo yükü küçüktür (menteşe momenti kanatçık başına \~0,05 N·m mertebesi), ama bağlantı rijitliği ve hız önemlidir: ≥ 600°/s, boşluk ≤ 0,5°. 20 kg·cm'lik metal dişli servo, kısa ve rijit rot, rot başlarında bilyalı mafsal.
- Servolar 8,4 V ayrı BEC'ten beslenir; uçuş kartının 5 V hattına servo akımı girmez.
- Servo arızası tek başına aracı düşürür (simülasyon bulgusu). Servo seçiminde güvenilirlik önceliklidir; her test gününden önce boşluk ve sıcaklık kontrol edilir.

### Elevonlar ve elevatör

- Elevonlar yalnız FW yuvarlanması için (CA\_SV\_CS2/CS3, TRQ\_Y ±0,5); simetrik sapma taşımayı bozduğu için yunuslamaya kullanılmaz.
- Elevatör FW yunuslaması için (CA\_SV\_CS4, TRQ\_P −1,0: firar kenarı aşağı = burun aşağı).
- Askıda elevonlar kilitli (VT\_ELEV\_MC\_LOCK 1).

### Kalibrasyon ve yön kontrolü

1. Her servonun nötrü mekanik olarak (3B baskı açı şablonu) ayarlanır, sonra PX4 trim'i ile ince ayar.
2. Sapma sınırları ölçülür: vane ±20°, elevon ve elevatör ±25°; gerçek sınır farklıysa p1.toml ve airframe güncellenir.
3. Yön testi (en kritik adım): araç dik dururken elle burun bir yana eğilir; vane'ler jeti aracı geri düzeltecek tarafa saptırmalı. Aynısı her eksen ve FW modunda elevon/elevatör için yapılır. Ters bağlanmış tek bir yüzey ilk kalkışta aracı kaybettirir.
4. Servo hızı basamak komutuyla ölçülür (kamera 240 fps ya da servo geri bildirimi); 300°/s'nin altındaysa simülasyon değeri düşürülüp kazançlar SITL'de yeniden doğrulanır.

## Elektronik ve kablolama

Elektrik sistemi üç ayrı hatta kurulur: 12S yüksek akım (batarya → ESC → motor), 8,4 V servo hattı ve 5,3 V aviyonik hattı. Hatlar ayrı BEC'lerden beslendiği için servo akımı ya da motor gürültüsü uçuş kartını sıfırlayamaz.

### Güç hattı

- İki 6S paket seri bağlanır (44,4 V nominal, 50,4 V dolu). Seri bağlantı için tek bir XT90-S (kıvılcımsız) ana soket; paketler aynı şarj döngüsünde ve aynı yaşta tutulur.
- Tepe akım \~150 A (T/W 1,4), askı \~95 A, seyir \~16 A. Ana hat 8 AWG silikon kablo, ESC dalları 10 AWG; batarya-ESC mesafesi \~0,3 m olduğu için her ESC girişine düşük ESR kapasitör.
- Akım/gerilim ölçer (Mauch HS-200-HV) ana hatta; PX4 batarya tahmini ve düşük batarya failsafe'i buradan.
- Harici kesme: gövde dışından erişilen bir güvenlik anahtarı ya da çıkarılabilir köprü (arming plug). Yerde motor gücü yalnız bu takılıyken gelir.

### Uçuş kartı ve sensörler

| Birim | Bağlantı | Yerleşim |
| --- | --- | --- |
| Cube Orange+ | – | AM'ye yakın (\~0,46–0,60), titreşim sönümlü tabla; ok işareti burun ekseninde |
| GNSS 1 ve 2 (Here4) | CAN | Kanat uçlarına yakın, \~1,5 m aralık: çift anten GNSS yönü, motor akımından uzak manyetometre |
| Hava hızı | I²C | Pitot kanat hücum kenarında, uçlara yakın; kanal jetinin dışında ve burun ekseni doğrultusunda |
| İniş lidarı | UART | Kuyruk ucu, aşağı bakar |
| ESC telemetrisi | UART (ya da DShot telemetri) | Devir, akım, sıcaklık |
| Herelink hava ünitesi | UART (MAVLink) + HDMI (kamera) | Antenler kanat ve kuyrukta, karbondan uzak |
| Yedek telemetri (SiK) | UART | Antenler karbon gövdenin dışında |
| Jetson Orin NX | Ethernet ya da UART (uXRCE-DDS) | 0,52 istasyonu; ayrı 5 V/12 V BEC |
| Paraşüt tetik | PWM çıkışı (uçuş sonlandırma) | Paraşüt ağızı |

Karbon fiber radyo sinyalini perdeler: bütün antenler gövdenin dışında ya da cam elyaf kaplı bölümlerde olmalıdır. Kanat cam elyaf kaplamalı olduğu için antenler için iyi yerdir.

### Eyleyici çıkışları

| Çıkış | İşlev | PX4 karşılığı |
| --- | --- | --- |
| Motor 1 | Üst pervane (CCW) | CA\_ROTOR0, KM +0,0309 |
| Motor 2 | Alt pervane (CW) | CA\_ROTOR1, KM −0,0309 |
| Servo 1 | Yunuslama vane'i | CA\_SV\_CS0 |
| Servo 2 | Sapma vane'i (askıda yuvarlanma) | CA\_SV\_CS1 |
| Servo 3 | Sol elevon | CA\_SV\_CS2 |
| Servo 4 | Sağ elevon | CA\_SV\_CS3 |
| Servo 5 | Elevatör | CA\_SV\_CS4 |
| Servo 6 | Paraşüt | Uçuş sonlandırma çıkışı |

### Paraşüt

- 12,7 kg için iniş hızı ≤ 6 m/s veren kubbe; askı ve düşük hızda açılabilmesi için yaylı ya da gazlı fırlatma şart (pasif açılma askıda işe yaramaz).
- Açılmadan önce motorlar kesilir; aksi halde ip pervanelere dolanır. PX4'te uçuş sonlandırma motorları durdurup paraşüt çıkışını tetikler.
- Tetik kaynakları: pilot anahtarı (her zaman), PX4 arıza algılayıcı (aşırı yatış/yunuslama), motor durması. Otomatik tetik, geçişte yanlış tetiklemediği SITL'de kanıtlanmadan açılmaz.
- En az 30 m güvenli açılma irtifası hedeflenir; paraşütün gerçek açılma süresi yerde ölçülür ve ilk uçuşlardan önce bir kez kum torbasıyla yüksekten (vinç ya da yüksek nokta) denenir.

### Elektrik mimarisi

&#91;embedded content: elektrik mimarisi · güç ve sinyal hatları\]

Motor akımı yalnız üst sıradaki hatta akar; servolar ve uçuş kartı kendi BEC'lerinden beslendiği için bir servonun sıkışması ya da ESC gürültüsü kartı sıfırlayamaz.

## Montaj sırası, tartım ve ağırlık merkezi

Montaj, her adımda tartıp ağırlık merkezini güncelleyerek ilerler; son adımda araç 12,7 kg ± 0,3 kg ve AM 0,459 m ± 5 mm olmalıdır. Aşım olursa p1.toml'a gerçek kütle yazılır ve boyutlandırma raporu yeniden üretilir: 12,7 kg'ın üstü askı gücünü ve seyir süresini doğrudan düşürür.

1. **İtki birimi:** göbek, motorlar, ESC'ler kanala takılır; pervaneler takılmadan motor yönü ve ESC kalibrasyonu yapılır. Pervane takıldıktan sonra uç boşluğu el ile çevrilerek tüm çevrede kontrol edilir.
2. **Kanal → gövde:** kanal arka flanşı gövde ön perdesine cıvatalanır ve yapıştırılır; eksen kaçıklığı < 0,5° (itki ekseni gövde ekseninde olmazsa askıda sürekli vane trimi gerekir).
3. **Vane'ler:** set, servo, rot; nötr ve sınırlar şablonla.
4. **Kuyruk ve ayaklar:** kanatçıklar 90° aralıkla, yatay kanatçıklar −4,4°; ayak uçları aynı düzlemde (araç düz zeminde sallanmamalı).
5. **Kablolama:** güç hattı, BEC'ler, aviyonik rafı, uçuş kartı, GNSS, pitot, lidar, haberleşme. Her konnektör etiketlenir; güç ve sinyal kanalları ayrı.
6. **Kanat:** kiriş boruyla gövdeye, servolar bağlı, GNSS ve antenler kanat üzerinde.
7. **Paraşüt** ve (sonra) yardımcı bilgisayar ya da balast.
8. **Tartım ve AM:** araç yatay, iki tartı üzerinde (burun ve kuyruk altı) tartılır; AM = (kuyruk tartısı × tartılar arası mesafe / toplam) + burun destek istasyonu. Batarya kızağı AM 0,459 m'ye gelene kadar kaydırılır ve kilitlenir. Aynı ölçüm yanal AM için de yapılır (yanal sapma < 3 mm).
9. **Atalet:** araç iki ipten (bifilar sarkaç) asılarak her eksende salınım periyodu ölçülür, atalet hesaplanır. Simülasyonda \~1 kg·m² tahmin edilen değerler ölçümle değiştirilir; askı kazançları (MC\_\*RATE\_P) bu ataletle doğrudan ölçeklenir.
10. **Simülasyonu güncelleme:** tartılan kütleler, istasyonlar, atalet, ölçülen itki ve vane momenti p1.toml'a yazılır; `python -m tools.aerogen`, `tools/sdfgen.py` ve `tools/px4gen.py` yeniden çalıştırılır; birim testleri ve SITL askı/geçiş kontrolleri tekrar geçmeli.

## Yazılım: PX4 ve parametreler

Gerçek araç simülasyondakiyle aynı PX4 sürümünü (v1.17.0) ve aynı iki yamayı kullanır; fark yalnız çıkış eşlemesi, sensör sürücüleri ve ölçülen değerlerle güncellenen kazançlardır. Böylece SITL'de geçen her ayar araçta da aynı anlamı taşır.

### Derleme

1. PX4 v1.17.0'a `px4/patches/0001-control-allocator-tailsitter-hover-surfaces.patch` (CA\_TS\_MC\_SV: vane'ler askı matrisinde) ve `0002-weathervane-side-on-axis.patch` (WV\_AXIS) uygulanır.
2. Hedef `make cubepilot_cubeorangeplus_default`; çıkan .px4 dosyası QGroundControl ile yüklenir.
3. Gerçek araç için airframe: `tools/px4gen.py`'ye bir donanım çıktısı eklenir. Aynı CA\_*, MC\_*, VT\_*, FW\_*, WV\_\* ve EKF2\_RNG\_\* değerlerini üretir; SIM\_GZ\_\* yerine fiziksel çıkış fonksiyonlarını yazar. Böylece iki airframe aynı p1.toml'dan gelir ve elle kopyalama olmaz.

### Parametreler: simülasyondan farklı olanlar

| Konu | Simülasyon | Gerçek araç |
| --- | --- | --- |
| Motor çıkışları | SIM\_GZ\_EC\_FUNC1/2 | Cube'un FMU (AUX) çıkışlarında DShot + telemetri; QGC Actuators sekmesinde Motor 1/2 |
| Servo çıkışları | SIM\_GZ\_SV\_FUNC1–5 | MAIN çıkışları, PWM 50–333 Hz (servonun desteğine göre), trim ve sınırlar ölçümden |
| Hava hızı | Eklentinin yayımladığı değer | Gerçek pitot; yerde sıfır kalibrasyonu, uçuşta ölçek faktörü (ASPD\_SCALE) doğrulaması |
| Yön kaynağı | Manyetometre | Çift anten GNSS yönü birincil (EKF2\_GPS\_CTRL yön biti), manyetometre yedek; EKF2\_MAG\_TYPE seçimi önce SITL'de |
| İniş lidarı | gpu\_lidar | Gerçek sürücü (ör. TFmini), EKF2\_RNG\_POS\_Z gerçek konuma göre |
| Askı kazançları | MC\_ROLLRATE/PITCHRATE P 2,5, I 0,8, D 0,04 | Başlangıç için yarısı; gimbal standında ve bağlı askıda artırılır |
| Askı gazı | MPC\_THR\_HOVER 0,71 | İtki standı ölçümünden; ilk askı uçuşunda log'dan düzeltilir |
| Kayıt | Varsayılan | SDLOG\_PROFILE'da yüksek hızlı ve sistem tanılama profilleri açık |

### Failsafe ve güvenlik parametreleri

- Batarya: BAT\_\* hücre sayısı 12, düşük ve kritik eşikler askı inişi için yeterli enerji bırakacak şekilde (kritikte RTL değil iniş). Seyirde 40 dk hedefinin yedek payı yok; ilk uçuşlar en çok 15 dk.
- Bağlantı kaybı: RTL (simülasyonda 50/50 başarılı, medyan iniş hatası 5 cm).
- GNSS kaybı: simülasyonda 60 s'de medyan 58 m sürüklenme; VIO olmadan GNSS kaybında hemen iniş.
- Geofence: test sahası sınırı ve 120 m irtifa tavanı.
- Quad-chute: FW'de aşırı yunuslama/yatış ya da irtifa kaybında askıya dönüş (VT\_FW\_QC\_\*); FW seyir irtifası simülasyondaki 30 m yerine ilk uçuşlarda 60–80 m.
- Uçuş sonlandırma ve paraşüt: CBRK\_FLIGHTTERM devre dışı bırakılır, arıza algılayıcı açıları (FD\_FAIL\_P/R) ve paraşüt çıkışı tanımlanır; ilk uçuşlarda yalnız pilot anahtarı.
- Kalkış öncesi kontroller (COM\_ARM\_\*) açık kalır; simülasyon kayıtlarında sık görülen "ESC failure detected" uyarısı gerçek araçta ESC telemetrisi bağlanınca anlamlı hale gelir ve göz ardı edilmez.

### Kalibrasyon (QGroundControl)

Sensör yönü (kart ok işareti burun ekseninde olduğu için tailsitter düzeltmesine dikkat), ivmeölçer, jiroskop, manyetometre (motor akımı kompanzasyonu ile), seviye, hava hızı, kumanda, ESC. Ardından Actuators sekmesinde her motor ve servo tek tek test edilir; yön kontrolü vane bölümündeki yöntemle yapılır.

### Yardımcı bilgisayar

Jetson'da ROS 2 Jazzy ve Micro XRCE-DDS Agent simülasyondakiyle aynı sürümler. İlk uçuşlarda yardımcı bilgisayar yalnız kayıt ve kamera içindir; PX4'e komut göndermez. Otonomi ve RL politikaları ancak 20 hatasız PX4 görevinden sonra devreye alınır.

## Yer testleri

Araç havalanmadan önce her alt sistem yerde ölçülür ve sonuç simülasyon varsayımıyla karşılaştırılır. Bir testin kapısı geçilmeden sonrakine geçilmez; ölçüm hesaptan %15'ten fazla saparsa p1.toml güncellenip SITL yeniden koşulur.

| # | Test | Düzenek | Ölçülen | Kapı ölçütü |
| --- | --- | --- | --- | --- |
| Y1 | Tezgah (pervanesiz) | Araç sehpada, pervaneler sökülü | Sensörler, haberleşme, her çıkışın doğru eyleyiciye gitmesi, failsafe'ler (kumanda kapat, telemetri kes) | Bütün eyleyiciler doğru ve doğru yönde; her failsafe beklenen moda geçiyor |
| Y2 | Tek pervane itki standı | Kanal + göbek + tek motor, yük hücresi | İtki, tork, devir, akım, sıcaklık; devir–itki eğrisi | Eğri CT'den ±%10; sargı < 90 °C |
| Y3 | Koaksiyel itki standı | İki pervane birlikte | Askı itkisi (124,5 N) için devir ve elektrik gücü; net tork; tepe itki (174 N) | Askı devri \~8.450 rpm ve elektrik gücü \~4,0 kW, ±%10; net tork ≈ 0; T/W ≥ 1,4 |
| Y4 | Titreşim | Y3 düzeneğinde uçuş kartı takılı | İvmeölçer spektrumu, kırpma (clipping) | Kırpma yok, titreşim < 30 m/s² |
| Y5 | Vane momenti | Araç AM'den mafsallı bağlı, kol ucunda yük hücresi | Askı itkisinde ±20° vane ile moment; servo hızı | ≥ 6,1 N·m (hesap); 11,9 N·m'ye ne kadar yakın olduğu rüzgâr sınırını belirler |
| Y6 | Batarya | Gerçek yük profili (2 dk askı + seyir akımı) elektronik yükle ya da standda | Gerilim düşüşü, sıcaklık, kullanılabilir Wh | ≥ 604 Wh kullanılabilir; paket < 60 °C |
| Y7 | Paraşüt | Yerde fırlatma, yüksekten kum torbası | Açılma süresi, irtifa kaybı, iniş hızı | \~12,7 kg ile ≤ 6 m/s; 30 m'den tam açılma |
| Y8 | HITL | Cube gerçek kart, PX4 HITL ile simülasyon | Tam görev: kalkış, geçiş, seyir, geri geçiş, iniş; gerçek kart yükü | 10 ardışık başarılı görev; işlemci yükü < %70 |
| Y9 | Gimbal standı | Araç AM'den 3 eksen serbest mafsala bağlı, motorlar çalışır | Askı tutumu kontrolü, basamak yanıtları, kazanç ayarı | Yatış/yunuslama basamağında aşım < %20, salınım yok |
| Y10 | Bağlı askı | Araç kuyruktan ya da AM'den ipe bağlı, ip boşken 1–2 m yükselir | Konum tutma, gaz, sıcaklık, batarya tüketimi | 5 dk kararlı askı; konum hatası < 0,5 m (simülasyon medyanı 0,27 m) |

Gimbal standı (Y9) bu araç için en değerli testtir: vane yönü ya da kazanç hatalıysa araç düşmez, yalnız standda döner. Stand AM'yi taşımalı, pervane dönüş alanının dışında kalmalı ve bir operatör güvenlik kesmesinin başında durmalıdır.

Her testin log'u (`.ulg`) saklanır ve `px4/analyze_flight.py` ile simülasyondaki aynı senaryonun log'uyla karşılaştırılır. Karşılaştırma, simülasyon modelinin düzeltilmesi için sistem tanılama verisidir (tasarım dokümanındaki "model hatası < %15" kapısı).

## Uçuş testi aşamaları

Uçuş testleri askıdan tam göreve yedi aşamada ilerler; her aşama simülasyonda aynı rüzgârla önceden koşulur ve gerçek log simülasyon log'uyla karşılaştırılmadan bir sonrakine geçilmez. İlk geçişe kadar rüzgâr sınırı 3 m/s'dir.

| Aşama | Uçuş | Rüzgâr sınırı | Kapı |
| --- | --- | --- | --- |
| U1. Alçak askı | El ile (Stabilized/Altitude), 2–5 m, 1–2 dk | ≤ 3 m/s | 5 uçuşta devrilmeden kalkış ve iniş; MPC\_THR\_HOVER doğrulandı |
| U2. Konum tutma ve iniş | Position modu, 10 m, otomatik iniş (lidar ile) | ≤ 3 m/s | Konum hatası < 0,5 m; iniş algılama her seferinde çalışıyor |
| U3. Rüzgârda askı | Weathervane açık, 10–30 m | Kademeli 3 → 6 m/s | Vane doyumu < %70; Y5'teki momente göre üst sınır |
| U4. Geçiş provası | 60–80 m'de geçiş başlatılıp erken geri alınır (askıya dönüş) | ≤ 3 m/s, burun rüzgâra | Askıya dönüş kararlı; irtifa kaybı < 5 m |
| U5. Tam geçiş ve kısa seyir | Geçiş, 1–2 dk düz seyir (21,8 m/s), geri geçiş | ≤ 3 m/s | Geçiş < 5 s ve < 5 m irtifa kaybı; hava hızı doğru |
| U6. Tam görev | Otonom görev: kalkış, geçiş, rota, RTL, iniş; süre 10 → 30 dk | 3 → 6 m/s kademeli | Seyir gücü \~700 W; enerji tüketimi hesaba ±%15 |
| U7. Güvenilirlik | Aynı görevin tekrarı, failsafe denemeleri (bağlantı kesme, RTL) | sahaya göre | 20 hatasız otonom görev (tasarım dokümanı kapısı) |

### Her uçuş gününde

1. Uçuş öncesi kontrol listesi: yapı, pervane çatlağı ve uç boşluğu, vane boşluğu, batarya gerilimi ve denge, AM işareti, paraşüt pimi, failsafe ayarları, rüzgâr ölçümü (el anemometresi, kalkış noktasında).
2. Kalkıştan önce araç, kanat kenarı rüzgâra bakacak şekilde konur (simülasyonda weathervane'in seçtiği yön; yanlamasına yatışı azaltır).
3. Ekip: pilot (manuel devralım ve paraşüt anahtarı), yer istasyonu operatörü (telemetri, sınırlar), gözcü (hava sahası ve insanlar).
4. Uçuş sonrası: log indirilir, analiz betikleriyle simülasyonla karşılaştırılır, bulgular karar kaydına (`docs/decisions.md`) işlenir.

### İptal kuralları

- Askıda yatış/yunuslama salınımı ±10°'yi aşarsa, vane'ler 2 s'den uzun doyumdaysa ya da "Attitude failure" mesajı gelirse: hemen iniş ya da paraşüt.
- Herhangi bir pusula/yön tutarsızlığı uyarısında geçiş yapılmaz.
- Rüzgâr aşamadaki sınırı aşarsa uçuş ertelenir; kalkış ve iniş yerde en riskli andır (şehir simülasyonunda 3,5 m/s üstünde 32/50 kalkış devrilmesi).
- Bir uçuşta beklenmeyen bir davranış görülürse, simülasyonda yeniden üretilip açıklanmadan aynı aşamaya dönülür, ileri gidilmez.

## Güvenlik ve mevzuat

12,7 kg'lık, 6 kW'lık ve 10.000 rpm'lik pervaneli bir araç ciddi yaralanma riski taşır; testler yerleşim yerinden uzak, izinli bir sahada ve en az iki kişiyle yapılır. Aşağıdaki mevzuat bilgileri hafızadandır ve bu çalışmada kaynaktan doğrulanmadı; başvurudan önce SHGM'nin güncel İHA talimatı kontrol edilmelidir.

### Mevzuat (Türkiye)

- SHGM İHA talimatında 4–25 kg arası araçlar İHA1 sınıfındadır: araç kaydı, İHA1 pilot belgesi ve her uçuş için hava sahası izni gerekir.
- Telsiz frekansları (kumanda, telemetri, video) BTK kurallarına uygun olmalıdır; 915 MHz Türkiye'de kullanılmaz, 868 ya da 433 MHz sürüm alınır.
- Araç sivil araştırma aracı olarak tanımlanır; silah ya da askeri faydalı yük entegrasyonu bu rehberin kapsamı dışındadır ve ayrıca ihracat kontrolü ile savunma mevzuatına tabidir.
- Sigorta: üçüncü şahıs sorumluluk sigortası test başlamadan yapılır.

### Test sahası

- Açık, düz, en az 500 × 500 m engelsiz alan; seyir testleri için 150 m bekleme yarıçapı + geofence payı ile \~1 km çap. Proje haritasındaki varsayılan 10 × 10 km açık alan henüz seçilmiş bir saha değil.
- Pervane ve paraşüt düşüş alanında kimse bulunmaz; ekip araçtan en az 15 m uzakta, kanal ekseninin dışında durur.

### LiPo güvenliği

- 710 Wh'lik iki paket: şarj yalnız LiPo çantasında ya da metal kutuda, gözetim altında, yanıcı olmayan zeminde.
- Darbe almış, şişmiş ya da 60 °C'yi aşmış paket kullanılmaz; kaza sonrası paket 15 dk kum kovasında gözlenir.
- Depolama gerilimi hücre başına \~3,8 V; taşımada soketler ayrı.

### Atölye

- Karbon kesme ve zımparada P3 maske, gözlük ve emiş; epoksi için nitril eldiven ve havalandırma.
- Pervane takılı testlerde araç sabitlenir ve güvenlik anahtarı bir kişide durur.

## Açık sorular

- [ ] Paraşüt mu, motor yedekliği mi? Rehber P1 için paraşütü varsayıyor; karar senin.
- [ ] İlk uçuşlarda gimbal faydalı yükü çıkarılıp yerine paraşüt konabilir mi (kütle payı sıfır)?
- [ ] Pervane: kırpılmış hazır pervaneyle mi başlanacak, özel CNC/karbon fan mı yaptırılacak?
- [ ] İtki standı hazır mı alınacak, yük hücresiyle mi yapılacak?
- [ ] Test sahası ve SHGM başvurusu: hangi il ve hangi tarihte?
- [ ] Simülasyon bozulmasının nedeni ve bozuk dönemde kalan 467 geçişin temiz tekrarı (ilk serbest uçuş kapısı buna bağlı).
- [ ] SITL'de iniş ayağı yarıçapını 0,45 m'den büyütmenin kalkış devrilmesine etkisi.
- [ ] Pusula: EKF2\_MAG\_TYPE ve GNSS yönü seçeneklerinin SITL'de karşılaştırılması.
