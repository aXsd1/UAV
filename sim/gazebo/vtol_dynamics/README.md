# vtol_dynamics — Gazebo Harmonic 6-DOF eklentisi

Kanal içindeki koaksiyel pervaneleri, vane'leri ve 0–90° geçiş aerodinamiğini
hazır eklentiler modelleyemediği için araç gövdesine toplam kuvvet ve momenti bu
sistem uygular.

- `include/`, `src/model.cpp`: Gazebo'dan bağımsız kuvvet modeli;
  `tools/dynamics/model.py`'nin C++ karşılığı.
- `src/VtolDynamics.cpp`: Gazebo sistemi. PX4'ün motor
  (`/<model>/command/motor_speed`) ve servo (`/model/<model>/servo_<i>`)
  komutlarını alır, motor gecikmesi ile servo hız sınırı ve ölü bölgeyi uygular,
  kuvvet ve momenti `base_link`'e (PX4 tailsitter çerçevesi) uygular.
  Rüzgâr: `/model/<model>/wind` (gz.msgs.Vector3d, dünya ENU, m/s).
- `test/vectors.json`: Python referansından (`python -m tools.dynamics.vectors`)
  üretilen 22 vaka; `ctest` C++ çıktısının birebir aynı olduğunu doğrular.

Derleme ve kurulum WSL içinde `px4/install.sh` ile yapılır.

Henüz modellenmeyenler: yer etkisi, pervane akışının kanat ve elevonlar
üzerindeki etkisi, α–β birleşik tablolar, Dryden türbülansı.
