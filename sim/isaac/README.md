# Isaac Lab ortamı (plan)

Gazebo eklentisinin PyTorch ile toplu (batched) eşi: aynı `models/aero/p1/`
tablolarını okur, binlerce aracı GPU'da paralel koşturur, PPO (rsl_rl veya
skrl) ile eğitilir. İlk görev rüzgârda askı; ardından geçiş ve iniş
(simülasyon dokümanı, "RL eğitimi" bölümü).

Ön koşul: NVIDIA RTX GPU, en az 16 GB VRAM. Isaac Sim kurulumu ağır olduğu
için onay beklemektedir.
