"""Load the single-source vehicle configuration (config/<name>.toml)."""

from __future__ import annotations

import hashlib
import tomllib
from pathlib import Path

REQUIRED_SECTIONS = (
    "meta", "environment", "mass", "aero", "wing", "elevons", "tail", "body",
    "landing_gear", "duct", "rotors", "motors", "vanes", "battery", "requirements",
)


class ConfigError(ValueError):
    pass


def load(path: str | Path) -> dict:
    path = Path(path)
    raw = path.read_bytes()
    cfg = tomllib.loads(raw.decode("utf-8"))
    validate(cfg)
    # Normalise line endings so the hash is the same on Windows and Linux checkouts.
    digest = hashlib.sha256(raw.replace(b"\r\n", b"\n")).hexdigest()
    cfg["_source"] = {"file": path.name, "sha256": digest}
    return cfg


def validate(cfg: dict) -> None:
    missing = [s for s in REQUIRED_SECTIONS if s not in cfg]
    if missing:
        raise ConfigError(f"missing sections: {', '.join(missing)}")

    total = sum(c["mass_kg"] for c in cfg["mass"]["components"])
    mtow = cfg["mass"]["mtow_kg"]
    if abs(total - mtow) > 1e-6:
        raise ConfigError(f"component masses sum to {total:.3f} kg, mtow is {mtow:.3f} kg")

    for c in cfg["mass"]["components"]:
        if c["mass_kg"] <= 0 or len(c.get("size_m", [0, 0, 0])) != 3:
            raise ConfigError(f"bad mass component: {c['name']}")

    if cfg["duct"]["hub_diameter_m"] >= cfg["duct"]["inner_diameter_m"]:
        raise ConfigError("hub diameter must be smaller than duct diameter")
    if cfg["rotors"]["count"] != 2:
        raise ConfigError("the coaxial model assumes exactly 2 rotors")
    if not 0 < cfg["rotors"]["figure_of_merit"] <= 1:
        raise ConfigError("figure of merit must be in (0, 1]")
    if cfg["vanes"]["max_deflection_deg"] > cfg["vanes"]["stall_deflection_deg"]:
        raise ConfigError("vane max deflection exceeds vane stall deflection")
    for key in ("cells_series", "capacity_ah", "max_continuous_c", "energy_wh", "usable_fraction"):
        if key not in cfg["battery"]:
            raise ConfigError(f"battery.{key} missing")
    e = cfg["elevons"]
    if not 0 <= e["span_start_fraction"] < e["span_end_fraction"] <= 1:
        raise ConfigError("elevon span fractions must satisfy 0 <= start < end <= 1")
