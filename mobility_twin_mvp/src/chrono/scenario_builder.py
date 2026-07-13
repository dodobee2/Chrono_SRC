from __future__ import annotations

from .smoke_scenario import SmokeScenarioConfig


def build_default_smoke_config() -> SmokeScenarioConfig:
    return SmokeScenarioConfig()

