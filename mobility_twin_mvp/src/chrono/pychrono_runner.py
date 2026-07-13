from __future__ import annotations

import argparse
import json
from pathlib import Path

from .smoke_scenario import SmokeScenarioConfig, run_smoke_scenario


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a PyChrono smoke scenario.")
    parser.add_argument("--scenario", type=Path, required=False)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    config = SmokeScenarioConfig()
    if args.scenario:
        payload = json.loads(args.scenario.read_text(encoding="utf-8"))
        config = SmokeScenarioConfig(**payload)

    result = run_smoke_scenario(config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")
    return 0 if result.status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

