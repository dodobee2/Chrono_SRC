from __future__ import annotations

from dataclasses import asdict, dataclass
import importlib
import importlib.util
import sys
from typing import Any


@dataclass(frozen=True)
class ChronoAvailability:
    python_executable: str
    python_version: str
    pychrono_available: bool
    vehicle_module_available: bool
    pychrono_module_path: str | None
    version: str | None
    diagnostic_message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def get_pychrono_availability() -> ChronoAvailability:
    pychrono_spec = importlib.util.find_spec("pychrono")
    if pychrono_spec is None:
        return ChronoAvailability(
            python_executable=sys.executable,
            python_version=sys.version,
            pychrono_available=False,
            vehicle_module_available=False,
            pychrono_module_path=None,
            version=None,
            diagnostic_message="pychrono is not importable in the active Python environment.",
        )

    module_path = pychrono_spec.origin
    version = None
    vehicle_available = False
    messages: list[str] = []
    try:
        pychrono = importlib.import_module("pychrono")
        module_path = getattr(pychrono, "__file__", module_path)
        version = detect_pychrono_version(pychrono)
    except Exception as exc:  # pragma: no cover - depends on local installation
        messages.append(f"pychrono import failed: {type(exc).__name__}: {exc}")
        return ChronoAvailability(
            python_executable=sys.executable,
            python_version=sys.version,
            pychrono_available=False,
            vehicle_module_available=False,
            pychrono_module_path=module_path,
            version=version,
            diagnostic_message="; ".join(messages),
        )

    try:
        importlib.import_module("pychrono.vehicle")
        vehicle_available = True
    except Exception as exc:  # pragma: no cover - depends on local installation
        messages.append(f"pychrono.vehicle import failed: {type(exc).__name__}: {exc}")

    if not messages:
        messages.append("pychrono import succeeded.")

    return ChronoAvailability(
        python_executable=sys.executable,
        python_version=sys.version,
        pychrono_available=True,
        vehicle_module_available=vehicle_available,
        pychrono_module_path=module_path,
        version=version,
        diagnostic_message="; ".join(messages),
    )


def detect_pychrono_version(pychrono: Any) -> str | None:
    for attr in ("__version__", "CHRONO_VERSION", "chrono_version"):
        value = getattr(pychrono, attr, None)
        if value:
            return str(value)
    for fn_name in ("GetChronoVersion", "GetVersion"):
        fn = getattr(pychrono, fn_name, None)
        if callable(fn):
            try:
                return str(fn())
            except Exception:
                continue
    return None

