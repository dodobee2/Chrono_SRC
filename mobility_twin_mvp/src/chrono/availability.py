from __future__ import annotations

from dataclasses import asdict, dataclass
import importlib.metadata
import importlib.util
import sys
from typing import Any


@dataclass(frozen=True)
class ChronoAvailability:
    python_executable: str
    python_version: str
    pychrono_available: bool
    vehicle_module_available: bool
    irrlicht_module_available: bool
    pychrono_module_path: str | None
    version: str | None
    diagnostic_message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def get_pychrono_availability() -> ChronoAvailability:
    """Return import-safe PyChrono availability diagnostics.

    This function intentionally avoids importing pychrono. On this project, real
    Chrono execution is isolated in a worker process so a blocked native import
    cannot freeze Streamlit or ordinary unit tests.
    """
    pychrono_spec = importlib.util.find_spec("pychrono")
    if pychrono_spec is None:
        return ChronoAvailability(
            python_executable=sys.executable,
            python_version=sys.version,
            pychrono_available=False,
            vehicle_module_available=False,
            irrlicht_module_available=False,
            pychrono_module_path=None,
            version=None,
            diagnostic_message="pychrono is not importable in the active Python environment.",
        )

    module_path = pychrono_spec.origin
    vehicle_available = importlib.util.find_spec("pychrono.vehicle") is not None
    irrlicht_available = importlib.util.find_spec("pychrono.irrlicht") is not None
    return ChronoAvailability(
        python_executable=sys.executable,
        python_version=sys.version,
        pychrono_available=True,
        vehicle_module_available=vehicle_available,
        irrlicht_module_available=irrlicht_available,
        pychrono_module_path=module_path,
        version=_detect_distribution_version(),
        diagnostic_message=(
            "pychrono module spec found without importing PyChrono. "
            "Smoke execution is isolated in a worker process to avoid blocking the app."
        ),
    )


def _detect_distribution_version() -> str | None:
    for package_name in ("pychrono", "chrono"):
        try:
            return importlib.metadata.version(package_name)
        except importlib.metadata.PackageNotFoundError:
            continue
    return None


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
