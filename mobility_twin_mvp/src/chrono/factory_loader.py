"""Dynamic loader for handoff-provided Chrono factories.

Factory URIs are repo-relative file paths plus a callable name:

    terrain_scenarios/jongmin_arena_v01/chrono_factory.py:build_terrain

or source-file-relative paths from a YAML field:

    chrono_factory.py:build_terrain

The callable is loaded only when a PyChrono backend actually builds terrain;
plain registry loading and Streamlit import remain safe without PyChrono.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable
import importlib.util


FactoryCallable = Callable[..., Any]


def load_factory_callable(factory_uri: str, *, source_path: Path, repo_root: Path) -> FactoryCallable:
    if not factory_uri:
        raise ValueError("factory_uri is required")
    if ":" not in factory_uri:
        raise ValueError("factory_uri must use '<path.py>:<callable>' format")

    module_part, callable_name = factory_uri.split(":", 1)
    if not module_part.endswith(".py"):
        raise ValueError("factory_uri path must point to a .py file")
    if not callable_name:
        raise ValueError("factory_uri callable name is required")

    raw_path = Path(module_part)
    candidates = []
    if raw_path.is_absolute():
        candidates.append(raw_path)
    else:
        candidates.append((source_path.parent / raw_path).resolve())
        candidates.append((repo_root / raw_path).resolve())

    module_path = next((candidate for candidate in candidates if candidate.exists()), None)
    if module_path is None:
        tried = ", ".join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(f"factory module not found for {factory_uri!r}; tried {tried}")

    module_name = "mobility_handoff_factory_" + str(abs(hash(module_path)))
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot create import spec for {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    target: Any = module
    for part in callable_name.split("."):
        target = getattr(target, part)
    if not callable(target):
        raise TypeError(f"factory target is not callable: {factory_uri}")
    return target