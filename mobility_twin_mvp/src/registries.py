from __future__ import annotations

from pathlib import Path
from typing import Generic, TypeVar

import yaml

from .integration_schemas import ControlProfile, RoverSpec, TerrainScenario


T = TypeVar("T")


class YamlRegistry(Generic[T]):
    def __init__(self, root: Path, schema_type: type[T]) -> None:
        self.root = Path(root)
        self.schema_type = schema_type

    def ids(self) -> list[str]:
        return sorted(self._paths().keys())

    def load(self, item_id: str) -> T:
        paths = self._paths()
        if item_id not in paths:
            raise KeyError(f"{item_id} not found in {self.root}")
        with paths[item_id].open("r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        return self.schema_type.from_mapping(payload)  # type: ignore[attr-defined]

    def load_all(self) -> dict[str, T]:
        return {item_id: self.load(item_id) for item_id in self.ids()}

    def _paths(self) -> dict[str, Path]:
        raise NotImplementedError


class RoverRegistry(YamlRegistry[RoverSpec]):
    """Loads rover_models/<rover_id>/rover.yaml."""

    def __init__(self, root: Path) -> None:
        super().__init__(root, RoverSpec)

    def _paths(self) -> dict[str, Path]:
        return {
            path.parent.name: path
            for path in self.root.glob("*/rover.yaml")
            if path.is_file()
        }


class TerrainRegistry(YamlRegistry[TerrainScenario]):
    """Loads terrain_scenarios/<terrain_id>/terrain.yaml."""

    def __init__(self, root: Path) -> None:
        super().__init__(root, TerrainScenario)

    def _paths(self) -> dict[str, Path]:
        return {
            path.parent.name: path
            for path in self.root.glob("*/terrain.yaml")
            if path.is_file()
        }


class ControlProfileRegistry(YamlRegistry[ControlProfile]):
    """Loads control_profiles/<profile_id>.yaml."""

    def __init__(self, root: Path) -> None:
        super().__init__(root, ControlProfile)

    def _paths(self) -> dict[str, Path]:
        return {
            path.stem: path
            for path in self.root.glob("*.yaml")
            if path.is_file()
        }

