from __future__ import annotations

from pathlib import Path
from typing import Any, Generic, TypeVar
import warnings

import yaml

from .integration_schemas import (
    ContactPairSpec,
    ControlProfile,
    RoverSpec,
    ScoutObservation,
    TerrainMaterialSpec,
    TerrainScenario,
)


T = TypeVar("T")


class YamlRegistry(Generic[T]):
    def __init__(self, root: Path, schema_type: type[T], repo_root: Path | None = None) -> None:
        self.root = Path(root)
        self.schema_type = schema_type
        self.repo_root = Path(repo_root) if repo_root else self.root.parent

    def ids(self) -> list[str]:
        return sorted(item_id for item_id in self._paths().keys() if not item_id.startswith("_"))

    def load(self, item_id: str) -> T:
        paths = self._paths()
        if item_id not in paths:
            raise KeyError(f"{item_id} not found in {self.root}")
        payload = self._load_yaml(paths[item_id])
        item = self.schema_type.from_mapping(payload)  # type: ignore[attr-defined]
        self._post_load_validate(item, paths[item_id])
        return item

    def load_all(self) -> dict[str, T]:
        return {item_id: self.load(item_id) for item_id in self.ids()}

    def _load_yaml(self, path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        if not isinstance(payload, dict):
            raise ValueError(f"{path} must contain a YAML mapping")
        return payload

    def _paths(self) -> dict[str, Path]:
        raise NotImplementedError

    def _post_load_validate(self, item: T, source_path: Path) -> None:
        return None

    def _validate_relative_asset(self, uri: str | None, source_path: Path, label: str) -> None:
        if not uri:
            return
        if "://" in uri:
            warnings.warn(f"{label} uses placeholder/non-file URI: {uri}", UserWarning, stacklevel=2)
            return
        path_part = uri
        if label.endswith("factory_uri") and ":" in uri:
            path_part = uri.split(":", 1)[0]
        candidate = (source_path.parent / path_part).resolve()
        repo_candidate = (self.repo_root / path_part).resolve()
        if candidate.exists() or repo_candidate.exists():
            return
        warnings.warn(f"{label} asset is not present yet: {uri}", UserWarning, stacklevel=2)


class RoverRegistry(YamlRegistry[RoverSpec]):
    """Loads rover_models/<rover_id>/rover.yaml."""

    def __init__(self, root: Path, repo_root: Path | None = None) -> None:
        super().__init__(root, RoverSpec, repo_root)

    def _paths(self) -> dict[str, Path]:
        return {
            path.parent.name: path
            for path in self.root.glob("*/rover.yaml")
            if path.is_file()
        }

    def _post_load_validate(self, item: RoverSpec, source_path: Path) -> None:
        self._validate_relative_asset(item.model_uri, source_path, "RoverSpec.model_uri")


class TerrainRegistry(YamlRegistry[TerrainScenario]):
    """Loads terrain_scenarios/<terrain_id>/terrain.yaml."""

    def __init__(self, root: Path, repo_root: Path | None = None) -> None:
        super().__init__(root, TerrainScenario, repo_root)

    def _paths(self) -> dict[str, Path]:
        return {
            path.parent.name: path
            for path in self.root.glob("*/terrain.yaml")
            if path.is_file()
        }

    def _post_load_validate(self, item: TerrainScenario, source_path: Path) -> None:
        self._validate_relative_asset(item.geometry.asset_uri, source_path, "TerrainGeometrySpec.asset_uri")
        self._validate_relative_asset(item.geometry.factory_uri, source_path, "TerrainGeometrySpec.factory_uri")
        for obstacle in item.obstacles:
            self._validate_relative_asset(obstacle.collision_uri, source_path, f"{obstacle.obstacle_id}.collision_uri")
            self._validate_relative_asset(obstacle.visual_uri, source_path, f"{obstacle.obstacle_id}.visual_uri")


class ControlProfileRegistry(YamlRegistry[ControlProfile]):
    """Loads control_profiles/<profile_id>.yaml."""

    def __init__(self, root: Path, repo_root: Path | None = None) -> None:
        super().__init__(root, ControlProfile, repo_root)

    def _paths(self) -> dict[str, Path]:
        return {
            path.stem: path
            for path in self.root.glob("*.yaml")
            if path.is_file()
        }


class ObservationRegistry(YamlRegistry[ScoutObservation]):
    """Loads observations/<observation_id>/observation.yaml."""

    def __init__(self, root: Path, repo_root: Path | None = None) -> None:
        super().__init__(root, ScoutObservation, repo_root)

    def _paths(self) -> dict[str, Path]:
        return {
            path.parent.name: path
            for path in self.root.glob("*/observation.yaml")
            if path.is_file()
        }

    def ids_for(self, terrain_id: str | None = None, control_profile_id: str | None = None) -> list[str]:
        matched: list[str] = []
        for observation_id in self.ids():
            observation = self.load(observation_id)
            if terrain_id and observation.terrain_id != terrain_id:
                continue
            if control_profile_id and observation.control_profile_id != control_profile_id:
                continue
            matched.append(observation_id)
        return matched


class TerrainMaterialRegistry(YamlRegistry[TerrainMaterialSpec]):
    """Loads terrain_materials/<material_id>.yaml."""

    def __init__(self, root: Path, repo_root: Path | None = None) -> None:
        super().__init__(root, TerrainMaterialSpec, repo_root)

    def _paths(self) -> dict[str, Path]:
        return {
            path.stem: path
            for path in self.root.glob("*.yaml")
            if path.is_file()
        }


class ContactPairRegistry(YamlRegistry[ContactPairSpec]):
    """Loads contact_pairs/<contact_pair_id>.yaml."""

    def __init__(self, root: Path, repo_root: Path | None = None) -> None:
        super().__init__(root, ContactPairSpec, repo_root)

    def _paths(self) -> dict[str, Path]:
        return {
            path.stem: path
            for path in self.root.glob("*.yaml")
            if path.is_file()
        }

    def ids_for(self, wheel_material_id: str, terrain_material_id: str) -> list[str]:
        matched: list[str] = []
        for contact_pair_id in self.ids():
            pair = self.load(contact_pair_id)
            if pair.wheel_material_id == wheel_material_id and pair.terrain_material_id == terrain_material_id:
                matched.append(contact_pair_id)
        return matched

    @staticmethod
    def validate_references(
        pair: ContactPairSpec,
        rover: RoverSpec,
        terrain: TerrainScenario,
        material_registry: TerrainMaterialRegistry,
    ) -> None:
        if pair.wheel_material_id != rover.wheel_material_id:
            raise ValueError("ContactPairSpec wheel_material_id does not match RoverSpec")
        if pair.terrain_material_id != terrain.material_id:
            raise ValueError("ContactPairSpec terrain_material_id does not match TerrainScenario")
        if pair.terrain_material_id not in material_registry.ids():
            raise ValueError("ContactPairSpec terrain material is not registered")

