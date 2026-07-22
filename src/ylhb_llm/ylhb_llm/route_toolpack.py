from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from ylhb_mobile_bridge.patrol_route_store import load_route_file, resolve_route_file_path


def _key(value: str) -> str:
    return re.sub(r"\s+", "", str(value or "")).lower()


@dataclass(frozen=True)
class RouteCatalog:
    data: Dict[str, Any]
    route_file_path: str = ''

    @classmethod
    def from_file(cls, path: str, route_directory: str | Path | None = None) -> "RouteCatalog":
        resolved = (
            resolve_route_file_path(path)
            if route_directory is None
            else resolve_route_file_path(path, route_directory)
        )
        return cls(load_route_file(str(resolved)), str(resolved))

    @property
    def route_ids(self) -> List[str]:
        return [route["id"] for route in self.data.get("routes", [])]

    @property
    def target_ids(self) -> List[str]:
        return [target["id"] for target in self.data.get("targets", [])]

    def route(self, route_id_or_alias: str) -> Dict[str, Any]:
        match = self._lookup(route_id_or_alias, self.data.get("routes", []))
        if match is None:
            raise ValueError(f"unknown route: {route_id_or_alias}")
        return match

    def target(self, target_id_or_alias: str) -> Dict[str, Any]:
        match = self._lookup(target_id_or_alias, self.data.get("targets", []))
        if match is None:
            raise ValueError(f"unknown target: {target_id_or_alias}")
        return match

    def resolve_target_id(self, target_id_or_alias: str) -> str:
        return self.target(target_id_or_alias)["id"]

    def route_targets(self, route_id_or_alias: str) -> List[Dict[str, Any]]:
        route = self.route(route_id_or_alias)
        targets = {target["id"]: target for target in self.data.get("targets", [])}
        return [targets[target_id] for target_id in route.get("target_ids", [])]

    def _lookup(
        self,
        value: str,
        items: List[Dict[str, Any]],
    ) -> Optional[Dict[str, Any]]:
        wanted = _key(value)
        for item in items:
            names = [item.get("id"), item.get("name"), *item.get("aliases", [])]
            if wanted in {_key(name) for name in names}:
                return dict(item)
        return None


class RouteToolPack:
    def __init__(self, catalog: RouteCatalog) -> None:
        self.catalog = catalog

    def tool_schemas(self) -> Dict[str, Dict[str, Any]]:
        route_enum = self.catalog.route_ids
        target_enum = self.catalog.target_ids
        return {
            "list_routes": {"properties": {}, "required": [], "additionalProperties": False},
            "describe_route": {
                "properties": {"route_id": {"type": "string", "enum": route_enum}},
                "required": ["route_id"],
                "additionalProperties": False,
            },
            "start_route": {
                "properties": {"route_id": {"type": "string", "enum": route_enum}},
                "required": ["route_id"],
                "additionalProperties": False,
            },
            "list_checkpoints": {
                "properties": {"route_id": {"type": "string", "enum": route_enum}},
                "required": ["route_id"],
                "additionalProperties": False,
            },
            "go_to_checkpoint": {
                "properties": {"target_id": {"type": "string", "enum": target_enum}},
                "required": ["target_id"],
                "additionalProperties": False,
            },
            "inspect_checkpoint": {
                "properties": {"target_id": {"type": "string", "enum": target_enum}},
                "required": ["target_id"],
                "additionalProperties": False,
            },
        }

    def list_routes(self) -> List[Dict[str, Any]]:
        return [
            {
                "id": route["id"],
                "name": route.get("name", route["id"]),
                "aliases": route.get("aliases", []),
                "description": route.get("description"),
                "target_ids": route.get("target_ids", []),
            }
            for route in self.catalog.data.get("routes", [])
        ]

    def describe_route(self, route_id: str) -> Dict[str, Any]:
        route = self.catalog.route(route_id)
        targets = self.route_targets(route["id"])
        return {
            **route,
            "targets": targets,
            "inspection_configured": any(target["inspection_items"] for target in targets),
        }

    def inspection_configured(self, route_id: str = '') -> bool:
        selected = route_id or str(self.catalog.data.get('active_route_id') or '')
        if not selected:
            return False
        try:
            return any(
                target.get('inspection_items') for target in self.catalog.route_targets(selected)
            )
        except ValueError:
            return False

    def route_targets(self, route_id: str) -> List[Dict[str, Any]]:
        return [
            {
                "id": target["id"],
                "name": target.get("name", target["id"]),
                "aliases": target.get("aliases", []),
                "area_id": target.get("area_id"),
                "pose": target.get("pose"),
                "inspection_items": target.get("inspection_items", []),
            }
            for target in self.catalog.route_targets(route_id)
        ]

    def inspect_checkpoint(self, target_id: str) -> Dict[str, Any]:
        target = self.catalog.target(target_id)
        return {
            "id": target["id"],
            "name": target.get("name", target["id"]),
            "pose": target.get("pose"),
            "inspection_items": target.get("inspection_items", []),
        }
