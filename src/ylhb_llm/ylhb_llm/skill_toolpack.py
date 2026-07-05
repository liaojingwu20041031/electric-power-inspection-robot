from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml


class SkillToolPack:
    def __init__(self, capabilities: Dict[str, Any], route_schemas: Dict[str, Dict[str, Any]]) -> None:
        self.capabilities = capabilities
        self.route_schemas = route_schemas

    @classmethod
    def from_file(
        cls,
        path: str,
        route_schemas: Dict[str, Dict[str, Any]],
    ) -> "SkillToolPack":
        with Path(path).expanduser().open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        capabilities = data.get("capabilities") or {}
        if not isinstance(capabilities, dict):
            raise ValueError("capabilities must be an object")
        return cls(capabilities, route_schemas)

    def tool_schemas(self) -> Dict[str, Dict[str, Any]]:
        schemas: Dict[str, Dict[str, Any]] = {}
        for name, capability in self.capabilities.items():
            argument_schema = dict(capability.get("argument_schema") or {})
            properties = dict(argument_schema.get("properties") or {})
            route_schema = self.route_schemas.get(name, {})
            properties.update(route_schema.get("properties") or {})
            schemas[name] = {
                "properties": properties,
                "required": list(argument_schema.get("required") or route_schema.get("required") or []),
                "risk_level": capability.get("risk_level", "normal"),
                "executor": capability.get("executor", "local"),
                "constraints": capability.get("constraints") or {},
                "timeout_sec": capability.get("timeout_sec"),
            }
        return schemas
