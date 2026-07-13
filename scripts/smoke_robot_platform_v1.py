#!/usr/bin/env python3
"""Minimal offline checks for Robot Platform Protocol v1 storage."""
import tempfile
from pathlib import Path

from ylhb_mobile_bridge.platform_store import DeploymentStore


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        store = DeploymentStore(Path(temporary))
        assert store.events(0, 100) == []
        assert store.append_event({"event": "command_accepted"})["sequence"] == 1
        assert store.events(0, 100)[0]["sequence"] == 1
    print("robot platform v1 smoke: ok")


if __name__ == "__main__":
    main()
