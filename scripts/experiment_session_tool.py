#!/usr/bin/env python3
"""
DG202611 experiment session tool.

Creates experiment directory structure and metadata for organized data collection.
"""

import sys
import json
import os
import subprocess
from pathlib import Path
from datetime import datetime
from typing import Optional


def get_git_info() -> tuple[Optional[str], Optional[str]]:
    """Get current git commit and branch."""
    try:
        commit = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'],
            stderr=subprocess.DEVNULL
        ).decode().strip()

        branch = subprocess.check_output(
            ['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
            stderr=subprocess.DEVNULL
        ).decode().strip()

        return commit, branch
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None, None


def create_experiment_session(
    base_dir: str,
    scenario: str,
    operator: Optional[str] = None,
    hardware_notes: Optional[str] = None
) -> str:
    """Create experiment session directory structure.

    Args:
        base_dir: Base experiments directory
        scenario: Scenario name (e.g., "nominal", "gnss_degradation")
        operator: Optional operator name
        hardware_notes: Optional hardware configuration notes

    Returns:
        Path to created session directory
    """
    # Create timestamp-based session ID
    timestamp = datetime.now()
    session_id = timestamp.strftime("%Y%m%d_%H%M%S")
    session_name = f"{session_id}_{scenario}"

    # Create directory structure
    base_path = Path(base_dir)
    session_path = base_path / session_name

    (session_path / "raw").mkdir(parents=True, exist_ok=True)
    (session_path / "csv").mkdir(parents=True, exist_ok=True)
    (session_path / "logs").mkdir(parents=True, exist_ok=True)
    (session_path / "bags").mkdir(parents=True, exist_ok=True)

    # Get git info
    git_commit, git_branch = get_git_info()

    # Create metadata
    metadata = {
        "experiment_id": session_id,
        "scenario": scenario,
        "start_time": timestamp.isoformat(),
        "git_commit": git_commit,
        "git_branch": git_branch,
        "operator": operator,
        "hardware_notes": hardware_notes
    }

    # Write metadata file
    metadata_path = session_path / "metadata.json"
    with open(metadata_path, 'w') as f:
        json.dump(metadata, f, indent=2)

    # Create README
    readme_path = session_path / "README.txt"
    with open(readme_path, 'w') as f:
        f.write(f"DG202611 Experiment Session\n")
        f.write(f"==========================\n\n")
        f.write(f"Session ID: {session_id}\n")
        f.write(f"Scenario: {scenario}\n")
        f.write(f"Created: {timestamp.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write(f"Directory Structure:\n")
        f.write(f"  raw/  - Raw sensor data\n")
        f.write(f"  csv/  - Processed CSV logs\n")
        f.write(f"  logs/ - System logs\n")
        f.write(f"  bags/ - ROS2 bag files\n\n")
        f.write(f"See metadata.json for full session details.\n")

    return str(session_path)


def main():
    """Main entry point."""
    if len(sys.argv) < 3:
        print("Usage: experiment_session_tool.py <base_dir> <scenario> [operator] [hardware_notes]", file=sys.stderr)
        print("", file=sys.stderr)
        print("Example: experiment_session_tool.py ./experiments nominal \"Alice\" \"UWB anchors at A0-A2\"", file=sys.stderr)
        return 1

    base_dir = sys.argv[1]
    scenario = sys.argv[2]
    operator = sys.argv[3] if len(sys.argv) > 3 else None
    hardware_notes = sys.argv[4] if len(sys.argv) > 4 else None

    try:
        session_path = create_experiment_session(base_dir, scenario, operator, hardware_notes)
        print(f"Created experiment session: {session_path}")

        # Print directory tree
        print("\nDirectory structure:")
        for root, dirs, files in os.walk(session_path):
            level = root.replace(session_path, '').count(os.sep)
            indent = ' ' * 2 * level
            print(f"{indent}{os.path.basename(root)}/")
            sub_indent = ' ' * 2 * (level + 1)
            for file in files:
                print(f"{sub_indent}{file}")

        return 0

    except Exception as e:
        print(f"Error creating session: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
