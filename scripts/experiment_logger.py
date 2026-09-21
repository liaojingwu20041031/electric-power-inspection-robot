#!/usr/bin/env python3
"""
DG202611 experiment logger.

Unified logging tool for capturing experiment data with consistent timestamps.
Supports CSV and JSONL output formats.
"""

import sys
import json
import csv
import time
from pathlib import Path
from datetime import datetime
from typing import Optional, Dict, Any, List


class ExperimentLogger:
    """Experiment data logger with multiple output formats."""

    def __init__(self, output_dir: str, experiment_id: str, format: str = "csv"):
        """Initialize logger.

        Args:
            output_dir: Directory for output files
            experiment_id: Unique experiment identifier
            format: "csv" or "jsonl"
        """
        self.output_dir = Path(output_dir)
        self.experiment_id = experiment_id
        self.format = format
        self.start_time = time.time()

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Open log files
        self.log_files = {}
        self.csv_writers = {}

        # Define log streams
        self.streams = [
            "uwb",
            "gnss",
            "lidar_diagnostics",
            "model1_state",
            "model2_decision",
            "model3_state",
            "relocalization",
            "fusion_output"
        ]

    def get_timestamp(self) -> float:
        """Get unified timestamp (seconds since logger start)."""
        return time.time() - self.start_time

    def _get_log_file(self, stream: str):
        """Get or create log file handle for stream."""
        if stream not in self.log_files:
            ext = "csv" if self.format == "csv" else "jsonl"
            filepath = self.output_dir / f"{self.experiment_id}_{stream}.{ext}"
            self.log_files[stream] = open(filepath, 'w', newline='')

            if self.format == "csv":
                # Will write header on first log entry
                self.csv_writers[stream] = None

        return self.log_files[stream]

    def log(self, stream: str, data: Dict[str, Any]) -> None:
        """Log data to specified stream.

        Args:
            stream: Stream name (e.g., "uwb", "gnss")
            data: Data dictionary (timestamp will be added if not present)
        """
        if stream not in self.streams:
            print(f"Warning: unknown stream '{stream}'", file=sys.stderr)

        # Add timestamp if not present
        if 'timestamp' not in data:
            data['timestamp'] = self.get_timestamp()

        logfile = self._get_log_file(stream)

        if self.format == "csv":
            self._write_csv(stream, logfile, data)
        else:
            self._write_jsonl(logfile, data)

    def _write_csv(self, stream: str, file, data: Dict[str, Any]) -> None:
        """Write CSV entry."""
        if self.csv_writers[stream] is None:
            # Write header on first entry
            fieldnames = list(data.keys())
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            self.csv_writers[stream] = writer

        self.csv_writers[stream].writerow(data)
        file.flush()

    def _write_jsonl(self, file, data: Dict[str, Any]) -> None:
        """Write JSONL entry."""
        json.dump(data, file)
        file.write('\n')
        file.flush()

    def close(self) -> None:
        """Close all log files."""
        for f in self.log_files.values():
            f.close()


def main():
    """Demo/test main."""
    if len(sys.argv) < 3:
        print("Usage: experiment_logger.py <output_dir> <experiment_id> [format]", file=sys.stderr)
        print("format: csv (default) or jsonl", file=sys.stderr)
        return 1

    output_dir = sys.argv[1]
    experiment_id = sys.argv[2]
    format = sys.argv[3] if len(sys.argv) > 3 else "csv"

    logger = ExperimentLogger(output_dir, experiment_id, format)

    # Demo logging
    logger.log("uwb", {
        "x": 10.5,
        "y": 20.3,
        "confidence": 0.85,
        "state": "GOOD"
    })

    logger.log("gnss", {
        "latitude": 30.0,
        "longitude": 120.0,
        "hdop": 1.2,
        "satellites": 12
    })

    logger.log("fusion_output", {
        "x": 10.4,
        "y": 20.2,
        "mode": "FUSED",
        "confidence": 0.9
    })

    logger.close()

    print(f"Demo logs written to {output_dir}/{experiment_id}_*.{format}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
