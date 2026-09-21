#!/usr/bin/env python3
"""
Relocalization evaluator for DG202611 experiments.

Analyzes relocalization attempts and computes success metrics.
"""

import sys
import csv
from dataclasses import dataclass
from typing import List, Dict, Optional
from collections import Counter


@dataclass
class RelocalizationEvent:
    """Single relocalization attempt."""
    timestamp: float
    attempt_id: int
    outcome: str  # RECOVERED, FAILED, MANUAL_REQUIRED
    time_to_recovery: Optional[float]  # seconds, None if not recovered
    failure_reason: Optional[str]


@dataclass
class RelocalizationMetrics:
    """Relocalization statistics."""
    total_attempts: int
    successes: int
    failures: int
    manual_required: int
    success_rate: float
    mean_time_to_recovery: Optional[float]
    median_time_to_recovery: Optional[float]
    failure_reasons: Dict[str, int]


def compute_metrics(events: List[RelocalizationEvent]) -> Optional[RelocalizationMetrics]:
    """Compute relocalization metrics from events."""
    if not events:
        return None

    total_attempts = len(events)
    successes = sum(1 for e in events if e.outcome == "RECOVERED")
    failures = sum(1 for e in events if e.outcome == "FAILED")
    manual_required = sum(1 for e in events if e.outcome == "MANUAL_REQUIRED")

    success_rate = successes / total_attempts if total_attempts > 0 else 0.0

    # Time to recovery statistics (only for successful recoveries)
    recovery_times = [e.time_to_recovery for e in events
                     if e.outcome == "RECOVERED" and e.time_to_recovery is not None]

    if recovery_times:
        mean_time = sum(recovery_times) / len(recovery_times)
        recovery_times_sorted = sorted(recovery_times)
        median_time = recovery_times_sorted[len(recovery_times_sorted) // 2]
    else:
        mean_time = None
        median_time = None

    # Count failure reasons
    failure_reasons_list = [e.failure_reason for e in events
                           if e.outcome in ("FAILED", "MANUAL_REQUIRED")
                           and e.failure_reason is not None]
    failure_reasons = dict(Counter(failure_reasons_list))

    return RelocalizationMetrics(
        total_attempts=total_attempts,
        successes=successes,
        failures=failures,
        manual_required=manual_required,
        success_rate=success_rate,
        mean_time_to_recovery=mean_time,
        median_time_to_recovery=median_time,
        failure_reasons=failure_reasons
    )


def read_csv(filepath: str) -> List[RelocalizationEvent]:
    """Read relocalization events from CSV file.

    Expected columns: timestamp, attempt_id, outcome, time_to_recovery, failure_reason
    """
    events = []

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                outcome = row['outcome'].strip()
                if outcome not in ("RECOVERED", "FAILED", "MANUAL_REQUIRED"):
                    print(f"Warning: invalid outcome '{outcome}', skipping", file=sys.stderr)
                    continue

                time_to_recovery_str = row.get('time_to_recovery', '').strip()
                if time_to_recovery_str and time_to_recovery_str != 'None':
                    time_to_recovery = float(time_to_recovery_str)
                else:
                    time_to_recovery = None

                failure_reason = row.get('failure_reason', '').strip()
                if not failure_reason or failure_reason == 'None':
                    failure_reason = None

                event = RelocalizationEvent(
                    timestamp=float(row['timestamp']),
                    attempt_id=int(row['attempt_id']),
                    outcome=outcome,
                    time_to_recovery=time_to_recovery,
                    failure_reason=failure_reason
                )
                events.append(event)
            except (KeyError, ValueError) as e:
                print(f"Warning: skipping invalid row: {e}", file=sys.stderr)
                continue

    return events


def print_metrics(metrics: RelocalizationMetrics) -> None:
    """Print metrics in human-readable format."""
    print("Relocalization Metrics:")
    print(f"  Total Attempts:       {metrics.total_attempts}")
    print(f"  Successes (RECOVERED): {metrics.successes}")
    print(f"  Failures:             {metrics.failures}")
    print(f"  Manual Required:      {metrics.manual_required}")
    print(f"  Success Rate:         {metrics.success_rate:.2%}")

    if metrics.mean_time_to_recovery is not None:
        print(f"  Mean Time to Recovery: {metrics.mean_time_to_recovery:.2f} s")
    if metrics.median_time_to_recovery is not None:
        print(f"  Median Time to Recovery: {metrics.median_time_to_recovery:.2f} s")

    if metrics.failure_reasons:
        print("  Failure Reasons:")
        for reason, count in sorted(metrics.failure_reasons.items(), key=lambda x: -x[1]):
            print(f"    {reason}: {count}")


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: relocalization_evaluator.py <csv_file>", file=sys.stderr)
        print("", file=sys.stderr)
        print("CSV format: timestamp,attempt_id,outcome,time_to_recovery,failure_reason", file=sys.stderr)
        print("outcome: RECOVERED | FAILED | MANUAL_REQUIRED", file=sys.stderr)
        return 1

    filepath = sys.argv[1]

    try:
        events = read_csv(filepath)
    except FileNotFoundError:
        print(f"Error: file not found: {filepath}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error reading file: {e}", file=sys.stderr)
        return 1

    if not events:
        print("Error: no valid events found", file=sys.stderr)
        return 1

    metrics = compute_metrics(events)
    if not metrics:
        print("Error: failed to compute metrics", file=sys.stderr)
        return 1

    print_metrics(metrics)
    return 0


if __name__ == "__main__":
    sys.exit(main())
