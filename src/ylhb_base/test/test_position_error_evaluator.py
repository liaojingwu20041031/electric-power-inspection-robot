#!/usr/bin/env python3
"""Unit tests for position_error_evaluator.py"""

import sys
import os
import tempfile
import csv
import unittest
from pathlib import Path

# Add scripts to path
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from position_error_evaluator import (
    PositionSample,
    compute_error,
    compute_metrics,
    compute_vertical_error,
    compute_vertical_metrics,
    read_csv,
)


class TestPositionErrorEvaluator(unittest.TestCase):
    """Test position error calculations."""

    def test_compute_error_zero(self):
        """Test zero error when estimated equals ground truth."""
        sample = PositionSample(0.0, 10.0, 20.0, 10.0, 20.0)
        error = compute_error(sample)
        self.assertAlmostEqual(error, 0.0, places=6)

    def test_compute_error_pythagorean(self):
        """Test 3-4-5 triangle."""
        sample = PositionSample(0.0, 0.0, 0.0, 3.0, 4.0)
        error = compute_error(sample)
        self.assertAlmostEqual(error, 5.0, places=6)

    def test_compute_error_unit(self):
        """Test 1m error in X direction."""
        sample = PositionSample(0.0, 10.0, 20.0, 11.0, 20.0)
        error = compute_error(sample)
        self.assertAlmostEqual(error, 1.0, places=6)

    def test_compute_metrics_single_sample(self):
        """Test metrics with single sample."""
        samples = [PositionSample(0.0, 0.0, 0.0, 1.0, 0.0)]
        metrics = compute_metrics(samples)

        self.assertIsNotNone(metrics)
        self.assertEqual(metrics.count, 1)
        self.assertAlmostEqual(metrics.mean, 1.0, places=6)
        self.assertAlmostEqual(metrics.median, 1.0, places=6)
        self.assertAlmostEqual(metrics.rmse, 1.0, places=6)
        self.assertAlmostEqual(metrics.p95, 1.0, places=6)
        self.assertAlmostEqual(metrics.max_error, 1.0, places=6)
        self.assertAlmostEqual(metrics.min_error, 1.0, places=6)

    def test_compute_metrics_multiple_samples(self):
        """Test metrics with multiple samples."""
        samples = [
            PositionSample(0.0, 0.0, 0.0, 1.0, 0.0),  # error = 1.0
            PositionSample(1.0, 0.0, 0.0, 2.0, 0.0),  # error = 2.0
            PositionSample(2.0, 0.0, 0.0, 3.0, 0.0),  # error = 3.0
        ]
        metrics = compute_metrics(samples)

        self.assertIsNotNone(metrics)
        self.assertEqual(metrics.count, 3)
        self.assertAlmostEqual(metrics.mean, 2.0, places=6)
        self.assertAlmostEqual(metrics.median, 2.0, places=6)
        self.assertAlmostEqual(metrics.max_error, 3.0, places=6)
        self.assertAlmostEqual(metrics.min_error, 1.0, places=6)

    def test_compute_metrics_rmse(self):
        """Test RMSE calculation."""
        samples = [
            PositionSample(0.0, 0.0, 0.0, 3.0, 0.0),  # error = 3.0
            PositionSample(1.0, 0.0, 0.0, 4.0, 0.0),  # error = 4.0
        ]
        metrics = compute_metrics(samples)

        # RMSE = sqrt((3^2 + 4^2) / 2) = sqrt(12.5) = 3.535...
        expected_rmse = (9.0 + 16.0) / 2.0
        expected_rmse = expected_rmse ** 0.5
        self.assertAlmostEqual(metrics.rmse, expected_rmse, places=6)

    def test_compute_metrics_empty(self):
        """Test metrics with no samples."""
        metrics = compute_metrics([])
        self.assertIsNone(metrics)

    def test_vertical_metrics_use_only_rows_with_independent_z_truth(self):
        samples = [
            PositionSample(0.0, 0.0, 0.0, 0.0, 0.0, 1.10, 1.00),
            PositionSample(1.0, 0.0, 0.0, 0.0, 0.0, 1.30, 1.00),
            PositionSample(2.0, 0.0, 0.0, 0.0, 0.0),
        ]
        self.assertAlmostEqual(compute_vertical_error(samples[0]), 0.10, places=6)
        metrics = compute_vertical_metrics(samples)
        self.assertIsNotNone(metrics)
        self.assertEqual(metrics.count, 2)
        self.assertAlmostEqual(metrics.mean, 0.20, places=6)

    def test_read_csv_valid(self):
        """Test reading valid CSV file."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'estimated_x', 'estimated_y',
                           'ground_truth_x', 'ground_truth_y'])
            writer.writerow([0.0, 10.0, 20.0, 11.0, 21.0])
            writer.writerow([1.0, 12.0, 22.0, 13.0, 23.0])
            temp_path = f.name

        try:
            samples = read_csv(temp_path)
            self.assertEqual(len(samples), 2)
            self.assertAlmostEqual(samples[0].timestamp, 0.0, places=6)
            self.assertAlmostEqual(samples[0].estimated_x, 10.0, places=6)
            self.assertAlmostEqual(samples[1].timestamp, 1.0, places=6)
        finally:
            os.unlink(temp_path)

    def test_read_csv_invalid_rows_skipped(self):
        """Test that invalid rows are skipped."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'estimated_x', 'estimated_y',
                           'ground_truth_x', 'ground_truth_y'])
            writer.writerow([0.0, 10.0, 20.0, 11.0, 21.0])  # valid
            writer.writerow(['bad', 'data', 'here', 'invalid', 'row'])  # invalid
            writer.writerow([1.0, 12.0, 22.0, 13.0, 23.0])  # valid
            temp_path = f.name

        try:
            samples = read_csv(temp_path)
            self.assertEqual(len(samples), 2)  # Only valid rows
        finally:
            os.unlink(temp_path)

    def test_read_csv_accepts_optional_z_columns(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'estimated_x', 'estimated_y', 'estimated_z',
                'ground_truth_x', 'ground_truth_y', 'ground_truth_z'
            ])
            writer.writerow([0.0, 1.0, 2.0, 0.25, 1.1, 2.1, 0.20])
            temp_path = f.name
        try:
            samples = read_csv(temp_path)
            self.assertEqual(len(samples), 1)
            self.assertAlmostEqual(samples[0].estimated_z, 0.25, places=6)
            self.assertAlmostEqual(samples[0].ground_truth_z, 0.20, places=6)
        finally:
            os.unlink(temp_path)


if __name__ == '__main__':
    unittest.main()
