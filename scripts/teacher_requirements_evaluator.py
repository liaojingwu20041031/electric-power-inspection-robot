#!/usr/bin/env python3
"""Strictly evaluate DG-202611 evidence against the teacher's four requirements.

The evaluator is deliberately fail-closed.  A numerical CSV is not enough: a
passing result also requires a complete REAL_ROBOT_CAPTURE session, a valid
manifest, a protocol frozen before capture, and the condition-specific evidence
listed below.  Missing evidence is reported as NOT_PROVEN or
INCOMPLETE_EVIDENCE, never silently converted to PASS.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Iterable, Sequence

from evaluate_real_robot_sessions import discover_sessions, read_json, verify_manifest
from feature_repeatability_evaluator import compute_metrics as compute_feature_metrics
from feature_repeatability_evaluator import read_csv as read_feature_csv
from position_error_evaluator import PositionSample, compute_metrics, compute_vertical_metrics
from relocalization_evaluator import compute_metrics as compute_relocalization_metrics
from relocalization_evaluator import read_csv as read_relocalization_csv


POSITION_REQUIREMENT = "POSITION_CONCURRENT_DEGRADATION"
FEATURE_REQUIREMENT = "DYNAMIC_FEATURE_REPEATABILITY"
RELOCALIZATION_REQUIREMENT = "RELOCALIZATION_ROBUSTNESS"
BEIDOU_REQUIREMENT = "BEIDOU_SHORT_MESSAGE_BIDIRECTIONAL"
REQUIREMENT_IDS = (
    POSITION_REQUIREMENT,
    FEATURE_REQUIREMENT,
    RELOCALIZATION_REQUIREMENT,
    BEIDOU_REQUIREMENT,
)
RESULT_STATES = {"PASS", "FAIL", "NOT_PROVEN", "INCOMPLETE_EVIDENCE"}
DEGRADED_STATES = {"DEGRADED", "REJECTED", "STALE", "FAILED"}
RELOCALIZATION_TRIGGER_CLASSES = {
    "VIOLENT_MOTION",
    "CROSS_DOMAIN",
    "PERCEPTION_DEGRADATION",
}
BEIDOU_EVENT_SEQUENCE = (
    "ROBOT_SEND_ACCEPTED",
    "REMOTE_RECEIVED",
    "REMOTE_REPLY_SENT",
    "ROBOT_REPLY_RECEIVED",
)


@dataclass
class RequirementResult:
    requirement_id: str
    title: str
    status: str
    target: str
    observed: dict[str, Any]
    accepted_sessions: list[str]
    reasons: list[str]

    def __post_init__(self) -> None:
        if self.status not in RESULT_STATES:
            raise ValueError(f"unknown result status: {self.status}")


@dataclass
class SessionEvidence:
    path: Path
    experiment_id: str
    metadata: dict[str, Any]
    result: dict[str, Any]
    protocol: dict[str, Any] | None
    manifest_valid: bool
    manifest_errors: list[str]
    manifest_paths: set[str]

    @property
    def requirement_id(self) -> str:
        return str((self.protocol or {}).get("requirement_id", ""))

    def manifest_includes(self, relative_path: str) -> bool:
        return Path(relative_path).as_posix() in self.manifest_paths

    def require_manifest_file(self, relative_path: str, errors: list[str]) -> None:
        if not self.manifest_includes(relative_path):
            errors.append(f"证据文件未纳入 SHA256 清单：{relative_path}")

    @property
    def base_errors(self) -> list[str]:
        errors: list[str] = []
        if self.metadata.get("run_class") != "REAL_ROBOT_CAPTURE":
            errors.append("metadata.run_class 不是 REAL_ROBOT_CAPTURE")
        if self.result.get("run_class") != "REAL_ROBOT_CAPTURE":
            errors.append("result.run_class 不是 REAL_ROBOT_CAPTURE")
        if self.result.get("capture_status") != "COMPLETE":
            errors.append("采集未完整结束")
        if self.result.get("rosbag_present") is not True:
            errors.append("result 未确认原始 rosbag 存在")
        if self.result.get("rosbag_metadata_present") is not True:
            errors.append("result 未确认 rosbag metadata.yaml 存在")
        if not (self.path / "rosbag" / "metadata.yaml").is_file():
            errors.append("缺少原始 rosbag/metadata.yaml")
        self.require_manifest_file("metadata.json", errors)
        self.require_manifest_file("result.json", errors)
        self.require_manifest_file("rosbag/metadata.yaml", errors)
        if not self.manifest_valid:
            errors.extend(f"清单校验失败：{item}" for item in self.manifest_errors)
        if not isinstance(self.protocol, dict):
            errors.append("没有冻结 teacher_protocol")
        else:
            if self.protocol.get("schema_version") != 1:
                errors.append("teacher_protocol.schema_version 必须为 1")
            if self.protocol.get("evidence_source") != "REAL_HARDWARE":
                errors.append("evidence_source 必须为 REAL_HARDWARE")
            if self.protocol.get("operator_observed") is not True:
                errors.append("operator_observed 必须明确为 true")
        return errors


def _read_protocol(session: Path, metadata: dict[str, Any]) -> dict[str, Any] | None:
    embedded = metadata.get("teacher_protocol")
    frozen_path = session / "teacher_protocol.json"
    frozen = None
    if frozen_path.is_file():
        frozen = read_json(frozen_path)
    if embedded is None:
        return frozen
    if not isinstance(embedded, dict):
        return None
    if frozen is not None and frozen != embedded:
        return None
    return embedded


def load_sessions(root: Path) -> list[SessionEvidence]:
    sessions: list[SessionEvidence] = []
    for session in discover_sessions(root):
        try:
            metadata = read_json(session / "metadata.json")
            result = read_json(session / "result.json")
            valid, errors = verify_manifest(session)
            manifest = read_json(session / "manifest.json")
            entries = manifest.get("files", [])
            manifest_paths = {
                str(entry.get("path", ""))
                for entry in entries
                if isinstance(entry, dict) and str(entry.get("path", "")).strip()
            }
            protocol = _read_protocol(session, metadata)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            sessions.append(
                SessionEvidence(
                    path=session,
                    experiment_id=session.name,
                    metadata={},
                    result={},
                    protocol=None,
                    manifest_valid=False,
                    manifest_errors=[f"会话不可读：{error}"],
                    manifest_paths=set(),
                )
            )
            continue
        sessions.append(
            SessionEvidence(
                path=session,
                experiment_id=str(result.get("experiment_id") or session.name),
                metadata=metadata,
                result=result,
                protocol=protocol,
                manifest_valid=valid,
                manifest_errors=errors,
                manifest_paths=manifest_paths,
            )
        )
    return sessions


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _bool_field(mapping: dict[str, Any], name: str, reasons: list[str], label: str) -> bool:
    if mapping.get(name) is True:
        return True
    reasons.append(f"{label}.{name} 未明确记录为 true")
    return False


def _float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def _diagnostic_values(row: dict[str, str]) -> dict[str, Any]:
    try:
        value = json.loads(row.get("values_json", ""))
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def _concurrent_degradation_times(session: SessionEvidence) -> list[float]:
    path = session.path / "structured" / "diagnostic_samples.csv"
    if not path.is_file():
        return []
    times: list[float] = []
    for row in _read_csv(path):
        values = _diagnostic_values(row)
        gnss = str(values.get("gnss_state", "")).strip().upper()
        lidar = str(values.get("lidar_state", "")).strip().upper()
        if gnss not in DEGRADED_STATES or lidar not in DEGRADED_STATES:
            continue
        timestamp = _float(values.get("timestamp"))
        if timestamp is None:
            timestamp = _float(row.get("message_timestamp"))
        if timestamp is None:
            timestamp = _float(row.get("received_timestamp"))
        if timestamp is not None:
            times.append(timestamp)
    return sorted(times)


def _position_samples_during(
    session: SessionEvidence, timestamps: Sequence[float], tolerance_sec: float
) -> tuple[list[PositionSample], list[str]]:
    path = session.path / "structured" / "position_samples.csv"
    if not path.is_file():
        return [], ["缺少 structured/position_samples.csv"]
    rows = _read_csv(path)
    if not rows:
        return [], ["position_samples.csv 为空"]
    if not timestamps:
        return [], ["诊断日志未证明 GNSS/BDS 与激光同时退化"]
    start = min(timestamps) - tolerance_sec
    end = max(timestamps) + tolerance_sec
    samples: list[PositionSample] = []
    errors: list[str] = []
    for line, row in enumerate(rows, 2):
        timestamp = _float(row.get("timestamp"))
        if timestamp is None or not start <= timestamp <= end:
            continue
        values = [
            _float(row.get("estimated_x")),
            _float(row.get("estimated_y")),
            _float(row.get("estimated_z")),
            _float(row.get("ground_truth_x")),
            _float(row.get("ground_truth_y")),
            _float(row.get("ground_truth_z")),
        ]
        if any(value is None for value in values):
            errors.append(f"position_samples.csv 第 {line} 行缺少有限 XY/Z 真值或估计值")
            continue
        samples.append(
            PositionSample(
                timestamp=timestamp,
                estimated_x=values[0],
                estimated_y=values[1],
                estimated_z=values[2],
                ground_truth_x=values[3],
                ground_truth_y=values[4],
                ground_truth_z=values[5],
            )
        )
    if not samples:
        errors.append("并发退化时间窗内没有完整 XY/Z 位置样本")
    return samples, errors


def evaluate_position(
    sessions: Sequence[SessionEvidence], threshold_m: float, minimum_samples: int
) -> RequirementResult:
    candidates = [item for item in sessions if item.requirement_id == POSITION_REQUIREMENT]
    reasons: list[str] = []
    accepted: list[str] = []
    samples: list[PositionSample] = []
    concurrent_samples = 0
    for session in candidates:
        session_reasons = list(session.base_errors)
        session.require_manifest_file("teacher_protocol.json", session_reasons)
        session.require_manifest_file("structured/diagnostic_samples.csv", session_reasons)
        session.require_manifest_file("structured/position_samples.csv", session_reasons)
        protocol = session.protocol or {}
        conditions = _dict(protocol.get("conditions"))
        truth = _dict(protocol.get("ground_truth"))
        _bool_field(conditions, "gnss_bds_interruption", session_reasons, "conditions")
        _bool_field(conditions, "lidar_degradation", session_reasons, "conditions")
        _bool_field(conditions, "concurrent", session_reasons, "conditions")
        _bool_field(truth, "independent_xy", session_reasons, "ground_truth")
        _bool_field(truth, "independent_z", session_reasons, "ground_truth")
        method = str(truth.get("method", "")).strip()
        if not method:
            session_reasons.append("ground_truth.method 为空")
        capture = _dict(session.metadata.get("evaluation_capture"))
        if capture.get("ground_truth_enabled") is not True:
            session_reasons.append("采集器未启用 XY 真值")
        if capture.get("ground_truth_z") is None:
            session_reasons.append("采集器未记录 Z 真值")
        frame = str(capture.get("ground_truth_frame", "")).strip()
        truth_frame = str(truth.get("coordinate_frame", "")).strip()
        if not frame or not truth_frame or frame != truth_frame:
            session_reasons.append("独立真值坐标系与采集坐标系未明确一致")
        degraded_times = _concurrent_degradation_times(session)
        session_samples, sample_errors = _position_samples_during(session, degraded_times, 0.6)
        session_reasons.extend(sample_errors)
        if session_reasons:
            reasons.extend(f"{session.experiment_id}: {reason}" for reason in session_reasons)
            continue
        accepted.append(session.experiment_id)
        samples.extend(session_samples)
        concurrent_samples += len(session_samples)

    if not candidates:
        status = "NOT_PROVEN"
        reasons.append("没有 POSITION_CONCURRENT_DEGRADATION 实机采集")
        observed: dict[str, Any] = {}
    elif len(samples) < minimum_samples:
        status = "INCOMPLETE_EVIDENCE"
        reasons.append(f"并发退化且具有独立 XY/Z 真值的样本不足：{len(samples)}/{minimum_samples}")
        observed = {"sample_count": len(samples)}
    else:
        horizontal = compute_metrics(samples)
        vertical = compute_vertical_metrics(samples)
        assert horizontal is not None and vertical is not None
        observed = {
            "sample_count": len(samples),
            "concurrent_degradation_sample_count": concurrent_samples,
            "horizontal": asdict(horizontal),
            "vertical": asdict(vertical),
        }
        status = (
            "PASS"
            if horizontal.p95 < threshold_m and vertical.p95 < threshold_m
            else "FAIL"
        )
        if status == "FAIL":
            reasons.append(
                f"P95 未同时小于 {threshold_m:.3f} m：XY={horizontal.p95:.4f}, Z={vertical.p95:.4f}"
            )
    return RequirementResult(
        POSITION_REQUIREMENT,
        "BDS/GNSS 中断与激光退化并发时 XY/Z 定位精度",
        status,
        f"XY P95 < {threshold_m:.2f} m 且 Z P95 < {threshold_m:.2f} m",
        observed,
        accepted,
        reasons,
    )


def _dynamic_evidence(session: SessionEvidence) -> tuple[bool, dict[str, int]]:
    path = session.path / "structured" / "lidar_quality.csv"
    totals = {"dynamic_candidate_count": 0, "auxiliary_dynamic_point_count": 0}
    if not path.is_file():
        return False, totals
    for row in _read_csv(path):
        for key in totals:
            value = _float(row.get(key))
            if value is not None and value > 0:
                totals[key] += int(value)
    return any(totals.values()), totals


def evaluate_features(
    sessions: Sequence[SessionEvidence], threshold: float, minimum_pairs: int
) -> RequirementResult:
    candidates = [item for item in sessions if item.requirement_id == FEATURE_REQUIREMENT]
    reasons: list[str] = []
    accepted: list[str] = []
    total_pairs = 0
    total_features = 0
    repeated_features = 0
    zero_pairs = 0
    session_metrics: list[dict[str, Any]] = []
    dynamic_totals = {"dynamic_candidate_count": 0, "auxiliary_dynamic_point_count": 0}
    for session in candidates:
        session_reasons = list(session.base_errors)
        session.require_manifest_file("teacher_protocol.json", session_reasons)
        session.require_manifest_file("structured/lidar_quality.csv", session_reasons)
        session.require_manifest_file("structured/lidar_features.csv", session_reasons)
        conditions = _dict((session.protocol or {}).get("conditions"))
        _bool_field(conditions, "robot_stationary", session_reasons, "conditions")
        _bool_field(conditions, "dynamic_target_present", session_reasons, "conditions")
        _bool_field(conditions, "dynamic_filter_enabled", session_reasons, "conditions")
        dynamic_seen, dynamic_counts = _dynamic_evidence(session)
        if not dynamic_seen:
            session_reasons.append("lidar_quality.csv 未记录任何动态候选或辅助动态剔除点")
        feature_path = session.path / "structured" / "lidar_features.csv"
        metrics = None
        if not feature_path.is_file():
            session_reasons.append("缺少 structured/lidar_features.csv")
        else:
            try:
                metrics = compute_feature_metrics(read_feature_csv(str(feature_path)))
            except (OSError, ValueError) as error:
                session_reasons.append(f"特征数据不可评测：{error}")
            if metrics is None:
                session_reasons.append("没有有效相邻特征帧")
            elif metrics.matching_method != "SPATIAL_2D":
                session_reasons.append("特征不是同坐标系二维空间匹配")
        if session_reasons:
            reasons.extend(f"{session.experiment_id}: {reason}" for reason in session_reasons)
            continue
        assert metrics is not None
        accepted.append(session.experiment_id)
        total_pairs += metrics.total_pairs
        total_features += metrics.total_features
        repeated_features += metrics.repeated_features
        zero_pairs += metrics.zero_feature_pair_count
        for key, value in dynamic_counts.items():
            dynamic_totals[key] += value
        session_metrics.append({"experiment_id": session.experiment_id, **asdict(metrics)})

    observed: dict[str, Any] = {
        "valid_pair_count": total_pairs,
        "zero_feature_pair_count": zero_pairs,
        "dynamic_evidence": dynamic_totals,
        "sessions": session_metrics,
    }
    if not candidates:
        status = "NOT_PROVEN"
        reasons.append("没有 DYNAMIC_FEATURE_REPEATABILITY 实机采集")
    elif total_pairs < minimum_pairs:
        status = "INCOMPLETE_EVIDENCE"
        reasons.append(f"有效相邻帧对不足：{total_pairs}/{minimum_pairs}")
    elif zero_pairs:
        status = "INCOMPLETE_EVIDENCE"
        reasons.append(f"存在 {zero_pairs} 个零特征帧对，不能把它们忽略后宣称达到指标")
    elif total_features <= 0:
        status = "INCOMPLETE_EVIDENCE"
        reasons.append("特征统计分母为零")
    else:
        repeatability = 2.0 * repeated_features / total_features
        observed["overall_repeatability"] = repeatability
        status = "PASS" if repeatability >= threshold else "FAIL"
        if status == "FAIL":
            reasons.append(f"有效特征重复率 {repeatability:.2%} 低于 {threshold:.2%}")
    return RequirementResult(
        FEATURE_REQUIREMENT,
        "动态目标剔除后的相邻帧有效特征重复率",
        status,
        f"R_pool >= {threshold:.0%}",
        observed,
        accepted,
        reasons,
    )


def evaluate_relocalization(
    sessions: Sequence[SessionEvidence], threshold: float, minimum_attempts: int
) -> RequirementResult:
    candidates = [item for item in sessions if item.requirement_id == RELOCALIZATION_REQUIREMENT]
    reasons: list[str] = []
    accepted: list[str] = []
    events = []
    trigger_classes: set[str] = set()
    for session in candidates:
        session_reasons = list(session.base_errors)
        session.require_manifest_file("teacher_protocol.json", session_reasons)
        session.require_manifest_file("structured/relocalization_events.csv", session_reasons)
        conditions = _dict((session.protocol or {}).get("conditions"))
        _bool_field(conditions, "controlled_real_trial", session_reasons, "conditions")
        _bool_field(conditions, "operator_supervised", session_reasons, "conditions")
        trigger = str(conditions.get("trigger_class", "")).strip().upper()
        if trigger not in RELOCALIZATION_TRIGGER_CLASSES:
            session_reasons.append("conditions.trigger_class 不是规定的三类真实触发之一")
        event_path = session.path / "structured" / "relocalization_events.csv"
        session_events = []
        if not event_path.is_file():
            session_reasons.append("缺少 structured/relocalization_events.csv")
        else:
            session_events = read_relocalization_csv(str(event_path))
            if not session_events:
                session_reasons.append("没有有效重定位终态")
            if any(
                event.outcome == "RECOVERED"
                and (event.time_to_recovery is None or event.time_to_recovery < 0.0)
                for event in session_events
            ):
                session_reasons.append("RECOVERED 事件缺少有效恢复时间")
        if session_reasons:
            reasons.extend(f"{session.experiment_id}: {reason}" for reason in session_reasons)
            continue
        accepted.append(session.experiment_id)
        trigger_classes.add(trigger)
        events.extend(session_events)

    observed: dict[str, Any] = {
        "attempt_count": len(events),
        "covered_trigger_classes": sorted(trigger_classes),
        "required_trigger_classes": sorted(RELOCALIZATION_TRIGGER_CLASSES),
    }
    if not candidates:
        status = "NOT_PROVEN"
        reasons.append("没有 RELOCALIZATION_ROBUSTNESS 实机采集")
    elif trigger_classes != RELOCALIZATION_TRIGGER_CLASSES:
        status = "INCOMPLETE_EVIDENCE"
        missing = sorted(RELOCALIZATION_TRIGGER_CLASSES - trigger_classes)
        reasons.append("真实触发类型未覆盖完整：" + ", ".join(missing))
    elif len(events) < minimum_attempts:
        status = "INCOMPLETE_EVIDENCE"
        reasons.append(f"有效重定位尝试不足：{len(events)}/{minimum_attempts}")
    else:
        metrics = compute_relocalization_metrics(events)
        assert metrics is not None
        observed["metrics"] = asdict(metrics)
        status = "PASS" if metrics.success_rate > threshold else "FAIL"
        if status == "FAIL":
            reasons.append(f"重定位成功率 {metrics.success_rate:.2%} 未高于 {threshold:.2%}")
    return RequirementResult(
        RELOCALIZATION_REQUIREMENT,
        "剧烈运动、跨域切换和感知退化下的重定位成功率",
        status,
        f"成功率 > {threshold:.0%}，且三类真实触发均覆盖",
        observed,
        accepted,
        reasons,
    )


def _valid_sha256(value: str) -> bool:
    return re.fullmatch(r"[0-9a-fA-F]{64}", value) is not None


def _valid_beidou_cycles(rows: Iterable[dict[str, str]]) -> tuple[int, list[str]]:
    grouped: dict[str, list[tuple[float, str]]] = {}
    errors: list[str] = []
    for line, row in enumerate(rows, 2):
        test_id = str(row.get("test_id", "")).strip()
        event = str(row.get("event", "")).strip().upper()
        timestamp = _float(row.get("timestamp"))
        if not test_id or event not in BEIDOU_EVENT_SEQUENCE or timestamp is None:
            errors.append(f"短报文事件 CSV 第 {line} 行字段无效")
            continue
        if not str(row.get("transport_reference", "")).strip():
            errors.append(f"短报文事件 CSV 第 {line} 行缺少真实传输引用")
            continue
        if not _valid_sha256(str(row.get("payload_sha256", "")).strip()):
            errors.append(f"短报文事件 CSV 第 {line} 行缺少有效载荷 SHA256")
            continue
        grouped.setdefault(test_id, []).append((timestamp, event))
    valid = 0
    for test_id, events in grouped.items():
        ordered = sorted(events)
        positions: list[float] = []
        for required in BEIDOU_EVENT_SEQUENCE:
            matches = [timestamp for timestamp, event in ordered if event == required]
            if not matches:
                break
            positions.append(matches[0])
        if len(positions) == len(BEIDOU_EVENT_SEQUENCE) and positions == sorted(positions):
            valid += 1
        else:
            errors.append(f"test_id={test_id} 没有完整按序双向闭环")
    return valid, errors


def evaluate_beidou(
    sessions: Sequence[SessionEvidence], minimum_cycles: int
) -> RequirementResult:
    candidates = [item for item in sessions if item.requirement_id == BEIDOU_REQUIREMENT]
    reasons: list[str] = []
    accepted: list[str] = []
    cycles = 0
    for session in candidates:
        session_reasons = list(session.base_errors)
        session.require_manifest_file("teacher_protocol.json", session_reasons)
        session.require_manifest_file("structured/beidou_short_message_events.csv", session_reasons)
        protocol = session.protocol or {}
        conditions = _dict(protocol.get("conditions"))
        for field in (
            "real_short_message_terminal",
            "satellite_transport",
            "service_active",
            "remote_endpoint_identified",
        ):
            _bool_field(conditions, field, session_reasons, "conditions")
        for field in ("terminal_model", "vendor_protocol"):
            if not str(protocol.get(field, "")).strip():
                session_reasons.append(f"{field} 为空")
        for field in ("raw_robot_log", "raw_remote_log"):
            relative = str(protocol.get(field, "")).strip()
            if not relative:
                session_reasons.append(f"{field} 为空")
            else:
                candidate = (session.path / relative).resolve()
                if session.path.resolve() not in candidate.parents or not candidate.is_file():
                    session_reasons.append(f"{field} 指向的原始日志不存在或越界")
                else:
                    session.require_manifest_file(Path(relative).as_posix(), session_reasons)
        events_path = session.path / "structured" / "beidou_short_message_events.csv"
        valid_cycles = 0
        if not events_path.is_file():
            session_reasons.append("缺少 structured/beidou_short_message_events.csv")
        else:
            rows = _read_csv(events_path)
            valid_cycles, event_errors = _valid_beidou_cycles(rows)
            session_reasons.extend(event_errors)
            if valid_cycles == 0:
                session_reasons.append("没有完整的机器人发送、远端接收、远端回复、机器人接收闭环")
        if session_reasons:
            reasons.extend(f"{session.experiment_id}: {reason}" for reason in session_reasons)
            continue
        accepted.append(session.experiment_id)
        cycles += valid_cycles

    observed = {"valid_bidirectional_cycle_count": cycles}
    if not candidates:
        status = "NOT_PROVEN"
        reasons.append("没有 BEIDOU_SHORT_MESSAGE_BIDIRECTIONAL 实机采集")
    elif cycles < minimum_cycles:
        status = "INCOMPLETE_EVIDENCE"
        reasons.append(f"真实双向短报文闭环不足：{cycles}/{minimum_cycles}")
    else:
        status = "PASS"
    return RequirementResult(
        BEIDOU_REQUIREMENT,
        "北斗短报文真实双向收发",
        status,
        f"至少 {minimum_cycles} 个可追溯卫星短报文双向闭环",
        observed,
        accepted,
        reasons,
    )


def evaluate_all(
    root: Path,
    *,
    position_threshold_m: float = 0.20,
    repeatability_threshold: float = 0.95,
    relocalization_threshold: float = 0.95,
    minimum_position_samples: int = 30,
    minimum_feature_pairs: int = 30,
    minimum_relocalization_attempts: int = 20,
    minimum_beidou_cycles: int = 1,
) -> dict[str, Any]:
    sessions = load_sessions(root)
    results = [
        evaluate_position(sessions, position_threshold_m, minimum_position_samples),
        evaluate_features(sessions, repeatability_threshold, minimum_feature_pairs),
        evaluate_relocalization(
            sessions, relocalization_threshold, minimum_relocalization_attempts
        ),
        evaluate_beidou(sessions, minimum_beidou_cycles),
    ]
    return {
        "schema_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "source_root": str(root.resolve()),
        "session_count": len(sessions),
        "overall_status": "PASS" if all(item.status == "PASS" for item in results) else "NOT_COMPLETE",
        "requirements": [asdict(item) for item in results],
        "claim_boundary": {
            "node_startup_is_not_metric_pass": True,
            "synthetic_or_fixture_data_is_not_real_evidence": True,
            "missing_independent_truth_is_not_position_accuracy": True,
            "missing_vendor_terminal_logs_is_not_short_message_verification": True,
        },
    }


def write_markdown(path: Path, summary: dict[str, Any]) -> None:
    labels = {
        "PASS": "通过",
        "FAIL": "指标未通过",
        "NOT_PROVEN": "尚未证明",
        "INCOMPLETE_EVIDENCE": "证据不完整",
    }
    lines = [
        "# DG-202611 老师四项要求严格验收结果",
        "",
        f"生成时间：{summary['generated_utc']}",
        f"采集会话数：{summary['session_count']}",
        f"总状态：{summary['overall_status']}",
        "",
    ]
    for item in summary["requirements"]:
        lines.extend(
            [
                f"## {item['title']}",
                "",
                f"- 状态：**{labels[item['status']]}（{item['status']}）**",
                f"- 判据：{item['target']}",
                f"- 接受的会话：{', '.join(item['accepted_sessions']) or '无'}",
                f"- 观测：`{json.dumps(item['observed'], ensure_ascii=False)}`",
            ]
        )
        for reason in item["reasons"]:
            lines.append(f"- 缺口/失败：{reason}")
        lines.append("")
    lines.extend(
        [
            "采集完成、节点启动或回放成功都不会自动算作老师指标通过。",
            "只有严格验收器接受的真实会话和数据才可回填为实测结论。",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_root")
    parser.add_argument("--output-dir")
    parser.add_argument("--position-threshold-m", type=float, default=0.20)
    parser.add_argument("--repeatability-threshold", type=float, default=0.95)
    parser.add_argument("--relocalization-threshold", type=float, default=0.95)
    parser.add_argument("--minimum-position-samples", type=int, default=30)
    parser.add_argument("--minimum-feature-pairs", type=int, default=30)
    parser.add_argument("--minimum-relocalization-attempts", type=int, default=20)
    parser.add_argument("--minimum-beidou-cycles", type=int, default=1)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = create_parser().parse_args(argv)
    root = Path(arguments.capture_root).expanduser().resolve()
    if not root.exists():
        print(f"Error: capture root does not exist: {root}", file=sys.stderr)
        return 2
    numeric = (
        arguments.position_threshold_m,
        arguments.repeatability_threshold,
        arguments.relocalization_threshold,
    )
    counts = (
        arguments.minimum_position_samples,
        arguments.minimum_feature_pairs,
        arguments.minimum_relocalization_attempts,
        arguments.minimum_beidou_cycles,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in numeric):
        print("Error: thresholds must be finite and positive", file=sys.stderr)
        return 2
    if any(value <= 0 for value in counts):
        print("Error: minimum evidence counts must be positive", file=sys.stderr)
        return 2
    output = (
        Path(arguments.output_dir).expanduser().resolve()
        if arguments.output_dir
        else root.parent / f"{root.name}_teacher_evaluation_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    if output.exists():
        print(f"Error: output directory already exists: {output}", file=sys.stderr)
        return 2
    output.mkdir(parents=True)
    try:
        summary = evaluate_all(
            root,
            position_threshold_m=arguments.position_threshold_m,
            repeatability_threshold=arguments.repeatability_threshold,
            relocalization_threshold=arguments.relocalization_threshold,
            minimum_position_samples=arguments.minimum_position_samples,
            minimum_feature_pairs=arguments.minimum_feature_pairs,
            minimum_relocalization_attempts=arguments.minimum_relocalization_attempts,
            minimum_beidou_cycles=arguments.minimum_beidou_cycles,
        )
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_markdown(output / "summary.md", summary)
    print(output)
    return 0 if summary["overall_status"] == "PASS" else 3


if __name__ == "__main__":
    raise SystemExit(main())
