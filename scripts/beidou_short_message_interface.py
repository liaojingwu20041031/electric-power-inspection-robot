#!/usr/bin/env python3
"""Vendor-neutral contract for a BeiDou short-message transport.

This module deliberately stops at the boundary that can be implemented without
inventing a vendor protocol. A hardware adapter is expected to take
``SendInstruction`` objects, write the vendor frame, and report a real terminal
acknowledgement through ``acknowledge``.

It is not an RTK/NTRIP, cellular, or LoRa transport implementation.
"""

from __future__ import annotations

import argparse
import json
import time
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional


class MessageState(str, Enum):
    QUEUED = "QUEUED"
    AWAITING_ACK = "AWAITING_ACK"
    DELIVERED = "DELIVERED"
    FAILED = "FAILED"


@dataclass
class OutboundMessage:
    message_id: str
    destination: str
    payload: bytes
    created_at: float
    state: MessageState = MessageState.QUEUED
    attempt_count: int = 0
    last_sent_at: Optional[float] = None
    acknowledgement_deadline: Optional[float] = None
    delivered_at: Optional[float] = None
    transport_reference: Optional[str] = None
    failure_reason: Optional[str] = None


@dataclass(frozen=True)
class SendInstruction:
    message_id: str
    destination: str
    payload: bytes
    attempt_number: int


@dataclass(frozen=True)
class InboundMessage:
    source: str
    payload: bytes
    received_at: float
    transport_reference: str


class ShortMessageQueue:
    """Queue, acknowledgement, retry, and audit logic shared by vendor adapters."""

    def __init__(
        self,
        *,
        max_payload_bytes: int,
        acknowledgement_timeout_sec: float = 30.0,
        max_retries: int = 2,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if max_payload_bytes <= 0:
            raise ValueError("max_payload_bytes must be positive")
        if acknowledgement_timeout_sec <= 0.0:
            raise ValueError("acknowledgement_timeout_sec must be positive")
        if max_retries < 0:
            raise ValueError("max_retries cannot be negative")
        self.max_payload_bytes = int(max_payload_bytes)
        self.acknowledgement_timeout_sec = float(acknowledgement_timeout_sec)
        self.max_retries = int(max_retries)
        self._clock = clock
        self._outbound: Dict[str, OutboundMessage] = {}
        self._inbound: List[InboundMessage] = []
        self._audit: List[dict] = []

    @staticmethod
    def _validate_endpoint(value: str, field_name: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{field_name} must be a non-empty string")
        return value.strip()

    def _validate_payload(self, payload: bytes) -> bytes:
        if not isinstance(payload, bytes):
            raise TypeError("payload must be bytes")
        if not payload:
            raise ValueError("payload cannot be empty")
        if len(payload) > self.max_payload_bytes:
            raise ValueError(
                f"payload is {len(payload)} bytes; transport limit is "
                f"{self.max_payload_bytes} bytes"
            )
        return payload

    def _now(self, value: Optional[float]) -> float:
        return float(self._clock() if value is None else value)

    def _record(self, event: str, *, at: float, message: OutboundMessage, **extra) -> None:
        record = {
            "event": event,
            "at": at,
            "message_id": message.message_id,
            "destination": message.destination,
            "state": message.state.value,
            "attempt_count": message.attempt_count,
            "payload_hex": message.payload.hex(),
            "payload_size_bytes": len(message.payload),
        }
        record.update(extra)
        self._audit.append(record)

    def enqueue(
        self,
        destination: str,
        payload: bytes,
        *,
        message_id: Optional[str] = None,
        now: Optional[float] = None,
    ) -> OutboundMessage:
        destination = self._validate_endpoint(destination, "destination")
        payload = self._validate_payload(payload)
        message_id = message_id or uuid.uuid4().hex
        message_id = self._validate_endpoint(message_id, "message_id")
        if message_id in self._outbound:
            raise ValueError(f"duplicate message_id: {message_id}")
        message = OutboundMessage(
            message_id=message_id,
            destination=destination,
            payload=payload,
            created_at=self._now(now),
        )
        self._outbound[message_id] = message
        self._record("ENQUEUED", at=message.created_at, message=message)
        return message

    def _instruction(self, message: OutboundMessage, now: float) -> SendInstruction:
        message.attempt_count += 1
        message.state = MessageState.AWAITING_ACK
        message.last_sent_at = now
        message.acknowledgement_deadline = now + self.acknowledgement_timeout_sec
        self._record("SEND_REQUESTED", at=now, message=message)
        return SendInstruction(
            message_id=message.message_id,
            destination=message.destination,
            payload=message.payload,
            attempt_number=message.attempt_count,
        )

    def poll(
        self,
        *,
        transport_ready: bool,
        now: Optional[float] = None,
    ) -> List[SendInstruction]:
        """Return work for a real transport adapter.

        Attempts are counted only when an instruction is emitted. If the
        hardware transport is unavailable, queued or expired messages wait
        without consuming retries.
        """

        current = self._now(now)
        instructions: List[SendInstruction] = []
        for message in self._outbound.values():
            if message.state in (MessageState.DELIVERED, MessageState.FAILED):
                continue
            if message.state == MessageState.QUEUED:
                if transport_ready:
                    instructions.append(self._instruction(message, current))
                continue
            deadline = message.acknowledgement_deadline
            if deadline is None or current < deadline:
                continue
            if message.attempt_count >= 1 + self.max_retries:
                message.state = MessageState.FAILED
                message.failure_reason = "ACK_TIMEOUT"
                self._record("FAILED", at=current, message=message, reason="ACK_TIMEOUT")
            elif transport_ready:
                instructions.append(self._instruction(message, current))
        return instructions

    def acknowledge(
        self,
        message_id: str,
        *,
        transport_reference: str,
        accepted: bool,
        now: Optional[float] = None,
        reason: Optional[str] = None,
    ) -> OutboundMessage:
        """Apply a real acknowledgement reported by the hardware adapter."""

        if message_id not in self._outbound:
            raise KeyError(f"unknown message_id: {message_id}")
        reference = self._validate_endpoint(transport_reference, "transport_reference")
        message = self._outbound[message_id]
        if message.state != MessageState.AWAITING_ACK:
            raise ValueError(
                f"message {message_id} is {message.state.value}; acknowledgement is impossible"
            )
        current = self._now(now)
        message.transport_reference = reference
        message.acknowledgement_deadline = None
        if accepted:
            message.state = MessageState.DELIVERED
            message.delivered_at = current
            self._record(
                "ACK_ACCEPTED",
                at=current,
                message=message,
                transport_reference=reference,
            )
        elif message.attempt_count >= 1 + self.max_retries:
            message.state = MessageState.FAILED
            message.failure_reason = reason or "TRANSPORT_REJECTED"
            self._record(
                "ACK_REJECTED_FINAL",
                at=current,
                message=message,
                transport_reference=reference,
                reason=message.failure_reason,
            )
        else:
            message.state = MessageState.QUEUED
            self._record(
                "ACK_REJECTED_RETRY_QUEUED",
                at=current,
                message=message,
                transport_reference=reference,
                reason=reason or "TRANSPORT_REJECTED",
            )
        return message

    def record_inbound(
        self,
        source: str,
        payload: bytes,
        *,
        transport_reference: str,
        now: Optional[float] = None,
    ) -> InboundMessage:
        source = self._validate_endpoint(source, "source")
        payload = self._validate_payload(payload)
        reference = self._validate_endpoint(transport_reference, "transport_reference")
        message = InboundMessage(
            source=source,
            payload=payload,
            received_at=self._now(now),
            transport_reference=reference,
        )
        self._inbound.append(message)
        self._audit.append(
            {
                "event": "INBOUND_RECEIVED",
                "at": message.received_at,
                "source": source,
                "transport_reference": reference,
                "payload_hex": payload.hex(),
                "payload_size_bytes": len(payload),
            }
        )
        return message

    def get(self, message_id: str) -> OutboundMessage:
        return self._outbound[message_id]

    def inbound_messages(self) -> List[InboundMessage]:
        return list(self._inbound)

    def audit_records(self) -> List[dict]:
        return [dict(item) for item in self._audit]


def capability_report() -> dict:
    return {
        "component": "beidou_short_message_common_contract",
        "contract_status": "IMPLEMENTED_AND_LOCALLY_TESTABLE",
        "implemented": [
            "payload_and_endpoint_validation",
            "outbound_queue",
            "transport_readiness_gate",
            "acknowledgement_required_for_delivery",
            "finite_timeout_retry",
            "inbound_recording",
            "hex_encoded_audit_records",
        ],
        "vendor_transport_status": "MISSING_PROTOCOL_AND_MODULE_DETAILS",
        "hardware_runtime_status": "NOT_VERIFIED",
        "required_next_data": [
            "terminal_model",
            "vendor_protocol_manual",
            "serial_parameters",
            "payload_and_rate_limits",
            "acknowledgement_semantics",
            "antenna_and_service_activation",
            "remote_receiver_identity",
        ],
        "not_equivalent_to": ["GNSS_RTK", "NTRIP", "4G_LTE", "LoRa"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Report the readiness of the BeiDou short-message common contract"
    )
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args()
    report = capability_report()
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return
    print("BeiDou short-message common contract")
    print(f"Contract: {report['contract_status']}")
    print(f"Vendor transport: {report['vendor_transport_status']}")
    print(f"Hardware runtime: {report['hardware_runtime_status']}")


if __name__ == "__main__":
    main()
