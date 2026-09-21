import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from beidou_short_message_interface import MessageState, ShortMessageQueue  # noqa: E402


def test_delivery_waits_for_transport_and_real_acknowledgement():
    queue = ShortMessageQueue(
        max_payload_bytes=16,
        acknowledgement_timeout_sec=5.0,
        max_retries=1,
    )
    message = queue.enqueue("terminal-B", b"alarm", message_id="msg-1", now=10.0)

    assert queue.poll(transport_ready=False, now=11.0) == []
    assert message.state == MessageState.QUEUED
    assert message.attempt_count == 0

    work = queue.poll(transport_ready=True, now=12.0)
    assert [(item.message_id, item.attempt_number) for item in work] == [("msg-1", 1)]
    assert message.state == MessageState.AWAITING_ACK

    queue.acknowledge(
        "msg-1",
        transport_reference="vendor-ack-7",
        accepted=True,
        now=13.0,
    )
    assert message.state == MessageState.DELIVERED
    assert message.transport_reference == "vendor-ack-7"


def test_missing_acknowledgement_retries_then_fails_without_fake_delivery():
    queue = ShortMessageQueue(
        max_payload_bytes=16,
        acknowledgement_timeout_sec=2.0,
        max_retries=1,
    )
    message = queue.enqueue("terminal-B", b"alarm", message_id="msg-2", now=0.0)

    assert queue.poll(transport_ready=True, now=0.0)[0].attempt_number == 1
    assert queue.poll(transport_ready=True, now=2.0)[0].attempt_number == 2
    assert queue.poll(transport_ready=True, now=4.0) == []
    assert message.state == MessageState.FAILED
    assert message.failure_reason == "ACK_TIMEOUT"
    assert all(record["state"] != "DELIVERED" for record in queue.audit_records())


def test_invalid_messages_and_impossible_acknowledgement_are_rejected():
    queue = ShortMessageQueue(max_payload_bytes=4)
    queue.enqueue("terminal-B", b"1234", message_id="fixed", now=0.0)

    with pytest.raises(ValueError, match="duplicate"):
        queue.enqueue("terminal-B", b"ok", message_id="fixed", now=0.0)
    with pytest.raises(ValueError, match="transport limit"):
        queue.enqueue("terminal-B", b"12345", now=0.0)
    with pytest.raises(ValueError, match="acknowledgement is impossible"):
        queue.acknowledge(
            "fixed",
            transport_reference="ack-before-send",
            accepted=True,
            now=1.0,
        )


def test_inbound_message_retains_transport_reference_and_audit_payload():
    queue = ShortMessageQueue(max_payload_bytes=16)
    message = queue.record_inbound(
        "terminal-B",
        b"reply",
        transport_reference="vendor-rx-9",
        now=20.0,
    )

    assert message.transport_reference == "vendor-rx-9"
    assert queue.inbound_messages() == [message]
    assert queue.audit_records()[-1]["payload_hex"] == b"reply".hex()
