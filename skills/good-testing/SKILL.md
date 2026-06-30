---
name: good-testing
description: Guides testing changes in this ROS2 workspace. Use when modifying, adding, or reviewing tests, pytest files, QML assertions, Nav2/SLAM config tests, or patrol/UI backend test coverage.
---

# Good Testing

Before adding or changing tests in this repository, read `docs/GOOD_TESTING_GUIDE.md`.

## Rules

1. Protect real failures: safety, map semantics, patrol state, command routing, schemas, and public UI/backend contracts.
2. Do not chase test count. Prefer one existing focused test over a new duplicate.
3. Test behavior and contracts, not incidental wording, layout dimensions, or component order.
4. For Nav2/SLAM values, lock exact numbers only when they are safety boundaries or file-format semantics. Otherwise check presence, positivity, range, or cross-parameter consistency.
5. Keep fakes tiny. Implement only methods the tested path calls.
6. Do not add hardware requirements to default pytest runs.

## When Editing Existing Tests

- Keep emergency stop, control lock, velocity/direction, footprint, map unknown, schema, route preview, and patrol state-machine coverage.
- Relax brittle UI text/layout assertions unless the text is an API.
- Merge same-file duplicate assertions only when it clearly reduces noise.
- Do not touch runtime code just to satisfy a test cleanup.
