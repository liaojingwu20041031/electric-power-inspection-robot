# GNSS/RTK and BeiDou Software Status

## GNSS/RTK Software Path: READY

The GNSS/RTK software chain is complete and ready for hardware integration:

### Software Chain
```
WTRTK980 Hardware (GNSS/RTK receiver)
  ↓ Serial NMEA frames
wtrtk980_nmea_node (C++)
  ↓ /gps/fix, /gps/rtk_status
gnss_quality_node (Python)
  ↓ /dg/gnss/quality, /dg/gnss/accepted_fix
multisource_fusion_node (MODEL-1)
  ↓ Fused position estimate
```

### Implementation Files
1. **WTRTK980 NMEA Parser (C++)**
   - `src/ylhb_base/include/ylhb_base/wtrtk980_nmea.hpp`
   - `src/ylhb_base/src/wtrtk980_nmea.cpp`
   - `src/ylhb_base/src/wtrtk980_nmea_node.cpp`
   - Parses NMEA frames from WTRTK980 receiver
   - Publishes sensor_msgs/NavSatFix
   - Publishes RTK status diagnostics

2. **GNSS Quality Gate (Python)**
   - `src/ylhb_base/scripts/gnss_quality_gate.py`
   - `src/ylhb_base/scripts/gnss_quality_node.py`
   - Quality assessment: GOOD / DEGRADED / REJECTED
   - HDOP, satellite count, differential age checks
   - State machine with hysteresis

3. **MODEL-1 Fusion Integration**
   - `src/ylhb_base/scripts/multisource_fusion_core.py`
   - `src/ylhb_base/scripts/multisource_fusion_node.py`
   - Consumes `/dg/gnss/accepted_fix`
   - Fuses with LiDAR, UWB, odometry
   - Geodetic → ENU conversion

### Launch Integration
File: `src/ylhb_base/launch/dg_navigation_integration.launch.py`

```python
gnss = Node(
    package="ylhb_base",
    executable="gnss_quality_node",
    parameters=[{
        "fix_topic": "/gps/fix",
        "status_topic": "/gps/rtk_status",
        "quality_topic": "/dg/gnss/quality",
    }]
)

multisource_fusion = Node(
    package="ylhb_base",
    executable="multisource_fusion_node",
    parameters=[{
        "gnss_fix_topic": "/dg/gnss/accepted_fix",
        "gnss_quality_topic": "/dg/gnss/quality",
        "gnss_origin_latitude": gnss_origin_latitude,
        "gnss_origin_longitude": gnss_origin_longitude,
        # ...
    }]
)
```

### Testing Status
- ✅ GNSS quality gate: 13 unit tests PASS
- ✅ MODEL-1 GNSS fusion: Covered in 45 fusion tests
- ✅ System scenarios: GNSS degradation (SYS_02) PASS

### Hardware Integration Checklist
- ✅ Parser interface defined
- ✅ Serial port parameterized (not hardcoded)
- ✅ Quality gate thresholds configurable
- ✅ ENU origin configurable via launch arguments
- ⏳ Real WTRTK980 hardware (next phase)
- ⏳ Real RTK base station (next phase)
- ⏳ Real NTRIP corrections (next phase)

---

## BeiDou Short Message: NO_PROTOCOL_AVAILABLE

### Investigation Results
Searched the repository and available DG-202611 materials for BeiDou short message protocol specifications, parser code, or module documentation.

**Finding:** No BeiDou short message protocol or implementation found in the codebase.

### Current Status
- ❌ No protocol specification
- ❌ No parser implementation
- ❌ No hardware module identified
- ❌ No message frame format defined

### Interface Skeleton Created
File: `scripts/beidou_short_message_interface.py`

Provides minimal interface placeholder for future implementation when protocol becomes available:

```python
class BeiDouShortMessage:
    """Placeholder for BeiDou short message when protocol available."""
    timestamp: float
    message_id: int
    direction: str  # "SEND" or "RECEIVE"
    payload: bytes
    status: str
```

### Important Notes
1. **NOT NTRIP:** NTRIP is RTK correction data via internet, not BeiDou short message
2. **NOT 4G:** 4G/LTE is commercial cellular, not BeiDou satellite messaging
3. **NOT LoRa:** LoRa is ISM band radio, not BeiDou satellite

BeiDou short message is satellite-based text messaging (similar to Iridium SBD), requires:
- BeiDou-compatible hardware module
- Specific message protocol/frame format
- Satellite visibility and subscription

### Next Steps (When Protocol Available)
1. Obtain BeiDou short message protocol specification
2. Identify actual hardware module model
3. Implement parser for actual frame format
4. Add ROS2 node wrapper
5. Register with CMakeLists.txt
6. Add to launch file
7. Create unit tests

### Verdict
**BEIDOU_SHORT_MESSAGE_SOFTWARE_INTERFACE: NO_PROTOCOL_AVAILABLE**

---

## Summary

| Component | Status | Reason |
|-----------|--------|--------|
| GNSS/RTK Software Path | ✅ READY | Complete chain: WTRTK980 → quality gate → MODEL-1 |
| BeiDou Short Message | ❌ NO_PROTOCOL_AVAILABLE | No protocol spec or hardware identified |

The GNSS/RTK path is production-ready for hardware integration. BeiDou short message requires protocol specification before implementation can begin.
