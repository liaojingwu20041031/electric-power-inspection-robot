#!/bin/bash
# DG202611 pre-hardware preflight checks
# Runs all tests and basic validation before experiments

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_WS="${DG_BASE_WS:-/home/nvidia/ros2_DL}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${BASE_WS}/install/setup.bash" ]; then
    source "${BASE_WS}/install/setup.bash"
else
    echo "Robot base workspace is not built: ${BASE_WS}" >&2
    exit 2
fi
set -u

echo "===DG202611 PREFLIGHT CHECKS==="
echo "Repository: $REPO_ROOT"
echo ""

# Track results
PASS_COUNT=0
FAIL_COUNT=0

run_check() {
    local name="$1"
    shift
    echo "[$name] Running..."
    if "$@"; then
        echo "[$name] PASS"
        PASS_COUNT=$((PASS_COUNT + 1))
        return 0
    else
        echo "[$name] FAIL"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    fi
}

run_check "AUXILIARY_ROS_SMOKE_SCRIPT_SYNTAX" bash -n "$REPO_ROOT/scripts/run_auxiliary_ros_smoke.sh"

# Step 1: ROS2 build
echo ""
echo "===STEP 1: ROS2 BUILD==="
cd "$REPO_ROOT"
if run_check "ROS2_BUILD" colcon build \
    --packages-select ylhb_base ylhb_llm \
    --allow-overriding ylhb_base ylhb_llm \
    --cmake-args -DBUILD_TESTING=OFF; then
    set +u
    source install/setup.bash
    set -u
else
    echo "Build failed, cannot proceed with tests"
    exit 1
fi

# Step 2: Python import checks
echo ""
echo "===STEP 2: PYTHON IMPORT CHECKS==="
cd "$REPO_ROOT/src/ylhb_base/scripts"

run_check "IMPORT_MODEL1" python3 -c "import multisource_fusion_core"
run_check "IMPORT_MODEL2" python3 -c "import lidar_robust_features"
run_check "IMPORT_MODEL3" python3 -c "import active_relocalization_core"
run_check "IMPORT_HEALTH" python3 -c "import navigation_health_core"
run_check "IMPORT_MISSION_GATE" python3 -c "import mission_activity"
run_check "IMPORT_UWB_MODEL" python3 -c "import uwb_data_model"
run_check "IMPORT_UWB_ESTIMATOR" python3 -c "import uwb_2d_estimator"
run_check "IMPORT_UWB_QUALITY" python3 -c "import uwb_quality_gate"
run_check "IMPORT_UWB_INPUT" python3 -c "import uwb_input"
run_check "IMPORT_AUX_SENSOR_DATA" python3 -c "import auxiliary_sensor_data"
run_check "IMPORT_EVENT_FEATURE_CORE" python3 -c "import event_feature_core"
run_check "IMPORT_POINTCLOUD_READER" python3 -c "import pointcloud_reader"
cd "$REPO_ROOT/scripts"
run_check "IMPORT_BEIDOU_SHORT_MESSAGE_CONTRACT" python3 -c "import beidou_short_message_interface"

# Step 3: Unit tests
echo ""
echo "===STEP 3: UNIT TESTS==="
cd "$REPO_ROOT"

run_check "MODEL1_TESTS" python3 -m pytest src/ylhb_base/test/test_multisource_fusion_core.py -v --tb=no
run_check "MODEL2_TESTS" python3 -m pytest src/ylhb_base/test/test_lidar_robust_features.py -v --tb=no
run_check "MODEL3_TESTS" python3 -m pytest src/ylhb_base/test/test_active_relocalization_core.py -v --tb=no
run_check "HEALTH_TESTS" python3 -m pytest src/ylhb_base/test/test_navigation_health_core.py -v --tb=no
run_check "UWB_TESTS" python3 -m pytest src/ylhb_base/test/test_uwb_model1_integration.py src/ylhb_base/test/test_uwb_model3_compatibility.py -v --tb=no
run_check "UWB_MEASURED_EVIDENCE_TEST" python3 -m pytest src/ylhb_base/test/test_measured_uwb_analysis.py -v --tb=no
run_check "INTEGRATION_PROTOCOL_TEST" python3 -m pytest src/ylhb_base/test/test_navigation_protocols.py -v --tb=no
run_check "AUX_SENSOR_PROTOCOL_TEST" python3 -m pytest src/ylhb_base/test/test_multisource_fusion_protocols.py -v --tb=no
run_check "EVENT_FEATURE_TEST" python3 -m pytest src/ylhb_base/test/test_event_feature_core.py -v --tb=no
run_check "POINTCLOUD_READER_TEST" python3 -m pytest src/ylhb_base/test/test_pointcloud_reader.py -v --tb=no
run_check "AUX_SENSOR_VALIDATION_TEST" python3 -m pytest src/ylhb_base/test/test_auxiliary_sensor_validation.py -v --tb=no
run_check "AUX_SESSION_TOOL_TEST" python3 -m pytest src/ylhb_base/test/test_auxiliary_session_tool.py -v --tb=no
run_check "BEIDOU_SHORT_MESSAGE_CONTRACT_TEST" python3 -m pytest src/ylhb_base/test/test_beidou_short_message_contract.py -v --tb=no

# Step 4: System scenarios
echo ""
echo "===STEP 4: SYSTEM SCENARIOS==="
run_check "SYSTEM_SCENARIOS" python3 scripts/sys_scenarios.py

# Step 5: Evaluator tests
echo ""
echo "===STEP 5: EVALUATOR TESTS==="
run_check "POSITION_EVALUATOR_TEST" python3 -m pytest src/ylhb_base/test/test_position_error_evaluator.py -v --tb=no
run_check "REAL_ROBOT_CAPTURE_TEST" python3 -m pytest src/ylhb_base/test/test_real_robot_capture.py -v --tb=no
run_check "TEACHER_REQUIREMENTS_EVALUATOR_TEST" python3 -m pytest src/ylhb_base/test/test_teacher_requirements_evaluator.py -v --tb=no

# Step 6: Evaluator import checks
echo ""
echo "===STEP 6: EVALUATOR IMPORTS==="
cd "$REPO_ROOT/scripts"
run_check "IMPORT_POS_EVAL" python3 -c "import position_error_evaluator"
run_check "IMPORT_FEAT_EVAL" python3 -c "import feature_repeatability_evaluator"
run_check "IMPORT_RELOC_EVAL" python3 -c "import relocalization_evaluator"
run_check "IMPORT_EXP_LOGGER" python3 -c "import experiment_logger"
run_check "IMPORT_SESSION_TOOL" python3 -c "import experiment_session_tool"
run_check "IMPORT_CAPTURE_EVAL" python3 -c "import evaluate_real_robot_sessions"
run_check "IMPORT_TEACHER_REQUIREMENTS_EVAL" python3 -c "import teacher_requirements_evaluator"
run_check "IMPORT_STRUCTURED_CAPTURE" python3 -c "import sys; sys.path.insert(0, '../src/ylhb_base/scripts'); import evidence_capture_node"

# Final summary
echo ""
echo "===PREFLIGHT SUMMARY==="
echo "PASSED: $PASS_COUNT"
echo "FAILED: $FAIL_COUNT"

if [ $FAIL_COUNT -eq 0 ]; then
    echo ""
    echo "✅ PREFLIGHT: PASS"
    exit 0
else
    echo ""
    echo "❌ PREFLIGHT: FAIL"
    exit 1
fi
