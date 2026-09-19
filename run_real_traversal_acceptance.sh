#!/usr/bin/env bash
set -euo pipefail
umask 077

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export SCOUT_WORKSPACE="${WORKSPACE}"
export PYTHONPATH="${WORKSPACE}/src/NeuPAN:${WORKSPACE}/.deps/python${PYTHONPATH:+:${PYTHONPATH}}"
SCENARIO="${1:-}"
OUTPUT_BASE="${2:-${WORKSPACE}/acceptance_results}"
CORE_ONLY="${CORE_ONLY:-true}"
if [[ "${SCENARIO}" != "curtain" && "${SCENARIO}" != "movable_box" && "${SCENARIO}" != "fixed_box" ]]; then
    echo "Usage: $0 curtain|movable_box|fixed_box [output-base]" >&2
    exit 2
fi
if [[ "${SCOUT_ALLOW_SENSITIVE_RECORDING:-0}" != "1" ]]; then
    echo "This command records images, maps, poses, and model output. Set SCOUT_ALLOW_SENSITIVE_RECORDING=1 after selecting a protected output directory." >&2
    exit 2
fi

echo "Real acceptance uses external adapters configured by their own module:factory variables." >&2
if [[ -z "${ISAACSIM_PYTHON_EXE:-}" ]]; then
    if [[ -n "${ISAACSIM_PATH:-}" && -x "${ISAACSIM_PATH}/python.sh" ]]; then
        export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
    else
        echo "Set ISAACSIM_PYTHON_EXE (or ISAACSIM_PATH) to the Isaac Sim python.sh executable." >&2
        exit 2
    fi
fi

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Isaac acceptance is entirely local. Isolate it from physical Scout or other
# ROS 2 participants that may also use domain 42 on the LAN.
export ROS_LOCALHOST_ONLY="${SCOUT_ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE}/runtime/ros_logs}"
mkdir -p -- "${ROS_LOG_DIR}"
chmod 700 -- "${ROS_LOG_DIR}"
# ROS Humble's generated setup scripts probe optional variables such as
# AMENT_TRACE_SETUP_FILES and are not safe to source with nounset enabled.
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
source "${WORKSPACE}/groundingdinoVLM/install/local_setup.bash"
source "${WORKSPACE}/llmModule/install/setup.bash"
source "${WORKSPACE}/llmdecision/install/setup.bash"
set -u

(
    set +u
    source "${WORKSPACE}/groundingdinoVLM/setup_groundingdino_vlm.sh"
    set -u
    python3 "${WORKSPACE}/groundingdinoVLM/src/obstacle_traversal/obstacle_traversal/preflight.py"
)

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${OUTPUT_BASE}/${SCENARIO}_${STAMP}"
mkdir -p "${RUN_DIR}"
EVENTS="${RUN_DIR}/events.jsonl"
BAG="${RUN_DIR}/rosbag"
REPORT="${RUN_DIR}/report.json"
SYSTEM_LOG="${RUN_DIR}/terminal1_system.log"
RECORDER_LOG="${RUN_DIR}/recorder.log"
BAG_LOG="${RUN_DIR}/rosbag.log"
HASHES="${RUN_DIR}/SHA256SUMS"

system_pid=""
bag_pid=""
recorder_pid=""
cleanup() {
    set +e
    [[ -n "${recorder_pid}" ]] && kill -INT "${recorder_pid}" 2>/dev/null
    [[ -n "${bag_pid}" ]] && kill -INT "${bag_pid}" 2>/dev/null
    [[ -n "${system_pid}" ]] && kill -INT "${system_pid}" 2>/dev/null
    wait "${recorder_pid}" 2>/dev/null
    wait "${bag_pid}" 2>/dev/null
    wait "${system_pid}" 2>/dev/null
}
abort_run() {
    trap - EXIT INT TERM
    cleanup
    exit 130
}
trap cleanup EXIT
trap abort_run INT TERM

(
    set +u
    source "${WORKSPACE}/groundingdinoVLM/setup_groundingdino_vlm.sh"
    source "${WORKSPACE}/llmdecision/install/setup.bash"
    set -u
    exec ros2 launch obstacle_traversal real_isaac_traversal.launch.py \
        workspace_dir:="${WORKSPACE}" \
        acceptance_obstacle:="${SCENARIO}" \
        core_only:="${CORE_ONLY}" \
        isaac_python:="${ISAACSIM_PYTHON_EXE}" \
        headless:="${HEADLESS:-true}" use_rviz:="${USE_RVIZ:-false}"
) >"${SYSTEM_LOG}" 2>&1 &
system_pid=$!

ros2 bag record -o "${BAG}" \
    /user_instruction \
    /semantic_navigation/intent /semantic_navigation/resolution \
    /semantic_navigation/verification /semantic_navigation/status \
    /semantic_navigation/verification_readiness /semantic_navigation/readiness \
    /map /semantic_validation/map /semantic_validation/global_costmap/costmap \
    /clicked_point /plan /neupan_initial_path \
    /scan_raw /scan /scan_removed \
    /groundingdino_vlm/ready /llmdecision/ready \
    /obstacle_traversal/ready /obstacle_traversal/status \
    /obstacle_traversal/filter_authorization /obstacle_traversal/filter_state \
    /obstacle_traversal/approach_cmd_vel \
    /obstacle_traversal/mechanical_assessment \
    /piper/yolo_ready /piper/yolo_obstacle_result \
    /obstacle_traversal/debug_image /groundingdino_vlm/debug_image \
    /neupan_cmd_vel_raw /neupan_cmd_vel /cmd_vel \
    /tf /tf_static /odom /isaac/acceptance_furniture_pose \
    /isaac_joint_states /isaac_joint_command \
    /semantic_mapping/obstacle_cloud /semantic_mapping/labels /robot_marker \
    /groundingdino_vlm/verify_target/_action/feedback \
    /groundingdino_vlm/verify_target/_action/status \
    /llmdecision/assess_pushability/_action/feedback \
    /llmdecision/assess_pushability/_action/status \
    /obstacle_traversal/approach_obstacle/_action/feedback \
    /obstacle_traversal/approach_obstacle/_action/status \
    /piper/probe_pushability/_action/feedback \
    /piper/probe_pushability/_action/status \
    >"${BAG_LOG}" 2>&1 &
bag_pid=$!

ros2 run obstacle_traversal acceptance_recorder --ros-args \
    -p scenario:="${SCENARIO}" -p output_path:="${EVENTS}" \
    >"${RECORDER_LOG}" 2>&1 &
recorder_pid=$!

wait_ready() {
    local topic="$1"
    local message_type="$2"
    local attempts=0
    local output=""
    echo "[WAIT] ${topic}"
    while (( attempts < 300 )); do
        # Avoid a stale ros2 daemon and request the same durable QoS used by
        # readiness publishers. Explicit types also make discovery deterministic
        # while the machine is busy loading Isaac Sim and runtime components.
        if output="$(timeout 5s ros2 topic echo --no-daemon --spin-time 3 \
            --once --qos-reliability reliable --qos-durability transient_local \
            "${topic}" "${message_type}" 2>&1)" \
            && rg -q 'data: true|ready: true' <<<"${output}"; then
            echo "[READY] ${topic}"
            return 0
        fi
        if ! kill -0 "${system_pid}" 2>/dev/null; then
            echo "Terminal 1 exited before readiness; inspect ${SYSTEM_LOG}" >&2
            return 1
        fi
        if (( attempts > 0 && attempts % 20 == 0 )) && [[ -n "${output}" ]]; then
            echo "[WAIT] ${topic}: $(tail -n 1 <<<"${output}")" >&2
        fi
        attempts=$((attempts + 1))
        sleep 1
    done
    echo "Readiness timeout: ${topic}" >&2
    return 1
}

if [[ "${CORE_ONLY}" != "true" ]]; then
    wait_ready /semantic_navigation/verification_readiness \
        semantic_navigation_interfaces/msg/SystemReadiness
    wait_ready /semantic_navigation/readiness \
        semantic_navigation_interfaces/msg/SystemReadiness
fi
wait_ready /groundingdino_vlm/ready std_msgs/msg/Bool
wait_ready /llmdecision/ready std_msgs/msg/Bool
wait_ready /obstacle_traversal/ready std_msgs/msg/Bool
wait_ready /piper/yolo_ready std_msgs/msg/Bool

if [[ "${CORE_ONLY}" != "true" ]]; then
    if [[ "${SCENARIO}" == "curtain" ]]; then
        INSTRUCTION="请导航到实体 bathroom_basin_0001"
    else
        INSTRUCTION="请导航到实体 living_room_cabinet_0008"
    fi
    ros2 run obstacle_traversal publish_instruction \
        --min-subscriptions 3 --delivery-wait 2 "${INSTRUCTION}"
fi

wait "${recorder_pid}"
recorder_pid=""
kill -INT "${bag_pid}" 2>/dev/null || true
wait "${bag_pid}" || true
bag_pid=""
kill -INT "${system_pid}" 2>/dev/null || true
wait "${system_pid}" || true
system_pid=""

report_args=(
    --scenario "${SCENARIO}" --events "${EVENTS}"
    --bag "${BAG}" --output "${REPORT}"
)
if [[ "${CORE_ONLY}" == "true" ]]; then
    report_args+=(--core-only)
fi
ros2 run obstacle_traversal acceptance_report "${report_args[@]}"
sha256sum "${EVENTS}" "${REPORT}" "${SYSTEM_LOG}" "${RECORDER_LOG}" "${BAG_LOG}" >"${HASHES}"
echo "Acceptance artifacts: ${RUN_DIR}"
