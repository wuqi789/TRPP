#!/usr/bin/env bash
set -Eeo pipefail
umask 077

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
isaacsim_path=${ISAACSIM_PATH:-}
isaac_python=${ISAACSIM_PYTHON_EXE:-${isaacsim_path:+$isaacsim_path/python.sh}}
export SCOUT_WORKSPACE="${workspace_dir}"
cpu_cores=${ISAAC_CPU_CORES:-12}
full_room=true
headless=false
debug_lidar=false
scan_topic=/scan
start_x=7.7
start_y=-4.0
start_yaw=1.0
arm_mount_x=-0.05
arm_mount_y=0.0
arm_mount_z=0.13

while (($#)); do
  case "$1" in
    --full-room) full_room=true; shift ;;
    --navigation-room) full_room=false; shift ;;
    --headless) headless=true; shift ;;
    --debug-lidar) debug_lidar=true; shift ;;
    --scan-topic) scan_topic=${2:?--scan-topic requires a value}; shift 2 ;;
    --start-x) start_x=${2:?--start-x requires a value}; shift 2 ;;
    --start-y) start_y=${2:?--start-y requires a value}; shift 2 ;;
    --start-yaw) start_yaw=${2:?--start-yaw requires a value}; shift 2 ;;
    --arm-mount-x) arm_mount_x=${2:?--arm-mount-x requires a value}; shift 2 ;;
    --arm-mount-y) arm_mount_y=${2:?--arm-mount-y requires a value}; shift 2 ;;
    --arm-mount-z) arm_mount_z=${2:?--arm-mount-z requires a value}; shift 2 ;;
    *) echo "Unsupported Isaac launch argument: $1" >&2; exit 2 ;;
  esac
done

source /opt/ros/humble/setup.bash
if [[ -z "$isaac_python" || ! -x "$isaac_python" ]]; then
  echo "Isaac Sim Python not found: $isaac_python" >&2
  exit 1
fi
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Workspace not built. Run commands from ISAAC_SIM_NAVIGATION.md first." >&2
  exit 1
fi
source "$workspace_dir/install/setup.bash"
set -u

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_LOCALHOST_ONLY="${SCOUT_ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${workspace_dir}/runtime/ros_logs}"
mkdir -p -- "${ROS_LOG_DIR}"
chmod 700 -- "${ROS_LOG_DIR}"
export PYTHONPATH="$workspace_dir/src/NeuPAN:$workspace_dir/.deps/python${PYTHONPATH:+:$PYTHONPATH}"

available_cores=$(nproc)
if (( cpu_cores < 1 )); then
  cpu_cores=1
elif (( cpu_cores > available_cores )); then
  cpu_cores=$available_cores
fi

exec ros2 launch neupan_ros2 isaac_sim.launch.py \
  workspace_dir:="$workspace_dir" \
  isaac_python:="$isaac_python" \
  cpu_cores:="$cpu_cores" \
  full_room:="$full_room" \
  headless:="$headless" \
  debug_lidar:="$debug_lidar" \
  scan_topic:="$scan_topic" \
  start_x:="$start_x" \
  start_y:="$start_y" \
  start_yaw:="$start_yaw" \
  arm_mount_x:="$arm_mount_x" \
  arm_mount_y:="$arm_mount_y" \
  arm_mount_z:="$arm_mount_z"
