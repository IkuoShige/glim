#!/bin/bash
set -euo pipefail

# EPCD GLIM Evaluation Script
# Usage: ./evaluate_epcd.sh [config_path] [dump_path]

CONFIG_PATH="${1:-/root/ros2_ws/src/glim/config/epcd_livox_cpu}"
DUMP_PATH="${2:-/tmp/glim_epcd_eval_$(date +%Y%m%d_%H%M%S)}"
GT_PATH="/root/ros2_ws/src/gt/traj_lidar_outdoor_hard_01.txt"
BAG1="/root/ros2_ws/src/outdoor_hard_01a"
BAG2="/root/ros2_ws/src/outdoor_hard_01b"
WS="/root/ros2_ws"
RESULT_JSON="${DUMP_PATH}/eval_result.json"

echo "=========================================="
echo "  EPCD GLIM Evaluation"
echo "=========================================="

# 1. Record CPU info
echo ""
echo "[1/5] Recording CPU info..."
CPU_MODEL=$(lscpu | grep "Model name" | sed 's/.*:\s*//')
CPU_CORES=$(nproc)
CPU_MAX_MHZ=$(lscpu | grep "CPU max MHz" | sed 's/.*:\s*//' || echo "unknown")
echo "  CPU: ${CPU_MODEL}"
echo "  Cores: ${CPU_CORES}"
echo "  Max MHz: ${CPU_MAX_MHZ}"

# 2. Build
echo ""
echo "[2/5] Building glim_ros..."
cd "${WS}"
source /opt/ros/humble/setup.bash 2>/dev/null || true
colcon build --packages-up-to glim_ros --symlink-install \
  --cmake-args -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTING=ON \
  2>&1 | tail -5
source install/setup.bash

# 3. Run unit test
echo ""
echo "[3/5] Running unit test..."
colcon test --packages-select glim --event-handlers console_direct+ \
  --ctest-args -R test_exact_gicp_factor --output-on-failure 2>&1 | tail -5
TEST_RESULT=$(colcon test-result --test-result-base "${WS}/build" 2>&1 | tail -1)
echo "  Test: ${TEST_RESULT}"

# 4. Run EPCD evaluation
echo ""
echo "[4/5] Running EPCD on outdoor_hard_01a/b..."
echo "  Config: ${CONFIG_PATH}"
echo "  Dump: ${DUMP_PATH}"

WALL_START=$(date +%s%N)

ros2 run glim_ros glim_rosbag \
  "${BAG1}" "${BAG2}" \
  --ros-args \
  -p config_path:="${CONFIG_PATH}" \
  -p dump_path:="${DUMP_PATH}" \
  -p auto_quit:=true 2>&1 | tee "${DUMP_PATH}/run.log" | tail -20

WALL_END=$(date +%s%N)
WALL_MS=$(( (WALL_END - WALL_START) / 1000000 ))
WALL_S=$(echo "scale=3; ${WALL_MS} / 1000" | bc)

# Count frames and compute bag span from trajectory
TRAJ_FILE="${DUMP_PATH}/traj_lidar.txt"
if [ ! -f "${TRAJ_FILE}" ]; then
  echo "ERROR: ${TRAJ_FILE} not found"
  exit 1
fi

NUM_FRAMES=$(wc -l < "${TRAJ_FILE}")
FIRST_TS=$(head -1 "${TRAJ_FILE}" | awk '{print $1}')
LAST_TS=$(tail -1 "${TRAJ_FILE}" | awk '{print $1}')
BAG_SPAN=$(echo "${LAST_TS} - ${FIRST_TS}" | bc)
RT_RATIO=$(echo "scale=6; ${BAG_SPAN} / ${WALL_S}" | bc)

echo ""
echo "  Frames: ${NUM_FRAMES}"
echo "  Bag span: ${BAG_SPAN} s"
echo "  Wall time: ${WALL_S} s"
echo "  RT ratio: ${RT_RATIO}x"

# 5. ATE evaluation
echo ""
echo "[5/5] Computing ATE..."
ATE_OUTPUT=$(evo_ape tum "${GT_PATH}" "${TRAJ_FILE}" --align 2>&1)
echo "${ATE_OUTPUT}"

ATE_RMSE=$(echo "${ATE_OUTPUT}" | grep "rmse" | awk '{print $2}')
ATE_MEAN=$(echo "${ATE_OUTPUT}" | grep "mean" | awk '{print $2}')
ATE_MEDIAN=$(echo "${ATE_OUTPUT}" | grep "median" | awk '{print $2}')
ATE_MAX=$(echo "${ATE_OUTPUT}" | grep "max" | awk '{print $2}')

# Write result JSON
cat > "${RESULT_JSON}" <<EOJSON
{
  "timestamp": "$(date -Iseconds)",
  "cpu": {
    "model": "${CPU_MODEL}",
    "cores": ${CPU_CORES},
    "max_mhz": "${CPU_MAX_MHZ}"
  },
  "config_path": "${CONFIG_PATH}",
  "dataset": "outdoor_hard_01a/b",
  "performance": {
    "num_frames": ${NUM_FRAMES},
    "bag_span_s": ${BAG_SPAN},
    "wall_time_s": ${WALL_S},
    "rt_ratio": ${RT_RATIO}
  },
  "accuracy": {
    "ate_rmse_m": ${ATE_RMSE},
    "ate_mean_m": ${ATE_MEAN},
    "ate_median_m": ${ATE_MEDIAN},
    "ate_max_m": ${ATE_MAX}
  },
  "test_result": "${TEST_RESULT}"
}
EOJSON

echo ""
echo "=========================================="
echo "  Summary"
echo "=========================================="
echo "  RT ratio:  ${RT_RATIO}x"
echo "  ATE RMSE:  ${ATE_RMSE} m"
echo "  ATE mean:  ${ATE_MEAN} m"
echo "  Wall time: ${WALL_S} s"
echo "  Result:    ${RESULT_JSON}"
echo "=========================================="
