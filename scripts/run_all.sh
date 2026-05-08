#!/bin/bash
# Run all processes of Hybrid A* System
# Processes:
#   1. Gazebo Bridge (ROS → SHM)
#   2. Planner (SHM → SHM)
#   3. Tracker (SHM → SHM)
#
# Note: Gazebo + roscore must be running first.

set -e

BUILD_DIR="$(cd "$(dirname "$0")/.." && pwd)/build_new"
LOG_DIR="/tmp/hybrid_astar_logs"
mkdir -p "${LOG_DIR}"

cleanup() {
    echo ""
    echo "=== Shutting down all processes ==="
    kill ${BRIDGE_PID} ${PLANNER_PID} ${TRACKER_PID} 2>/dev/null || true
    wait ${BRIDGE_PID} ${PLANNER_PID} ${TRACKER_PID} 2>/dev/null || true
    echo "All processes stopped."
}

trap cleanup EXIT INT TERM

echo "=== Starting Hybrid A* System ==="

# 1. Gazebo Bridge (needs ROS)
echo "[1/3] Starting Gazebo Bridge..."
"${BUILD_DIR}/src/gazebo_bridge/run_gazebo_bridge" \
    > "${LOG_DIR}/bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "  Bridge PID: ${BRIDGE_PID}"

sleep 2

# 2. Planner
echo "[2/3] Starting Planner..."
"${BUILD_DIR}/src/planner/run_planner" \
    > "${LOG_DIR}/planner.log" 2>&1 &
PLANNER_PID=$!
echo "  Planner PID: ${PLANNER_PID}"

sleep 1

# 3. Tracker
echo "[3/3] Starting Tracker..."
"${BUILD_DIR}/src/tracker/run_tracker" \
    > "${LOG_DIR}/tracker.log" 2>&1 &
TRACKER_PID=$!
echo "  Tracker PID: ${TRACKER_PID}"

echo ""
echo "All processes running. Logs in ${LOG_DIR}/"
echo "Press Ctrl+C to stop."

# Wait for any to exit
wait -n ${BRIDGE_PID} ${PLANNER_PID} ${TRACKER_PID} 2>/dev/null || true
