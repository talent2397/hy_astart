#!/bin/bash
# 离线测试：不依赖 Gazebo，用模拟数据测试 Planner + Tracker
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/build_new"
LOG_DIR="/tmp/hybrid_astar_test"
mkdir -p "${LOG_DIR}"

cleanup() {
    echo ""
    echo "=== Stopping all test processes ==="
    # 先杀掉子进程
    kill ${PLANNER_PID} ${TRACKER_PID} 2>/dev/null || true
    # 数据提供者会自己清理
    kill ${DATA_PID} 2>/dev/null || true
    wait 2>/dev/null || true
    # 清理 SHM
    rm -f /dev/shm/hy_astar_* 2>/dev/null || true
    echo "Cleanup done."
}

trap cleanup EXIT INT TERM

echo "=== Offline Test Mode ==="
echo "This test creates a simulated Bridge to provide test data"
echo ""

# 清理残留 SHM
rm -f /dev/shm/hy_astar_* 2>/dev/null || true

# 1. 启动 SHM 数据提供者
echo "[1/4] Starting SHM test data provider..."
python3 "${SCRIPT_DIR}/test_data_provider.py" &
DATA_PID=$!
sleep 2

# 2. 启动 Planner
echo "[2/4] Starting Planner..."
"${BUILD_DIR}/src/planner/run_planner" > "${LOG_DIR}/planner.log" 2>&1 &
PLANNER_PID=$!
echo "  Planner PID: ${PLANNER_PID}"

sleep 3

# 3. 启动 Tracker
echo "[3/4] Starting Tracker..."
"${BUILD_DIR}/src/tracker/run_tracker" > "${LOG_DIR}/tracker.log" 2>&1 &
TRACKER_PID=$!
echo "  Tracker PID: ${TRACKER_PID}"

echo ""
echo "=== All processes started ==="
echo "Logs: ${LOG_DIR}/"
echo "Press Ctrl+C to stop"
echo ""

# 4. 等待并监控
sleep 8

echo ""
echo "--- Planner log (last 15 lines) ---"
tail -15 "${LOG_DIR}/planner.log"

echo ""
echo "--- Tracker log (last 15 lines) ---"
tail -15 "${LOG_DIR}/tracker.log"

echo ""
echo "=== System running... Press Ctrl+C to stop ==="
# 等待数据提供者进程（或用户 Ctrl+C）
wait ${DATA_PID} 2>/dev/null || true
