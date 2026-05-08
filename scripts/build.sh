#!/bin/bash
# Build script for Hybrid A* System (new modules only)
# Usage: ./scripts/build.sh [Release|Debug]

set -e

BUILD_TYPE=${1:-Release}
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${PROJECT_DIR}/build_new"

echo "=== Building Hybrid A* System ==="
echo "Project:  ${PROJECT_DIR}"
echo "Build:    ${BUILD_DIR}"
echo "Type:     ${BUILD_TYPE}"

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure
cmake "${PROJECT_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Build
make -j$(nproc)

echo ""
echo "=== Build Complete ==="
echo ""
echo "Binaries:"
echo "  ${BUILD_DIR}/src/planner/run_planner"
echo "  ${BUILD_DIR}/src/tracker/run_tracker"
echo "  ${BUILD_DIR}/src/gazebo_bridge/run_gazebo_bridge"
echo ""
echo "To run the full system:"
echo "  1. source /opt/ros/<distro>/setup.bash"
echo "  2. roslaunch hybrid_a_star full_system.launch"
echo ""
echo "Or run manually:"
echo "  ./scripts/run_all.sh"
