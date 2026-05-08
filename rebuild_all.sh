#!/bin/bash
# Hybrid A* 全系统构建脚本
# 同时构建 catkin ROS 包和新 CMake 系统
set -e

WS=$(dirname "$(readlink -f "$0")")

echo "============================================"
echo " Hybrid A* 全系统构建"
echo "============================================"

# Source ROS
source /opt/ros/noetic/setup.bash

# 1. 构建 catkin ROS 包 (Hybrid_A_Star)
echo ""
echo "[1/2] 构建 catkin ROS 包..."
cd "$WS"
# 暂时移开根 CMakeLists.txt 避免干扰 catkin_make
if [ -f CMakeLists.txt ]; then
    mv CMakeLists.txt CMakeLists_new.txt
fi
catkin_make
if [ -f CMakeLists_new.txt ]; then
    mv CMakeLists_new.txt CMakeLists.txt
fi

# 2. 构建新 CMake 系统 (planner, tracker, gazebo_bridge)
echo ""
echo "[2/2] 构建新 CMake 系统..."
mkdir -p "$WS/build_new"
cd "$WS/build_new"
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 3. 复制新可执行文件到 ROS devel 空间
echo ""
echo "[3/3] 安装可执行文件..."
cp -v src/planner/run_planner "$WS/devel/lib/hybrid_a_star/"
cp -v src/tracker/run_tracker "$WS/devel/lib/hybrid_a_star/"
cp -v src/gazebo_bridge/run_gazebo_bridge "$WS/devel/lib/hybrid_a_star/"

echo ""
echo "============================================"
echo " 构建完成！"
echo ""
echo " 使用方式："
echo "   source $WS/devel/setup.bash"
echo "   roslaunch hybrid_a_star run_hybrid_a_star.launch   (仅规划，无 Gazebo)"
echo "   roslaunch hybrid_a_star full_system.launch          (全系统 + Gazebo)"
echo "============================================"
