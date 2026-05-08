#!/usr/bin/env python3
"""
SHM 测试数据提供者：使用 ctypes 确保与 C++ struct 布局完全一致。
为 Planner + Tracker 提供模拟的车辆状态、目标位姿和地图数据。
"""
import ctypes
import mmap
import os
import time
import math
import signal
import sys

# ============================================================
# C++ struct 对应的 ctypes 定义（天然匹配 C 内存布局）
# ============================================================

class VehicleStateData(ctypes.Structure):
    _fields_ = [
        ("is_valid", ctypes.c_uint8),
        ("sequence", ctypes.c_uint64),
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("theta", ctypes.c_double),
        ("linear_velocity", ctypes.c_double),
        ("angular_velocity", ctypes.c_double),
        ("steering_angle", ctypes.c_double),
        ("timestamp_ns", ctypes.c_uint64),
    ]

class GoalPoseData(ctypes.Structure):
    _fields_ = [
        ("is_valid", ctypes.c_uint8),
        ("sequence", ctypes.c_uint64),
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("theta", ctypes.c_double),
        ("timestamp_ns", ctypes.c_uint64),
    ]

class MapShmHeader(ctypes.Structure):
    _fields_ = [
        ("is_valid", ctypes.c_uint8),
        ("sequence", ctypes.c_uint64),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("resolution", ctypes.c_double),
        ("origin_x", ctypes.c_double),
        ("origin_y", ctypes.c_double),
        ("timestamp_ns", ctypes.c_uint64),
    ]

SHM_VEHICLE_STATE = "/hy_astar_vehicle_state"
SHM_GOAL_POSE = "/hy_astar_goal_pose"
SHM_MAP_DATA = "/hy_astar_map_data"

MAP_W, MAP_H = 200, 200
MAP_RES = 0.1

running = True

def cleanup():
    for name in [SHM_VEHICLE_STATE, SHM_GOAL_POSE, SHM_MAP_DATA]:
        try:
            os.unlink(f"/dev/shm{name}")
        except OSError:
            pass

def signal_handler(sig, frame):
    global running
    running = False

def main():
    global running
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    cleanup()

    # ---- 创建 vehicle_state SHM ----
    vs_size = ctypes.sizeof(VehicleStateData)
    fd = os.open(f"/dev/shm{SHM_VEHICLE_STATE}", os.O_CREAT | os.O_RDWR)
    os.ftruncate(fd, vs_size)
    state_mm = mmap.mmap(fd, vs_size, mmap.MAP_SHARED, mmap.PROT_WRITE)
    os.close(fd)
    print(f"VehicleStateData size: {vs_size} bytes")

    # ---- 创建 goal_pose SHM ----
    gp_size = ctypes.sizeof(GoalPoseData)
    fd = os.open(f"/dev/shm{SHM_GOAL_POSE}", os.O_CREAT | os.O_RDWR)
    os.ftruncate(fd, gp_size)
    goal_mm = mmap.mmap(fd, gp_size, mmap.MAP_SHARED, mmap.PROT_WRITE)
    os.close(fd)
    print(f"GoalPoseData size: {gp_size} bytes")

    # ---- 创建 map SHM ----
    header_size = ctypes.sizeof(MapShmHeader)
    map_shm_size = header_size + MAP_W * MAP_H
    fd = os.open(f"/dev/shm{SHM_MAP_DATA}", os.O_CREAT | os.O_RDWR)
    os.ftruncate(fd, map_shm_size)
    map_mm = mmap.mmap(fd, map_shm_size, mmap.MAP_SHARED, mmap.PROT_WRITE)
    os.close(fd)
    print(f"MapShmHeader size: {header_size} bytes, total SHM: {map_shm_size}")

    # ---- 写入地图数据 ----
    map_header = MapShmHeader()
    map_header.is_valid = 1
    map_header.sequence = 1
    map_header.width = MAP_W
    map_header.height = MAP_H
    map_header.resolution = MAP_RES
    map_header.origin_x = -10.0
    map_header.origin_y = -10.0
    map_header.timestamp_ns = 1

    # 用 ctypes memmove 写入 header
    ctypes.memmove(ctypes.addressof(
        ctypes.c_uint8.from_buffer(map_mm, 0)),
        ctypes.addressof(map_header), header_size)
    # 地图数据初始化为 0（全部 free）
    map_mm[header_size:header_size + MAP_W * MAP_H] = b'\x00' * (MAP_W * MAP_H)
    print(f"Map written: {MAP_W}x{MAP_H}, origin=(-10,-10), res={MAP_RES}")

    print("SHM data provider ready. Starting data loop...")

    seq = 0
    while running:
        seq += 1

        # 写入车辆状态
        vs = VehicleStateData()
        vs.is_valid = 1
        vs.sequence = seq
        vs.x = 0.0
        vs.y = 0.0
        vs.theta = 0.0
        vs.linear_velocity = 0.0
        vs.angular_velocity = 0.0
        vs.steering_angle = 0.0
        vs.timestamp_ns = seq

        ctypes.memmove(ctypes.addressof(
            ctypes.c_uint8.from_buffer(state_mm, 0)),
            ctypes.addressof(vs), vs_size)

        # 写入目标位姿（模拟在 15m 前方）
        gp = GoalPoseData()
        gp.is_valid = 1
        gp.sequence = seq
        gp.x = 5.0
        gp.y = 0.0
        gp.theta = 0.0
        gp.timestamp_ns = seq

        ctypes.memmove(ctypes.addressof(
            ctypes.c_uint8.from_buffer(goal_mm, 0)),
            ctypes.addressof(gp), gp_size)

        print(f"[seq={seq}] Vehicle state & goal pose written")
        time.sleep(3)

    cleanup()
    print("Data provider stopped.")

if __name__ == "__main__":
    main()
