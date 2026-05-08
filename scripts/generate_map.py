#!/usr/bin/env python3
"""生成与 Gazebo test.world 匹配的 occupancy grid 地图"""
import numpy as np
from PIL import Image
import yaml

# 地图参数
RESOLUTION = 0.5  # m/pixel (更精细)
WIDTH = 120       # 60m
HEIGHT = 80       # 40m
ORIGIN_X = -5.0   # 世界坐标系原点对应网格(0,0)
ORIGIN_Y = -15.0

# 创建空地（0 = free）
grid = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

def world_to_grid(wx, wy):
    """世界坐标 → 网格索引"""
    gx = int((wx - ORIGIN_X) / RESOLUTION)
    gy = int((wy - ORIGIN_Y) / RESOLUTION)
    return gx, gy

def fill_rect(cx, cy, w, h):
    """在网格上填充矩形障碍物"""
    x0, y0 = world_to_grid(cx - w/2, cy - h/2)
    x1, y1 = world_to_grid(cx + w/2, cy + h/2)
    x0, x1 = max(0, min(x0, x1)), min(WIDTH, max(x0, x1))
    y0, y1 = max(0, min(y0, y1)), min(HEIGHT, max(y0, y1))
    grid[y0:y1, x0:x1] = 100

# Gazebo test.world 中的障碍物
fill_rect(5, 0, 1, 1)      # obstacle_1
fill_rect(8, 2, 1, 1)      # obstacle_2
fill_rect(8, -2, 1, 1)     # obstacle_3
fill_rect(10, 5, 20, 0.3)  # wall_top
fill_rect(10, -5, 20, 0.3) # wall_bottom

# 保存 PGM
img = Image.fromarray(grid, mode='L')
img.save('/home/xiaoyubb/hybrid_a_star/src/Hybrid_A_Star/maps/map_gazebo.pgm')

# 保存 YAML
yaml_content = {
    'image': 'map_gazebo.pgm',
    'resolution': RESOLUTION,
    'origin': [ORIGIN_X, ORIGIN_Y, 0.0],
    'occupied_thresh': 0.1,
    'free_thresh': 0.05,
    'negate': 0,
}
with open('/home/xiaoyubb/hybrid_a_star/src/Hybrid_A_Star/maps/map_gazebo.yaml', 'w') as f:
    yaml.dump(yaml_content, f, default_flow_style=False)

print(f"Generated map_gazebo.pgm ({WIDTH}x{HEIGHT}) and map_gazebo.yaml")
