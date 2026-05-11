================================================================
 Hybrid A* 系统重构 — 开发日志
 最后更新: 2026-05-11 15:00
================================================================

================================================================
一、重构进度总览
================================================================

✅ 已完成 (框架层):
  ✅ 多进程架构 (3 进程: gazebo_bridge / planner / tracker)
  ✅ 共享内存 IPC (POSIX shm_open + mmap, sequence 同步)
  ✅ 消息队列 (POSIX mq, PATH_READY / REPLAN_REQUEST)
  ✅ Gazebo 仿真环境 (test.world + 4轮 diff_drive 车辆, 差速转弯已验证)
  ✅ Gazebo Bridge (ROS ↔ SHM 双向桥接, /gazebo/model_states 真实位姿, TF 发布)
  ✅ Hybrid A* Planner (接口化 IPlanner, RS 曲线捷径, 支持 initialpose 起点)
  ✅ Pure Pursuit Tracker (接口化 ITracker, 自适应前视+曲率减速+转向平滑)
  ✅ 地图匹配 (generate_map.py, Y轴翻转已修复)
  ✅ RViz 路径可视化 (/planned_path, 绿色规划线)
  ✅ 初始位姿传送 (/initialpose → Gazebo set_model_state)
  ✅ 一键构建部署 (rebuild_all.sh)
  ✅ 全链路跑通: RViz → Planner → Tracker → Gazebo, 车能沿路径转弯

🔄 进行中 (算法层):
  🔄 Pure Pursuit 参数调优 (前视距离/弯道速度/转向响应)
  🔄 路径跟踪精度提升 (减少偏离规划路径的偏差)

❌ 待实现 (按 xuqiu.md):
  ❌ MPC 控制器 (替换 Pure Pursuit)
  ❌ 状态估计器 EKF
  ❌ Unix Domain Socket 配置通道
  ❌ YAML 配置加载
  ❌ 动态障碍物处理
  ❌ 在线重规划

📊 当前阶段判断:
  框架已搭建完成, 全链路通。剩余工作主要是规控算法改进:
  - Pure Pursuit → MPC (性能提升)
  - 参数系统化 (YAML 配置)
  - 高级功能 (动态障碍物, 重规划)

================================================================
二、完整 Bug 修复清单 (按时间线)
================================================================

Bug #1 | Gazebo 崩溃 (Segfault)
  时间:   2026-05-08 19:17
  现象:   Gazebo 启动立即崩溃
  根因:   1) test.world 中 <light name="sun"> 与 model://sun 内的 light 重名
          2) skid_steer_drive 插件引用 front_left_wheel (link 名) 而非
             front_left_steering_joint (joint 名)
  修复:   - 移除 test.world 重复的 light 定义
          - 改用 diff_drive 插件驱动后轮, 移除 gazebo_ros_control
          - diff_drive 不使用 /car 命名空间 → topic 为 /odom, /cmd_vel

Bug #2 | 进程死锁 (互相等待)
  时间:   2026-05-08 19:17
  现象:   各进程启动后卡住, 无任何输出
  根因:   bridge 启动时阻塞等待 control_cmd SHM, 但 control_cmd 由 tracker 创建,
          tracker 又在等 bridge 的 vehicle_state
  修复:   bridge 改为非阻塞模式, 进入 ControlLoop 后再轮询打开 control_cmd SHM
  文件:   src/gazebo_bridge/app/run_gazebo_bridge.cpp

Bug #3 | TF 缺失 world→odom
  时间:   2026-05-08 19:17
  现象:   RViz 中 RobotModel 不显示
  根因:   diff_drive 发布 odom→base_link, 但没有 world→odom 连接
  修复:   launch 文件添加 static_transform_publisher world→odom
  文件:   src/Hybrid_A_Star/launch/full_system.launch

Bug #4 | DataBusWriter::Write 覆盖 sequence
  时间:   2026-05-08 19:53
  现象:   bridge OdomCallback 持续写入 SHM (write_ok=1),
          planner 永远读到 sequence=1, ReadLatest 始终返回 false
  根因:   Write() 中 memcpy(source→SHM) 把 source 的 sequence=0 复制进去,
          然后 sequence++ 永远 0→1
  修复:   先保存 SHM 中的 old_sequence, memcpy 后恢复 old_sequence+1,
          写入期间设置 is_valid=0
  文件:   src/ipc/include/ipc/data_bus.h

Bug #5 | PGM 地图 negate — 全图被判为障碍物
  时间:   2026-05-08 20:52
  现象:   GetNeighborNodes 返回 0, 搜索无法展开
  根因:   map_gazebo.pgm 中 free=0 (黑色), ROS map_server 默认黑色=occupied
  修复:   map_gazebo.yaml 设置 negate: 1
  文件:   src/Hybrid_A_Star/maps/map_gazebo.yaml

Bug #6 | Pure Pursuit 控制抖动 + 超速
  时间:   2026-05-08 21:49
  现象:   车乱动, 完全不按规划路径走
  根因:   a) 转向角逐帧振荡 (-0.06 ↔ -0.003), FindClosestPoint 找的最近点每帧不同
          b) 速度始终 2.0 m/s 满速, 弯道不减速
          c) 无转向速率限制, 帧间 steering 可突变
  修复:   a) 转向低通滤波 0.3*new + 0.7*old + 速率限制 0.5 rad/s
          b) 曲率自适应速度: desired = v_target / (1 + 8*|curvature|)
          c) 自适应前视距离: Ld = 3.0 + 2.0*|v|
  文件:   src/tracker/src/pure_pursuit_tracker.cpp
          src/tracker/include/tracker/pure_pursuit_tracker.h

Bug #7 | 车辆模型: 质点→运动学 (planar_move→diff_drive)
  时间:   2026-05-08 21:49
  根因:   planar_move 是质点模型, 无车轮物理, 不够真实
  修复:   两轮 diff_drive, 50kg, 关节阻尼 5.0, 轮子高摩擦 mu=1.0, 扭矩 10Nm
  文件:   src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro

Bug #8 | diff_drive 关节轴方向反了 → 车倒着开
  时间:   2026-05-08 22:16
  现象:   cmd_vel=+2.0, 但 odom 速度全为负, 车从 x=0 退到 x=-6,
          冲出地图边界 → planner "0 neighbors" → 规划失败
  根因:   wheel joint axis="0 1 0" 导致 diff_drive 计算轮子转向与实际相反
  修复:   axis xyz="0 1 0" → axis xyz="0 -1 0" (两个轮子)
  文件:   src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro

Bug #9 | 二进制未部署 → roslaunch 运行旧代码
  时间:   2026-05-08 22:16
  现象:   代码改了日志清理, 但运行后 Odom/CMD 日志照样打印
  根因:   make 编译到 build_new/, 但 roslaunch 从 devel/lib/hybrid_a_star/ 加载
  修复:   1) make 后自动 cp 到 devel 空间
          2) rebuild_all.sh 统一使用 build_new 目录
  文件:   rebuild_all.sh

================================================================
三、当前已知余留问题
================================================================

1. TF_REPEATED_DATA 警告 (无害)
   原因: diff_drive 以 100Hz 发布 odom→base_link TF, 与 Gazebo 内部发布冲突
   影响: 终端有 WARN 输出, 不影响功能
   方案: 可在 diff_drive 插件设置 <publishTF>false</publishTF>

2. RViz 中看不到规划路径线
   原因: 新架构 Planner/Tracker 无 ROS 依赖, 不发布 /searched_path 等 topic
   方案: 在 gazebo_bridge 中加一个 ROS publisher 转发路径到 RViz

3. 纯跟踪 (Pure Pursuit) 无 MPC 预测能力
   方案: 后续替换为 MPC 控制器 (按 xuqiu.md 阶段四)

================================================================
四、修改过的文件总览
================================================================

IPC 层:
  src/ipc/include/ipc/data_bus.h              (Write bug 修复)
  src/ipc/include/ipc/data_protocol.h         (INITIAL_POSE 通道新增)

Planner:
  src/planner/app/run_planner.cpp             (initialpose 起点支持)
  src/planner/src/hybrid_a_star.cpp           (搜索核心)
  src/planner/src/hybrid_a_star_planner.cpp   (碰撞尺寸 2.0×1.0)

Tracker:
  src/tracker/include/tracker/pure_pursuit_tracker.h  (Pure Pursuit 接口)
  src/tracker/src/pure_pursuit_tracker.cpp            (Pure Pursuit 完整实现+参数调优)
  src/tracker/app/run_tracker.cpp                     (配置参数)

Bridge:
  src/gazebo_bridge/app/run_gazebo_bridge.cpp   (Gazebo真值位姿+TF发布+速度投影+
                                                  last_steering+路径可视化+cmd_vel每帧)

模型 & 启动:
  src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro    (4轮重写: 轴/扭矩/命名空间/rpy)
  src/Hybrid_A_Star/vehicle_model/worlds/test.world      (light 修复)
  src/Hybrid_A_Star/launch/gazebo_simulation.launch       (joint_state_pub 移除)
  src/Hybrid_A_Star/launch/full_system.launch             (TF+地图+延迟启动)
  src/Hybrid_A_Star/maps/map_gazebo.yaml                 (negate: 1)
  src/Hybrid_A_Star/rviz/display_hybrid_astar.rviz        (Path→/planned_path)

脚本:
  scripts/generate_map.py     (生成匹配地图+Y轴翻转修复)
  rebuild_all.sh              (一键构建+部署)
  rizhi.md                    (本文件)

================================================================
五、启动流程
================================================================

  source devel/setup.bash
  roslaunch hybrid_a_star full_system.launch

  进程启动顺序:
    1. roscore
    2. map_server (map_gazebo.yaml)
    3. Gazebo + spawn car (test.world + diff_drive)
    4. robot_state_publisher
    5. gazebo_bridge   (ROS→SHM, 延迟1s)
    6. planner         (SHM→SHM, 延迟2s)
    7. tracker         (SHM→SHM, 延迟2s)
    8. RViz

  数据流:
    Gazebo → /odom → Bridge → SHM/vehicle_state → Planner
                                                  → Tracker
    RViz   → /goal → Bridge → SHM/goal_pose     → Planner
    Planner → SHM/planner_path → Tracker
    Tracker → SHM/control_cmd  → Bridge → /cmd_vel → Gazebo


[2026-05-08 22:30] 第五次修复 — RViz 起点无效 + 速度慢 + 启动延迟高
----------------------------------------------------------------
问题1: RViz "2D Pose Estimate" 设置了起点但不生效
  原因: Bridge 未订阅 /initialpose 话题, Planner 总是用车辆当前 odom 位置作为起点
  修复: 1) data_protocol.h 新增 SHM_INITIAL_POSE 通道
        2) Bridge 订阅 /initialpose, 写入 SHM_INITIAL_POSE
        3) Planner 读取 initialpose, 若有效则覆盖起点
  文件: src/ipc/include/ipc/data_protocol.h
        src/gazebo_bridge/app/run_gazebo_bridge.cpp
        src/planner/app/run_planner.cpp

问题2: 小车运行速度慢 (target_velocity=1.0 m/s)
  修复: tracker target_velocity: 1.0 → 2.0 m/s
        bridge 最大速度上限: 2.0 → 3.0 m/s

问题3: 进程启动延迟过高 (bridge 3s, planner/tracker 5s)
  原因: SHM 创建几乎瞬时完成, 3-5 秒等待过长
  修复: bridge sleep 1s, planner/tracker sleep 2s
  文件: src/Hybrid_A_Star/launch/full_system.launch


[2026-05-09 13:00] 第六次修复 — 车速慢 + RViz 初始位姿不生效 + 地图脚本 bug
----------------------------------------------------------------
Bug #10 | 小车速度极慢 (~0.5 m/s) 且改 target_velocity 无效
  时间:   2026-05-09
  现象:   tracker 设 target_velocity=2.0 m/s, bridge 上限 3.0 m/s,
          但小车实际只能跑到 ~0.5 m/s
  根因:   轮子关节阻尼 damping=5.0 远高于电机扭矩 torque=10 Nm。
          实现 2.0 m/s 需要轮子角速度 8 rad/s, 阻尼力矩 = 5.0×8 = 40 Nm,
          但电机仅提供 10 Nm → 最大速度 = 10/5 × 0.25 = 0.5 m/s
  修复:   damping 5.0 → 1.0, friction 0.5 → 0.1
          此时阻尼力矩 1.0×8 = 8 Nm < 10 Nm → 可达到 2.0+ m/s
  文件:   src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro

Bug #11 | RViz "2D Pose Estimate" 设置起点但小车不传送
  时间:   2026-05-09
  现象:   用户在 RViz 设 2D Pose Estimate, Planner 用了该位置作为规划起点,
          但 Gazebo 中小车物理位置未变, Tracker 按真实 odom 跟踪路径,
          导致小车不按预期的初始位姿运动
  根因:   Bug #5 的修复只让 Planner 读取 initialpose 作为起点,
          但没有传送 Gazebo 中的小车模型
  修复:   Bridge InitPoseCallback 中调用 /gazebo/set_model_state 服务
          将小车模型传送到 RViz 指定的位姿
  文件:   src/gazebo_bridge/app/run_gazebo_bridge.cpp

Bug #12 | generate_map.py 写 negate: 0 而非 negate: 1
  时间:   2026-05-09
  现象:   重新运行 generate_map.py 生成的地图全部被判为障碍物
  根因:   脚本写 negate: 0, 但 map_gazebo.yaml 手动改为 negate: 1 才正确
          若重建地图会复现 Bug #5 (全图障碍物)
  修复:   脚本中 negate: 0 → negate: 1, 与 map_gazebo.yaml 一致
  文件:   scripts/generate_map.py


[2026-05-09 13:45] 第七次修复 — 4轮URDF + ROS路径可视化 + 碰撞形状修正
----------------------------------------------------------------
Bug #13 | URDF 只有 2 个驱动轮，无前支撑轮 → 车身拖地 + 速度被额外摩擦拖慢
  时间:   2026-05-09
  现象:   小车看起来只有 2 个轮子，车身前端容易擦地
  修复:   重写 URDF: 4 轮布局
          - 后轮 (rear_left/right_wheel): 驱动轮, diff_drive 控制, radius=0.25
          - 前轮 (front_left/right_wheel): 从动轮, 纯支撑, radius=0.2
          - 轮距 1.2m (wheelSeparation), 轴距 1.0m
          - 后轮 damping 0.5 (降低阻力), torque 20Nm (双倍扭矩)
          - 轮子 cylinder 增加 rpy 旋转使视觉正确
  文件:   src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro

Bug #14 | 无 ROS 路径可视化 → RViz 看不到规划路径
  时间:   2026-05-09
  现象:   新架构 Planner/Tracker 脱离 ROS, RViz 中看不到 /planned_path
  修复:   gazebo_bridge 新增:
          1) 读取 SHM planner_path (直接指针, 避免 1.6MB 拷贝)
          2) 转换为 nav_msgs::Path 发布到 /planned_path topic
          3) RViz 添加 Path display 即可看到规划路径
  文件:   src/gazebo_bridge/app/run_gazebo_bridge.cpp

Bug #15 | 碰撞检测车辆尺寸过大 (4.7x2.0m) 不匹配实际小车 (2.0x1.0m)
  时间:   2026-05-09
  现象:   Planner 用 4.7m 长车做碰撞检测, 规划过于保守
  修复:   SetVehicleShape(4.7, 2.0, 1.3) → SetVehicleShape(2.0, 1.0, 0.5)
  文件:   src/planner/src/hybrid_a_star_planner.cpp


[2026-05-09 14:00] 第八次修复 — 倒车 + 坐标系偏移
----------------------------------------------------------------
Bug #16 | 车倒着跑 (新 URDF 后轮轴方向反了)
  时间:   2026-05-09
  根因:   旧 URDF 轮子在 x=0 (车身中心), Bug #8 改了轴为 "0 -1 0" 正向。
          新 URDF 后轮在 x=-0.5 (尾部), 力矩方向变了, 旧轴方向导致倒车。
  修复:   后轮驱动轴 "0 -1 0" → "0 1 0", 前轮从动轴同样改为 "0 1 0"
  文件:   src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro

Bug #17 | RViz 与 Gazebo 坐标系有固定偏移
  时间:   2026-05-09
  根因:   diff_drive 插件通过积分轮子转速推算 odometry (odometrySource 默认 "encoder")。
          车 spawn 时落地的瞬间轮子微动 → 初始 odometry 与 Gazebo 真实位姿产生固定偏移。
  修复:   diff_drive 插件加 <odometrySource>world</odometrySource>,
          直接从 Gazebo 世界坐标系读取真实位姿作为 odometry 输出,
          消除积分初始误差和长期漂移。
  文件:   src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro


[2026-05-09 14:15] 第九次修复 — RViz/Gazebo 坐标系偏移 (方案设计)
----------------------------------------------------------------
Bug #18 | RViz 与 Gazebo 坐标始终有固定偏移, set_model_state 后不消失
  时间:   2026-05-09
  根因:   odometrySource=world 参数在当前 gazebo_ros_diff_drive 版本不支持,
          被静默忽略。diff_drive 仍用轮子积分推位置, set_model_state 传送
          小车后 odometry 不变 → RViz 显示旧位置。
  方案:   彻底绕过 diff_drive 的 odometry:
          1) Bridge 订阅 /gazebo/model_states 获取 Gazebo 真实位姿
          2) Bridge 自己发布 odom→base_link TF (用真实位姿)
          3) diff_drive 设 <publishTf>false</publishTf> 防止 TF 冲突
          4) 真实位姿写入 SHM vehicle_state (替代 /odom 回调)
  ⚠️ 注意: 以上方案当时仅记录在 rizhi.txt, 实际代码未修改!
          实际代码仍用 /odom (diff_drive 轮子积分) 作为 vehicle_state 来源,
          diff_drive 仍 publishOdomTF=true, Bug #18 实际未修复。

[2026-05-11 10:30] 第十次修复 — 实施 Bug #18 方案 (Gazebo 真实位姿替代轮子里程计)
----------------------------------------------------------------
Bug #19 | 实施 Bug #18 的修复方案: Gazebo/RViz 坐标系对齐
  时间:   2026-05-11
  现象:   Gazebo 中小车在走廊正中间 (y=0), 但 RViz 中小车显示在很远的位置,
          与 Gazebo 实际位置不一致
  根因:   Bug #18 的修复方案只在 rizhi.txt 中记录但从未写入代码。
          当前代码仍用 diff_drive 的 /odom (轮子积分里程计) 作为车辆位姿来源。
          diff_drive 通过积分轮子转速推算 odometry, set_model_state 传送
          小车后轮子未转动 → 积分里程计不更新 → RViz 通过 TF 显示旧位置。
          同时 diff_drive publishOdomTF=true 发布的 odom→base_link TF
          也是积分里程计, 与 Gazebo 真实位姿存在偏差。
  修复:   1) Bridge 新增订阅 /gazebo/model_states (替代 /odom)
             - GazeboModelStatesCallback 遍历 model name 找到 "car"
             - 用 Gazebo 真实 pose 写入 SHM vehicle_state
             - 用 Gazebo 真实 pose 发布 odom→base_link TF
          2) Bridge 新增 tf::TransformBroadcaster (发布 odom→base_link)
          3) car.urdf.xacro: publishOdomTF true → false
             (防止 diff_drive 与 Bridge 同时发布 odom→base_link 造成冲突)
          4) Bridge 不再订阅 /odom (OdomCallback 替换为 GazeboModelStatesCallback)
  TF 链路:
    RViz (fixed: map) → world (static identity) → odom (static identity)
    → base_link (Bridge TF: Gazebo 真实位姿)
  SHM 数据流:
    Gazebo → /gazebo/model_states → Bridge → SHM/vehicle_state → Planner / Tracker
  文件:   src/gazebo_bridge/app/run_gazebo_bridge.cpp
          src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro


[2026-05-11 11:00] 第十一次修复 — 地图 Y 轴翻转 (PGM/ROS 坐标系不匹配)
----------------------------------------------------------------
Bug #20 | generate_map.py 中 numpy 行索引与 PGM/ROS 约定方向相反, 地图上下翻转
  时间:   2026-05-11
  现象:   Bug #19 修复后相对运动正确, 但 Gazebo 中小车在走廊中间 (y=0),
          而 RViz 中小车在下墙下方很远 (y < 4.5)。差异是固定的起始偏移。
          "相对位移是一致的, 只是起始点不一样" — 说明 TF/位姿链路已正确,
          问题出在静态地图本身。
  根因:   generate_map.py 中 world_to_grid() 计算 gy 时直接使用"从底部向上"
          的索引 (gy=0 → 世界 y=-15), 但 numpy 数组保存为 PGM 时
          row=0 对应图像顶部。ROS map_server 加载 PGM 时又会翻转一次
          (PGM row 0 → ROS grid row HEIGHT-1)。两次翻转叠加导致只有
          地图中心的元素位置正确, 离中心越远偏差越大。
          具体: wall_bottom (世界 y=-5) → gy=20 → numpy row 20 (图像上部)
          → PGM row 20 → ROS grid row 59 → 世界 y=14.5 (飞到顶部! 偏差 19.5m)
          wall_top (世界 y=5) → gy=40 → numpy row 40 → 世界 y=4.5 (偏差 0.5m)
          RViz 中两道墙分别在 y=4.5 和 y=14.5, 车在 y=0 在"下墙"下方。
          Gazebo 中墙在 y=5 和 y=-5, 车在 y=0 正好在走廊中间。
  修复:   world_to_grid() 中加一行: gy = HEIGHT - 1 - gy (Y 轴翻转)
          翻转后:
          wall_bottom: gy=59 → 世界 y=-5.0 ✓
          wall_top:   gy=39 → 世界 y=5.0 ✓
          重新运行 python3 scripts/generate_map.py 生成正确地图
  文件:   scripts/generate_map.py
          src/Hybrid_A_Star/maps/map_gazebo.pgm (重新生成)
  ⚠️ 关键教训: numpy row=0 是图像顶部, PGM row=0 也是图像顶部,
     ROS map_server 内部会做 Y 翻转 (PGM top→grid bottom)。
     生成地图时 numpy 行索引必须和 PGM/图像坐标一致 (0=顶部)。


[2026-05-11 11:30] 第十二次修复 — 小车直行不转向 (两个子 Bug)
----------------------------------------------------------------
Bug #21 | Pure Pursuit 跟踪器: 车只沿起始方向直行, 不跟随路径转弯
  时间:   2026-05-11
  现象:   设定起始点和终止点后, 小车朝着起始点的方向一直笔直向前,
          不转向去跟踪规划的路径。路径已成功生成 (89点, 17.6m),
          Tracker 也显示收到路径, 但车始终直行。

  子 Bug #21a — Bridge 丢帧导致 angular_z 被清零
    根因:   Bridge ControlLoop 和 Tracker 各自运行在 100Hz,
            但不同进程的调度时序不完全同步。
            Bridge 通过 ReadLatest(检测 sequence 变化)读取控制指令,
            当 Bridge 在 Tracker 两次写入之间多次读取时, ReadLatest
            返回 false → else 分支强制 angular_z=0 → 转向被打断。
            即使 Tracker 正确计算出转向角, 实际发给 diff_drive 的
            cmd_vel 中 angular.z 大部分时间为 0。
    修复:   Bridge 保存 last_steering (上一次收到的转向角),
            ReadLatest 失败时使用 last_steering 计算 angular_z
            (而非强制清零), 同时缓慢减速。
    文件:   src/gazebo_bridge/app/run_gazebo_bridge.cpp

  子 Bug #21b — FindLookaheadPoint 参考点错误
    根因:   FindLookaheadPoint 使用 path_[closest_idx] 的坐标作为
            距离测量参考点, 而非车辆当前位置。当车辆偏离路径时
            (如起始时有微小偏差), 前视距离测量基准偏移,
            导致找到错误的目标点。
    修复:   FindLookaheadPoint 改为接收 VehicleState 参数,
            使用 state.x/state.y 作为距离测量参考 (标准 Pure Pursuit 做法)。
    文件:   src/tracker/include/tracker/pure_pursuit_tracker.h
            src/tracker/src/pure_pursuit_tracker.cpp

  附:    target_velocity 从 5.0 → 1.5 (5.0 m/s 过高, Bridge 上限仅 3.0)
    文件:   src/tracker/app/run_tracker.cpp


[2026-05-11 12:30] 第十三次修复 — 路径可视化不显示 + 车速衰减
----------------------------------------------------------------
Bug #22 | RViz 中看不到绿色规划路径线 (/planned_path)
  时间:   2026-05-11
  现象:   RViz 已配置 Path display 订阅 /planned_path, 但规划成功后
          不显示绿色路径线。同时小车运行速度非常慢, 一会就停下来。
  根因:   两个独立问题:
          1) g_path_reader.Open(SHM_PLANNER_PATH) 在 main() 中调用,
             此时 Planner 尚未启动 (Bridge 延迟 1s, Planner 延迟 2s),
             SHM 不存在 → Open 静默失败 → Get() 返回 nullptr →
             路径可视化代码中的 path_ptr 永远为 null → 从不发布。
          2) ControlLoop 的 else 分支（ReadLatest 失败时）每帧衰减
             target_vel -= 0.5*dt。Bridge/Tracker 进程不同步导致
             ReadLatest 频繁失败 → 速度被不断衰减 → 车跑一会就停。
  修复:   1) g_path_reader.Open() 移到 ControlLoop 内, 每帧 retry
             直到 Planner 创建 SHM 后连接成功。
          2) else 分支重构: 保存 last_accel, ReadLatest 失败时用
             last_accel 维持速度 (而非盲目减速)。
          3) cmd_vel 和路径可视化每帧无条件执行, 不再依赖 ReadLatest 结果。
          4) target_velocity: 1.5 → 2.0, kp_velocity: 0.5 → 1.0
  文件:   src/gazebo_bridge/app/run_gazebo_bridge.cpp
          src/tracker/app/run_tracker.cpp


[2026-05-11 13:15] 第十四次修复 — 速度反馈取错坐标系分量 (车不转弯的根因)
----------------------------------------------------------------
Bug #23 | Tracker 速度反馈恒为零, 导致车永远直行不转弯
  时间:   2026-05-11
  现象:   设定起点朝北、终点在东后, RViz 规划线有明显弯道,
          但小车仍笔直朝北运动。Debug 显示 steer=0.27 rad (15°)
          计算正确, 但车 θ 始终 1.57 rad (正北), 完全不转。
  根因:   GazeboModelStatesCallback 中:
            state.linear_velocity = twist.linear.x;
          twist.linear.x 是 Gazebo 世界坐标系 X 轴速度分量。
          车朝北 (θ=π/2) 时前进速度全在 Y 轴, X 轴分量为 0。
          → Tracker 永远读到速度≈0
          → 速度 P 控制器始终输出最大加速度 (v_error = target - 0 = target)
          → 但反馈错误导致速度环完全失效
          → angular_z = target_vel * tan(steer) / wheelbase
             target_vel 虽被推到上限, 但 diff_drive 接收的 cmd_vel
             因速度环紊乱导致实际车轮转速异常, 转角无法体现。
          关键证据: 帧#0 到 #99, car.y 从 2.01→3.01 (北移 1m),
          car.θ 始终 1.57 (没转!), v 读数始终 ≈0。
  修复:   将世界坐标系速度投影到车辆前进方向:
            cos_y = cos(state.theta), sin_y = sin(state.theta)
            linear_velocity = vx*cos_y + vy*sin_y
          车朝北时: vx=0, vy=1 → forward_speed = 0*0 + 1*1 = 1 m/s ✓
          车朝东时: vx=1, vy=0 → forward_speed = 1*1 + 0*0 = 1 m/s ✓
  文件:   src/gazebo_bridge/app/run_gazebo_bridge.cpp


[2026-05-11 14:00] 第十五次修复 — diff_drive 差速不生效: 命名空间 + 轮轴方向 + 配置缺失
----------------------------------------------------------------
Bug #24 | diff_drive 插件收到 cmd_vel 后车轮同速旋转, 差速控制完全失效
  时间:   2026-05-11
  现象:   Pure Pursuit 计算正确 (steer=0.27 rad), Bridge 发布 angular.z=-0.36
          正确, 但 joint_states 显示左右后轮转速完全一致 (~4.55 rad/s), 车不转弯。
          隔离测试: 停掉 Bridge/Tracker/Planner, 只开 Gazebo + 手动发 cmd_vel,
          angular.z=0.8 时车依然直行, 两轮同速。→ 确认为 URDF/插件配置问题

  子 Bug #24a — 插件给 topic 自动加了 /car 命名空间
    根因:   diff_drive 插件默认以模型名 "car" 作为 namespace,
            <commandTopic>cmd_vel</commandTopic> 实际变成 /car/cmd_vel,
            Bridge 向 /cmd_vel 发指令, 插件根本没收到。
    修复:   用绝对路径 topic: <commandTopic>/cmd_vel</commandTopic>,
            <odometryTopic>/odom</odometryTopic>

  子 Bug #24b — 左右轮轴必须同向 + 正确方向
    根因:   差速驱动插件假设两轴同向。轴 (0,-1,0) 导致正速度=后退,
            轴 (0, 1,0) 导致正速度=前进。左右轴需同为 (0,1,0)。
    修复:   两后轮轴均设为 (0,1,0)。差速由插件公式保证。
    验证:   torque=100, accel=100, damping=0.1 → 差速轮速差异明显, 车能转弯

  子 Bug #24c — 插件缺少关键参数
    根因:   <alwaysOn>true</alwaysOn> 缺失; <wheelTorque>/<wheelAcceleration> 缺失
    修复:   参照官方 Pioneer 3DX 示例补全所有参数

  子 Bug #24d — 轮子 collision/visual 缺 rpy 旋转
    根因:   圆柱体默认轴沿 Z, 轮子轴应沿 Y。缺 rpy="1.5708 0 0" 导致物理错乱,
            车身翘头、颠簸。
    修复:   所有 4 轮 visual 和 collision 都加上 rpy="1.5708 0 0"

  验证:   roslaunch hybrid_a_star gazebo_simulation.launch
          rostopic pub -r 10 /cmd_vel ... → 车直线前进+转弯 ✓
  文件:   src/Hybrid_A_Star/vehicle_model/urdf/car.urdf.xacro (重写)


[2026-05-11 15:00] 第十六次修复 — Pure Pursuit 参数调优
----------------------------------------------------------------
Bug #25 | 车在弯道转向不足, 偏离路径后速度骤降
  时间:   2026-05-11
  现象:   直线段速度正常, 弯道处转向角不够大, 车无法及时沿路径转弯,
          逐渐偏离规划路径。偏离后曲率增大 → 速度降低 → 更难跟上路径。
  根因:   前视距离过长 (ld = 3 + 2*v, 最大 8m)。车速 2m/s 时 ld=7m,
          车"看到"的弯在 7 米外, 对近处弯道响应严重滞后。
          弯道减速系数太弱 (4.0), 入弯前速度降不下来。
  修复:   1) 缩短前视: base 3.0→1.5, 系数 2.0→1.0, 上限 8.0→4.0
             → 2m/s 时 ld=3.5m (原 7m), 弯道响应快一倍
          2) 加强弯道减速: 4.0→10.0, 弯道降速更激进
          3) 恢复转向速率限制 (3.0 rad/s, 防突变但足够响应)
          4) target_velocity: 2.0, kp_velocity: 1.5
          5) goal_tolerance: 0.2→0.3 (增大判定窗, 避免停不下来)
  效果:   短前视 + 弯道慢速 = 入弯前提早转向, 不易偏离路径
  文件:   src/tracker/src/pure_pursuit_tracker.cpp
          src/tracker/app/run_tracker.cpp
