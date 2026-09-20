# UR3磁驱动实验 ROS 2 环境

使用 ROS 2 Humble。真实机器人型号参数必须是 `ur3`，不能写成 `ur3e`。

## 安全约定

- `mock_system.launch.py` 只启动模拟图像、视觉检测、测试路径、模拟电机和安全监控，不连接真实UR3。
- 模拟电机默认拒绝所有非零转速。
- 真实驱动启动后不会自行运动；只有显式发送轨迹目标才会运动。
- 真实运动前必须单独确认坐标系、距离、速度、路径，并把示教器和急停放在手边。
- 不要同时运行旧的 `ur3_linear_move.py` 和 ROS 2 External Control。
- 所有正常轨迹、停车模型和在线检查统一使用全局净空策略：亚克力板底
  `5 mm`、左右侧各 `10 mm`、桌面 `10 mm`。唯一配置位置是
  `config/ur3_system.yaml:safety.clearance_policy_m`，运动脚本不得单独覆盖。
- 历史规划报告使用过的 `3/18/20/23/100 mm` 阈值只描述当时的审核，配置哈希
  已变化，不能直接重放；必须按当前全局策略重新生成审核。

## 加载与构建

```bash
source /home/yc/UR3/ros2_env.sh
cd /home/yc/UR3/ros2_ws
colcon build --symlink-install
source /home/yc/UR3/ros2_env.sh
```

## 低延迟内核与实时权限

已安装 `linux-lowlatency-hwe-22.04`（内核 `6.8.0-138-lowlatency`）和
`rt-tests`，用户 `yc` 已加入 `realtime` 组，权限配置位于
`/etc/security/limits.d/99-realtime.conf`。

2026-09-20 已复核当前会话正在运行 low-latency 内核，实时优先级上限为 99：

```bash
uname -r
ulimit -r
```

当前结果分别为 `6.8.0-138-lowlatency` 和 `99`。更换内核或系统配置后应重新检查。

## 无硬件模拟验证

```bash
ros2 launch ur3_magnetic_control mock_system.launch.py
```

另开终端可查看：

```bash
source /home/yc/UR3/ros2_env.sh
ros2 topic list
ros2 topic echo /safety/status
```

主要话题：

- `/camera/image_raw`：模拟相机图像
- `/origami/pose_pixels`：目标像素位置
- `/origami/confidence`：检测置信度
- `/desired_path`：100 mm测试直线路径
- `/motor/command_rpm`：电机转速命令，当前默认被安全拒绝
- `/motor/state_rpm`：模拟电机状态
- `/safety/stop_requested`：安全停止请求
- `/safety/status`：安全状态

## FLIR真实相机ROS采集

以下命令使用相机序列号对应的USB3设备，以2448×2048 BayerRG8采集，在进入ROS前
转换并缩放为1224×1024 BGR，发布目标为20 FPS：

```bash
source /home/yc/UR3/ros2_env.sh
ros2 launch ur3_magnetic_control flir_vision.launch.py
```

该启动文件不会连接或控制UR3。当前绿色目标检测器只是接口验证版本，折纸机器人
到货后必须按实际外观重新设计检测方法和阈值。

## 示教器上的 External Control

URCap文件在 `robot/urcap/`。在CB3示教器上：

1. `设置机器人 → URCaps → +`，安装 `externalcontrol-1.0.5.urcap`，按提示重启控制箱。
2. `编程机器人 → 安装设置 → External Control`，主机IP填写 `192.168.56.1`，端口保留 `50002`。
3. 新建程序，只加入一个 `External Control` 节点，保存为 `external_control.urp`。
4. 电脑端驱动启动后，在示教器上运行这个程序。

`.urp` 应由PolyScope界面生成，不在电脑上手工构造。

## 提取厂家标定

该命令只读取标定，不会移动机械臂：

```bash
source /home/yc/UR3/ros2_env.sh
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.56.101 \
  target_filename:=/home/yc/UR3/config/ur3_calibration.yaml
```

2026-09-15安装的软件源同时提供了 `ur_calibration 2.14.0` 与
`ur_client_library 2.15.0`。后者把连接初始化从 `run()` 中分离，前者尚未调用
初始化，原始二进制会一直重连但不建立TCP连接。本工作区覆盖了同版本官方
`ur_calibration`，只增加 `pipeline.init()`，可构建源码保存在
`ros2_ws/src/ur_calibration/`；完整上游仓库快照仅在实验电脑的 `vendor/` 中留档。

## 启动真实驱动

先只观察状态，不发送轨迹：

```bash
source /home/yc/UR3/ros2_env.sh
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  kinematics_params_file:=/home/yc/UR3/config/ur3_calibration.yaml \
  launch_rviz:=true
```

驱动显示等待机器人程序后，再运行示教器中的 `external_control.urp`。

## 受保护的笛卡尔直线节点

`cartesian_line_move` 默认只规划，不向机械臂发送命令。它使用 MoveIt 笛卡尔
路径服务进行 2 mm 采样，检查路径覆盖率、自碰撞、关节跳变和带余量的关节限位，
并处理连续关节的 `2π` 表示。

规划示例：

```bash
source /home/yc/UR3/ros2_env.sh
ros2 run ur3_magnetic_control cartesian_line_move \
  --axis=-z --distance-mm 150 --speed-mm-s 10
```

只有显式添加 `--execute` 和确认令牌时才会执行真实运动。真实执行前仍必须确认
方向、距离、速度、现场净空、示教器限速和急停位置。

2026-09-15 实测：基座 `-Z` 方向规划 150 mm，60 个轨迹点，覆盖率 100%。
示教器限速 8% 时控制器执行成功；实际 TCP 从约
`(-50.610, -110.226, 253.597) mm` 到
`(-50.302, -110.337, 103.688) mm`，Z 位移约 149.909 mm，结束线速度
0.000 mm/s，安全状态 NORMAL。
