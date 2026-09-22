# UR3磁驱动实验 ROS 2 环境

使用 ROS 2 Humble。真实机器人型号参数必须是 `ur3`，不能写成 `ur3e`。

## 安全约定

- 真实驱动启动后不会自行运动；只有显式发送轨迹目标才会运动。
- 真实运动前必须单独确认坐标系、距离、速度、路径，并把示教器和急停放在手边。
- 真实运动只使用统一的 `magnet_trajectory` 入口，不要同时运行其他机械臂控制客户端。
- 所有正常轨迹、停车模型和在线检查统一使用全局净空策略：亚克力覆盖区板底
  `5 mm`、左右侧各 `10 mm`、桌面 `10 mm`。唯一配置位置是
  `config/ur3_system.yaml:safety.clearance_policy_m`，运动脚本不得单独覆盖。
- 亚克力内侧左下角世界坐标为 `(0.135,0.025) m`，记录在
  `config/table_world_calibration.yaml`。当前只启用下边缘：完整 link/工具包络
  `max(Y)<0.025 m` 时免除板底高度限制；触及或跨越该边界仍应用 5 mm 净空。
  X=0.135 m 暂只记录，不产生 X 方向豁免。
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

## FLIR 相机与 H 目标定位

统一启动脚本使用相机序列号对应的 USB3 设备，以 2448×2048 BayerRG8 采集，
转换并缩放为 1224×1024 BGR 后发布，同时启动 H 目标定位：

```bash
bash /home/yc/UR3/camera/start_h_tracking.sh
```

该启动方式不会连接或控制 UR3，也不会发送电机命令。详细话题、录像和定位限制见
[相机程序](../../camera/README.md)。

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

## 统一的磁铁中心轨迹节点

机械臂运动统一使用 `magnet_trajectory`。目标坐标是 `table_world` 中的磁铁中心，
默认只规划；单点、正方形、圆形、多路点和电机轴方向约束共用同一套碰撞、净空、
关节限位、速度及实测轨迹检查。底层笛卡尔规划实现保留为包内公共模块，不再提供
容易绕过统一检查的独立运动命令。

单点规划示例：

```bash
source /home/yc/UR3/ros2_env.sh
ros2 run ur3_magnetic_control magnet_trajectory point \
  --target-mm 120 10 350 --speed-mm-s 5
```

只有显式添加 `--execute` 和全部现场确认参数时才会执行真实运动。完整参数、轨迹
文件格式、绘图方式和执行门控见 [磁铁中心点位与轨迹脚本](../../robot/MAGNET_TRAJECTORY.md)。
