# UR3 磁驱动机器人实验

本目录包含 UR3、FLIR 相机和 H 型磁机器人的实验代码。代码、配置、实验数据和
历史资料已经分开存放；真实机械臂运动前必须重新读取当前状态并生成新的审核结果，
不得直接重放历史轨迹。

## 目录结构

| 目录 | 内容 |
|---|---|
| `config/` | 唯一的系统、标定、工具和场景配置 |
| `ros2_ws/src/ur3_magnetic_control/` | 正式 ROS 2 Python 包、统一轨迹入口和包内测试 |
| `robot/` | 当前 UR3 启动入口、轨迹说明和通用只读 RTDE 工具 |
| `camera/` | 相机启动、标定脚本和 H 跟踪入口 |
| `experiments/camera_only/` | 未关联主程序实验的独立相机录像 |
| `experiments/` | 每次真实执行一个日期时间目录，含机械臂、磁铁、电机和 H 机器人记录 |
| `plans/` | 只规划时产生的审核结果，不算实验执行 |
| `docs/` | 环境说明、历史交接、照片和参考论文 |
| `vendor/` | 固定版本的第三方 UR ROS 2 驱动源码 |

ROS 的 `build/`、`install/`、`log/` 和 Python 缓存都是生成物。当前
`ros2_ws/install/` 使用指向 `build/` 的符号链接，因此不要只删 `build/`；需要彻底
重建时应同时清理三者再运行 `colcon build --symlink-install`。

## 常用入口

加载 ROS 2 Humble 和本工作区：

```bash
source /home/yc/UR3/ros2_env.sh
```

启动 H 机器人视觉跟踪（不控制机械臂或电机）：

```bash
bash /home/yc/UR3/camera/start_h_tracking.sh
```

运行全部离线测试：

```bash
source /home/yc/UR3/ros2_env.sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/test \
  /home/yc/UR3/robot/tests
```

详细入口见 [机械臂说明](robot/README.md)、[统一磁铁轨迹说明](robot/MAGNET_TRAJECTORY.md)、
[相机说明](camera/README.md)和 [ROS 2 环境说明](docs/setup/ROS2.md)。H 移动录像的快捷入口位于
[`camera/motion_examples/`](camera/motion_examples/README.md)。

ZE300 电机连接监视服务在用户登录后自动运行，断电重连时按已保存的单圈绝对角度
回到磁铁原点。此仓库记录 H 机器人的视觉轨迹，不含 H 机器人的驱动控制。

## 当前安全配置

全局净空策略只允许在
`config/ur3_system.yaml:safety.clearance_policy_m` 中配置：

- 亚克力板底净空：进入亚克力覆盖区域时为 5 mm
- 左、右侧净空：各 10 mm
- 桌面净空：10 mm

运动脚本不得分别覆盖板底或左右侧净空。历史报告中出现的其他阈值仅描述当时
工况，不符合当前配置时不得复用。

2026-09-22 记录的亚克力内侧左下角世界坐标为 `(X,Y)=(135,25) mm`。当前有限
顶板和两块侧板都从用户确认的 `Y=25 mm` 前沿开始：一个 link/工具的完整世界坐标包络满足
`max(Y) < 25 mm` 时，不受顶板底面及侧板约束；到达或跨越 `Y=25 mm` 时执行
板底与侧板净空检查。`X=135 mm` 已记录，但尚未作为高度豁免边界。

## 数据保留原则

实验执行记录默认写入 `/home/yc/UR3/experiments/日期_时间/`，一次命令执行
只建一个目录，所有路点写入同一份轨迹文件。只规划的输出位于
`/home/yc/UR3/plans/`；`--output` 可显式指定其他位置。
相机仅在点击录像后保存：实验运行期间写入该实验的 `h_robot/clip_录像时间/`，
其他时候写入 `experiments/camera_only/录像时间/h_robot/clip_录像时间/`。

GitHub 轻量版本只提交源码、配置、说明和小型图片；原始跟踪会话、
标定原图、论文 PDF、构建输出以及完整第三方驱动快照仅保留在实验电脑。为保证克隆
后仍可构建，本项目实际修改过的 `ur_calibration` 小型源码包直接保存在
`ros2_ws/src/ur_calibration/`。
