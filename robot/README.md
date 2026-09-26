# UR3 机械臂

当前运动入口是 ROS 2 包 `ur3_magnetic_control` 中的 `magnet_trajectory`。
命令和当前末端参数见 [MAGNET_TRAJECTORY.md](MAGNET_TRAJECTORY.md)。

ZE300 图形控制窗口可双击桌面图标，或运行 `bash /home/yc/UR3/robot/start_ze300_gui.sh`。
重复打开会切回已有窗口；启动失败日志位于 `~/.local/state/ur3/ze300-gui.log`。

- `motion_stack.launch.py`：启动真实 UR3 驱动和本机标定版 MoveIt。
- `calibrated_moveit.launch.py`：加载本机 UR3 标定的 MoveIt。
- `urcap/`：示教器 External Control 插件。
- `ur3_realtime_monitor.py`、`rtde_status_outputs.py`、`record_rtde_outputs.py`：
  只读 RTDE 诊断和记录，不发送运动命令；对应测试位于 `tests/`。

真实执行默认保存在 `/home/yc/UR3/experiments/日期_时间/`，只规划结果保存在 `/home/yc/UR3/plans/`；
相机数据仅在跟踪页面点击录像后保存：实验运行期间放在该实验的 `h_robot/clip_录像时间/`，
其他时候放在 `experiments/camera_only/录像时间/h_robot/clip_录像时间/`。
旧电机与磁铁的规划和执行记录已清理；新硬件需重新规划。
