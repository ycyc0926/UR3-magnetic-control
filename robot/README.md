# UR3 机械臂代码

本目录保存不适合放进通用 ROS 2 包的 UR3 实验工具。源代码留在本目录根部，测试
集中在 `tests/`，C++ 离线碰撞工具在 `offline_collision/`，历史规划和执行记录在
`planning_checks/`。

## 使用边界

- `config/ur3_system.yaml:safety.clearance_policy_m` 是唯一净空配置源：板底 5 mm、
  左右侧各 10 mm、桌面 10 mm。
- 历史报告中的 3、18、20、23 或 100 mm 阈值不能覆盖当前策略。
- `planning_checks/` 中的轨迹和报告只用于审计，不能直接重放。
- 所有 `run_*`、`bounded_*`、`guarded_*` 和旧 `ur3_linear_move.py` 都可能包含
  实机执行路径；没有本次具体动作授权时只能进行离线或只读检查。

## 代码索引

只读状态和记录：

- `ur3_realtime_monitor.py`：直接读取 RTDE 状态，不发送运动。
- `rtde_status_outputs.py`、`record_rtde_outputs.py`：状态解析和独立输出记录。

几何预览和离线规划：

- `preview_yaw_geometry.py`、`preview_yaw_clearance.py`：FK、工具包络和净空诊断。
- `prepare_yaw_pilot.py`、`prepare_lower_yaw.py`：偏航/下降轨迹草案。
- `prepare_xy_alignment.py`、`prepare_xy_motor_probe.py`：XY 对齐和短探测试验草案。
- `prepare_uniform_x100.py`、`capture_uniform_x100_baseline.py`：匀速 X 轨迹及基线。
- `offline_collision/`：MoveIt/FCL 自碰撞和控制器插值离线检查器。

受保护执行器：

- `bounded_lower_yaw.py`、`run_checked_yaw.py`
- `run_checked_xy_alignment.py`、`run_checked_xy_motor_probe.py`
- `guarded_level_workflow.py`

辅助入口：

- `calibrated_moveit.launch.py`：加载本机 UR3 标定的 MoveIt。
- `manual_motor_probe_guard.py`、`explicit_probe_trigger.py`：人工电机试验门控。
- `ur3_linear_move.py`：旧 URScript MoveL 工具；当前板下实验不要使用执行模式。

## 历史找平记录

2026-09-19 的找平计划和执行结果已归档到
[`planning_checks/level_magnet_axis_20260919/`](planning_checks/level_magnet_axis_20260919/)。
其中约 4.43 mm 的板底模型间隙低于当前 5 mm 策略，只能作为历史记录，不能据此
重放动作。

坐标系示意图位于 [`docs/images/coordinates/`](../docs/images/coordinates/)，硬件照片
位于 [`docs/images/hardware/`](../docs/images/hardware/)。当前坐标约定为：在基座坐标
系下，从机械臂带线缆一侧看，TCP 向左为 +X、向后为 +Y、向上为 +Z。

## 测试

```bash
source /home/yc/UR3/ros2_env.sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  /home/yc/UR3/robot/tests
```

测试按设计应为离线测试；浏览器相关测试可能因缺少独立 Firefox/Marionette 环境而
跳过。
