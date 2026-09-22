# UR3 机械臂代码

本目录只保留通用的 UR3 辅助工具、文档；实验电脑另保留一份成功记录。机械臂点位与轨迹控制的
统一实现位于 ROS 2 包 `ur3_magnetic_control`；不再在这里保存每次实验的一次性
规划器或执行器。

## 使用边界

- `config/ur3_system.yaml:safety.clearance_policy_m` 是唯一净空配置源：亚克力覆盖区
  板底 5 mm、左右侧各 10 mm、桌面 10 mm。
- 亚克力内侧左下角实测世界坐标记录在 `config/table_world_calibration.yaml`：
  `(X,Y)=(135,25) mm`。完整几何包络仅在 `max(Y)<25 mm` 时免除板底高度限制；
  触及或跨越边界仍按 5 mm 检查。X 坐标暂只记录，不用于豁免。
- 历史报告中的 3、18、20、23 或 100 mm 阈值不能覆盖当前策略。
- `planning_checks/` 中的轨迹和报告只用于审计，不能直接重放。
- 真实运动统一使用 `ur3_magnetic_control/magnet_trajectory`，仍须针对每次动作先
  规划并取得现场确认。

## 代码索引

只读状态和记录：

- `ur3_realtime_monitor.py`：直接读取 RTDE 状态，不发送运动。
- `rtde_status_outputs.py`、`record_rtde_outputs.py`：状态解析和独立输出记录。

离线检查和辅助入口：

- `offline_collision/`：MoveIt/FCL 自碰撞和控制器插值离线检查器。
- `calibrated_moveit.launch.py`：加载本机 UR3 标定的 MoveIt。

## 保留的成功记录

`planning_checks/square24_xy_alignment_20260919_execution_02/` 是清理后保留的
一份历史成功执行数据；`trajectory_runs/target_146_118_400_recovery_execution_20260922a/`
是统一轨迹脚本的一份成功执行数据。二者仅用于审计，不能直接重放。

坐标系示意图位于 [`docs/images/coordinates/`](../docs/images/coordinates/)，硬件照片
位于 [`docs/images/hardware/`](../docs/images/hardware/)。当前坐标约定为：在基座坐标
系下，从机械臂带线缆一侧看，TCP 向左为 +X、向后为 +Y、向上为 +Z。

## 测试

```bash
source /home/yc/UR3/ros2_env.sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  /home/yc/UR3/robot/tests
```

这里只保留三份通用 RTDE 工具的离线测试；统一轨迹脚本测试位于 ROS 2 包内。
