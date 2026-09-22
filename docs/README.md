# 文档与资料

- `setup/`：当前环境和部署说明。
- `history/`：按日期保存的实验交接、进展和旧流程，仅供追溯。
- `images/hardware/`：设备、工具和现场照片。
- `images/coordinates/`：UR3 基座、TCP 和坐标系示意图。
- `references/`：论文与参考资料。

当前运行规则以根目录 `README.md`、`config/` 和代码为准。`history/` 中的旧阈值、
旧路径或旧运行状态不能作为新的实机执行授权。

根目录现在只保留 camera/、config/、docs/、robot/、ros2_ws/、vendor/。
  - 机械臂测试统一移到 robot/tests/。
  - 硬件照片、坐标图和论文统一移到 docs/。
  - 历史交接和实验进展移到 docs/history/。
  - 旧规划审计记录位于 robot/planning_checks/，统一轨迹运行记录位于
    robot/trajectory_runs/；两者都只在实验电脑保留。
  - 四段 H 明显移动的录像集中建立了快捷入口：移动录像目录 (camera/
    motion_examples/README.md)。
