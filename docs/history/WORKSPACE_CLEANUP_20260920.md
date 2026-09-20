# 工作区整理记录（2026-09-20）

本次整理没有连接或控制机械臂、相机或电机。

## 结构调整

- 根目录环境说明移至 `docs/setup/ROS2.md`。
- 交接、进展和旧方形流程移至 `docs/history/`。
- 两份论文移至 `docs/references/`。
- 硬件照片和坐标图分别移至 `docs/images/hardware/` 与
  `docs/images/coordinates/`，并同步更新配置引用。
- `robot/test_*.py` 移至 `robot/tests/`，增加测试路径配置。
- 找平计划/执行 JSON 移至
  `robot/planning_checks/level_magnet_axis_20260919/`。
- 增加 `camera/motion_examples/`，用相对符号链接集中展示四段 H 移动录像，原始
  文件仍保留在 `camera/tracking_sessions/`。
- 增加根目录、实验数据和规划审计索引文档。

## 已清理的可再生成内容

下列内容使用 `gio trash` 移入桌面环境回收站，清空回收站前可恢复：

- 工作区内的 Python `__pycache__` 和 pytest 缓存；
- `ros2_ws/log/` 中的历史 colcon 构建/测试日志；
- `camera/SpinView_QT_log` 运行日志。

没有删除 `ros2_ws/src/`、`build/` 或 `install/`，也没有删除任何录像、CSV、RTDE
记录、规划报告、执行报告、标定样本或参考资料。

## 验证

- ROS 包测试与 `robot/tests/`：101 passed，4 skipped。
- Python 源码编译检查通过。
- 配置引用的硬件照片全部存在。
- 四个录像快捷链接均可解析到原始 AVI。
- 文档中的本地 Markdown 链接检查通过。
