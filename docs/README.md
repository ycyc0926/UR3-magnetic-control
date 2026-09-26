# 文档与资料

- `setup/`：当前环境和部署说明。
- `images/hardware/`：设备、工具和现场照片。
- `images/coordinates/`：UR3 基座、TCP 和坐标系示意图。
- `references/`：论文与参考资料。

磁铁标定原始数据见 [圆柱磁铁磁场实测记录](references/magnet_field_measurements.md)。

当前运行规则以根目录 `README.md`、`config/` 和代码为准。

机械臂轨迹测试位于 ROS 2 包内，通用 RTDE 工具测试位于 `robot/tests/`。
新轨迹运行记录默认写入 `/home/yc/UR3/experiments/`，只规划结果写入 `/home/yc/UR3/plans/`。
