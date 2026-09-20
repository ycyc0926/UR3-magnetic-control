# 只读换向碰撞诊断

这个程序只在内存中构造 MoveIt PlanningScene/FCL。没有 ROS 节点、硬件驱动、发布器、网络连接或执行接口。Python 包装程序默认只读取一次 RTDE 输出；指定 `--snapshot` 时不连接机械臂。

```bash
source /home/yc/UR3/ros2_env.sh
cmake -S /home/yc/UR3/robot/offline_collision -B /home/yc/UR3/robot/offline_collision/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/yc/UR3/robot/offline_collision/build -j2
python3 /home/yc/UR3/robot/preview_yaw_clearance.py --output /home/yc/UR3/robot/planning_checks/NEW_DIRECTORY
```

输出目录必须不存在。结果中的关节采样没有时间参数，**不能送给控制器执行**。`sampled_geometry_pass` 仅指当前假设下的离散模型检查；`execution_allowed` 始终为 false。

检查范围：

- 本机标定后的 UR3 正逆运动学；两个独立 FK 实现的一致性。
- 全部 URDF 碰撞面与两侧无限平面、板底及桌面的间隙；阈值统一读取
  `config/ur3_system.yaml:safety.clearance_policy_m`，当前为板底 5 mm、
  左右侧各 10 mm、桌面 10 mm，规划器不得单独覆盖。
- MoveIt/FCL 自碰撞，以及暂定工具包围盒与机械臂的碰撞。
- 保留官方 SRDF 的相邻/never 碰撞排除规则；工具仅允许接触 tool0、flange、wrist_3_link。
- 加入后移除一个必然碰撞的测试盒，验证碰撞检测确实有效。
- URDF 关节位置范围，采样点之间的关节变化幅度。

不包括实物工具包络的确认、铝型材立柱和杂物、线缆、控制器实际时间插值、速度/加速度/停车过程，以及标定和实物变形误差。不得据此自动批准实机运行。
