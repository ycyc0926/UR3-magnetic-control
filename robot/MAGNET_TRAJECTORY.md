# 磁铁中心点位与轨迹脚本

统一入口是 ROS 2 节点 `magnet_trajectory`。它把任务点统一解释为
`table_world` 中磁铁中心的绝对坐标（单位 mm），自动换算成 `tool0` 位姿。默认
保持开始时的末端姿态不变，也可以显式要求最终电机轴平行于 `table_world` 的
X/Y/Z 轴。当前支持单点、XY 正方形、XY 圆形和任意三维多路点路径。

脚本只控制机械臂，不发送任何电机命令。当前电机必须停止；以后如果需要边转电机
边移动，应另做电机状态、线缆和急停联锁，不能删掉本脚本的确认开关来代替。

## 当前使用的工具参数

参数直接从当前配置读取，不使用计划更换的海泰电机或 30×30 mm 圆柱磁铁参数：

- 磁铁中心相对 `tool0`：`[0, -62, 41] mm`；
- 当前磁铁碰撞球半径：`10 mm`；
- 当前工具负载记录：`0.32 kg`；
- 当前电机轴：`tool0 -Y`；
- 暂定完整工具包络：`[-30,-75,-5]` 到 `[30,50,70] mm`（`tool0` 坐标）。

最后一项尚未现场确认包含全部支架、凸出件和线缆，因此真实执行需要单独明确接受
这个暂定包络。软件检查是应用级保护，不是安全认证功能。

## 构建

```bash
source /home/yc/UR3/ros2_env.sh
cd /home/yc/UR3/ros2_ws
colcon build --symlink-install --packages-select ur3_magnetic_control
source /home/yc/UR3/ros2_env.sh
```

规划需要真实或仿真的 `/joint_states`、MoveIt 服务和本机标定模型。真实系统通常先
启动 UR3 驱动，再另开终端启动标定版 MoveIt：

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  kinematics_params_file:=/home/yc/UR3/config/ur3_calibration.yaml \
  launch_rviz:=true
```

```bash
source /home/yc/UR3/ros2_env.sh
ros2 launch /home/yc/UR3/robot/calibrated_moveit.launch.py \
  ur_type:=ur3 launch_rviz:=false
```

仅做规划时不要求播放示教器的 External Control；真实执行前则必须播放且程序树中
只能有 External Control 节点。

## 默认只规划

下面坐标只是命令格式示例。每次都应先用本次实际目标执行只规划模式，并在 RViz、
`plan.json` 和轨迹图中复核结果。没有 `--execute` 时不会发送运动目标。

单点：

```bash
ros2 run ur3_magnetic_control magnet_trajectory point \
  --target-mm 120 10 350 --speed-mm-s 5
```

边长 20 mm、中心为 `(120,10,350) mm` 的闭合正方形：

```bash
ros2 run ur3_magnetic_control magnet_trajectory square \
  --center-mm 120 10 350 --size-mm 20 \
  --rotation-deg 0 --speed-mm-s 5
```

半径 10 mm 的闭合圆：

```bash
ros2 run ur3_magnetic_control magnet_trajectory circle \
  --center-mm 120 10 350 --radius-mm 10 --speed-mm-s 5
```

圆会自动选取足够路点，使相邻弦长不超过 2 mm；也可用 `--samples` 指定更多点。
正方形和圆形都会先从当前磁铁中心直线接近第一个图形点，再走完闭合图形。

多路点文件：

```bash
ros2 run ur3_magnetic_control magnet_trajectory waypoints \
  --file /home/yc/UR3/robot/magnet_waypoints.example.yaml \
  --speed-mm-s 5
```

YAML/JSON 可写成：

```yaml
frame: table_world
waypoints_mm:
  - [120.0, 0.0, 350.0]
  - [140.0, 0.0, 350.0]
  - [140.0, 20.0, 345.0]
```

CSV 列名固定为 `x_mm,y_mm,z_mm`。

当一个较长笛卡尔路径靠近逆解分支边界、整段规划发生关节跳变时，可用
`--planning-chunk-mm 2..50` 从同一实时起点顺序规划短段。各段末端状态会传给
下一段，随后合并为一条轨迹并重新执行全部速度、插值碰撞、净空和路径误差检查。
普通连续路径最长 300 秒；这种在每个短段边界停稳的分段路径最长 420 秒，两个
模式的速度和关节限位完全相同。

## 电机轴方向约束

用 `--motor-axis-parallel x|y|z` 指定最终轴线。`--motor-axis-direction nearest`
（默认）会在正、负两个平行方向中选择当前姿态转角较小的一侧，也可以用
`positive` 或 `negative` 锁定符号。默认 `--axis-alignment-phase after`：先走完球心
位置路径，再保持球心不动、让 `tool0` 绕球心旋转；`before` 则先原地对轴，再以新
姿态走位置路径。两种方式都重新检查完整工具包络、碰撞、关节限位和净空。

轴向只约束两个姿态自由度，剩余的绕轴滚转可用 `--motor-axis-roll-deg` 选择。
它不会改变最终轴向，但可能避开腕部逆解边界。应先从 `0` 开始只规划；只有规划
表明最小转角姿态不可达时，才选择通过审核的非零值。工具角速度与关节速度的规划
上限均为 5°/s，实测工具角速度中止阈值为 7°/s；实时监控还会检查姿态是否偏离
已审核的平移/绕球心旋转路径。

例如，最终轴平行世界 Y 且选择最近方向：

```bash
ros2 run ur3_magnetic_control magnet_trajectory point \
  --target-mm 120 10 350 --speed-mm-s 5 \
  --planning-chunk-mm 40 \
  --motor-axis-parallel y --motor-axis-direction nearest \
  --axis-alignment-phase after
```

如果机械臂开始时已在配置工作区以外，只允许使用经过复核的路点文件，并显式添加
`--allow-workspace-ingress`。该模式只允许已有越界量逐样本单调减小；一旦进入工作区
便不允许再次离开。

工作区外恢复或必须绕障的路径应保存为明确的多路点文件，先只规划审核；任何手动
移动都会使旧计划失效，不能重放历史轨迹。

## 输出和重新绘图

默认在 `robot/trajectory_runs/<UTC时间>_<命令>/` 新建一个不可覆盖的结果目录：

- `plan.json`：目标、配置哈希、完整关节轨迹、规划磁铁中心轨迹和净空结果；
- `magnet_path.png`：请求/规划/实测 XY 路径以及 X/Y/Z 随时间曲线；
- `actual_magnet_path.jsonl`：真实执行期间从关节反馈重算的磁铁中心轨迹；
- `execution.json`：执行成功和最终误差摘要；
- `failure.json`：规划或执行失败原因。

可从保存结果重新绘图：

```bash
ros2 run ur3_magnetic_control magnet_trajectory plot \
  --input /home/yc/UR3/robot/trajectory_runs/运行目录
```

也可把 `--input` 指向单个 `plan.json` 或 `actual_magnet_path.jsonl`，并用
`--output` 指定 PNG。

## 真实执行门控

真实执行会从最新静止状态重新规划，不会直接重放旧的 `plan.json`。只有下面所有
参数同时存在才会发送轨迹：

```bash
ros2 run ur3_magnetic_control magnet_trajectory point \
  --target-mm 120 10 350 --speed-mm-s 5 \
  --execute \
  --confirmation-token I_ACCEPT_REAL_ROBOT_MOTION \
  --accept-provisional-tool-envelope \
  --motor-stopped \
  --onsite-clearance-confirmed \
  --sole-operator-confirmed \
  --external-control-only-confirmed
```

这些参数是对“本次动作”的现场确认，不应写进 alias 或永久启动脚本。执行前还应
核对示教器限速、TCP/负载、急停位置、人员和线缆；任一项不成立就只做规划。

执行前还会确认 UR Dashboard Stop 服务可用。如果实时几何、反馈新鲜度、速度或
轨迹误差任一检查失败，脚本先调用 Dashboard Stop 停止 External Control 程序，
然后才取消 MoveIt 动作。原因是取消 ROS 动作不能保证已进入 UR 控制器缓冲区的
轨迹立即停止。Dashboard Stop 仍不是安全等级急停；若现场仍在运动，必须使用
示教器停止或硬件急停，且不得自动重试。

## 自动检查范围

- 使用本机 UR3 标定哈希，拒绝通用或错误的 MoveIt 模型；
- 默认固定开始姿态；显式指定轴向时，以 1° 姿态路点绕球心旋转，并按当前
  62/41 mm 偏置逐路点换算磁铁中心与 `tool0`；
- MoveIt 检查自碰撞、环境碰撞、关节跳变及带 0.15 rad 余量的关节限位，并对
  每段控制器样条的 1/4、1/2、3/4 内点再次做状态碰撞检查；
- 完整 URDF 连杆、磁铁球和暂定完整工具包络均参与独立几何复核；
- 全局最低净空为板底 5 mm、左右侧各 10 mm、桌面 10 mm；独立轨迹检查再增加
  2 mm 执行余量；
- 只有完整几何包络满足 `max(table_world Y) < 25 mm` 才免除板底限制；
- 磁铁中心还必须位于 `base` 坐标的配置工作区内；
- 笛卡尔采样最大 2 mm，单段最大 300 mm，总路径最大 1000 mm；
- 请求速度范围 0.1–20 mm/s，规划关节速度和工具角速度最大 5°/s；
- 关节位置限位、规划/实测速度限位不能用急停承诺替代或关闭；急停只是最后保护；
- 时间参数化后的磁铁路径与请求路径双向误差不超过 0.75 mm；
- 执行时实时检查完整几何、姿态路径、关节速度、工具角速度、磁铁速度和 2 mm
  路径偏差；最终测量位置误差必须不超过 0.75 mm，指定轴向时最终轴误差必须不
  超过 0.25°。

未建模项仍包括平台立柱、松动物体和完整动态线缆形状。控制器取消轨迹也不等价于
安全等级急停，所以现场硬件安全设置始终优先。
