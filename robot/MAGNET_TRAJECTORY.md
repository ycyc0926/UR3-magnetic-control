# 磁铁中心点位与轨迹脚本

统一入口是 ROS 2 节点 `magnet_trajectory`。它把任务点统一解释为
`table_world` 中磁铁中心的绝对坐标（单位 mm），自动换算成 `tool0` 位姿。默认
保持开始时的末端姿态不变，也可以显式要求最终电机轴平行于 `table_world` 的
X/Y/Z 轴。当前支持单点、XY 正方形、XY 圆形和任意三维多路点路径。
也可选择在轨迹期间使 `tool0` 的 Z 轴平行桌面。

轨迹命令可用 `--motor-rpm` 在机械臂运动前启动 ZE300 电机。规划模式不发送电机指令。
成功后电机保持设定转速；若机械臂执行失败，程序尝试发送 0 rpm。

## 当前使用的工具参数

参数直接从当前配置读取：

- 电机：`HT4510-J10-2E`，转轴与 `tool0 +Z` 同轴；
- 磁铁：N52 轴向充磁圆柱，直径和长度各 30 mm，参考相位 N 指向 `tool0 +X`；
- 磁铁中心相对 `tool0`：`[0, 0, 97.5] mm`；
- 完整固定件负载：`0.54 kg`，质心尚未实测；
- 用户测得最远径向尺寸 38 mm、最高到 `tool0 +Z 115 mm`；法兰壳 STL 最大径向约 38.91 mm，因此暂定完整工具包络为 `[-39,-39,-5]` 到 `[39,39,115] mm`。

磁铁转动时 N 极方向随相位变化；碰撞模型使用绕电机轴旋转一周的包络，半径约
21.21 mm、高 30 mm。

包络的负 Z 端及线缆形状尚未实测。旧硬件的轨迹规划结果和起点假设在换件后失效。

## 构建

```bash
source /home/yc/UR3/ros2_env.sh
cd /home/yc/UR3/ros2_ws
colcon build --symlink-install --packages-select ur3_magnetic_control
source /home/yc/UR3/ros2_env.sh
```

规划需要真实或仿真的 `/joint_states`、MoveIt 服务和本机标定模型。实验时在
终端 1 同时启动 UR3 驱动与标定版 MoveIt：

```bash
source /home/yc/UR3/ros2_env.sh && ros2 launch /home/yc/UR3/robot/motion_stack.launch.py
```

仅做规划时不要求播放示教器的 External Control；真实执行前则必须播放且程序树中
只能有 External Control 节点。

## 默认只规划

下面坐标只是命令格式示例。每次都应先用本次实际目标执行只规划模式，并在 RViz、
`plan.json` 和轨迹图中复核结果。没有 `--execute` 时不会发送运动目标。

单点：省略 Z 时保持启动规划时的磁铁中心高度；使用 `--target-z-mm` 时保持当前 XY。
速度默认 5 mm/s，可用 `--speed-mm-s` 设置为 0–20 mm/s；0 表示保持机械臂静止，
不会发送运动命令。
所有磁铁轨迹要求 `wrist_3_joint=0°`，规划容差 0.5°，执行监测容差 1°。
启动轨迹前必须先使手腕 3 到达 0°。
普通点位移动允许末端姿态相对起始姿态偏差不超过 10°，以便手腕 3 保持 0°；
规划生成后会按固定手腕 3 的关节轨迹重新检查磁铁中心路径与碰撞。

```bash
ros2 run ur3_magnetic_control magnet_trajectory wrist3-zero --execute
```

```bash
ros2 run ur3_magnetic_control magnet_trajectory point --target-xy-mm 88 148
```

如需指定高度，仍用 `--target-mm X Y Z`。

边长 20 mm、中心为 `(120,10,350) mm` 的闭合正方形。正方形默认先使
`tool0 +Z`（也是电机转轴）平行桌面，再开始平面轨迹：

```bash
ros2 run ur3_magnetic_control magnet_trajectory square \
  --center-mm 120 10 350 --size-mm 20 \
  --rotation-deg 0
```

若磁铁**当前**在 `(88,148)` mm，以它作左下角起点走 50×50 mm 正方形，
中心是 `(113,173)` mm。`--center-xy-mm` 保持当前磁铁高度。先规划新硬件路径：

```bash
ros2 run ur3_magnetic_control magnet_trajectory square --center-xy-mm 113 173 --size-mm 50 --tool-z-heading-deg 60
```

`--tool-z-parallel-table` 也可显式选择该模式。换件前验证的姿态和路径不适用于新偏置；
若规划显示逆解分支不可达，可检查现场姿态后试用滚转角或 2–50 mm 分段规划。

```bash
ros2 run ur3_magnetic_control magnet_trajectory square --center-xy-mm 113 173 --size-mm 50 --tool-z-parallel-table --tool-z-heading-deg 60 --axis-roll-deg 90 --planning-chunk-mm 50
```

这条命令只做规划，尚未在新工具上验证可达性。

半径 10 mm 的闭合圆：

```bash
ros2 run ur3_magnetic_control magnet_trajectory circle \
  --center-mm 120 10 350 --radius-mm 10
```

圆会自动选取足够路点，使相邻弦长不超过 2 mm；也可用 `--samples` 指定更多点。
正方形和圆形都会先从当前磁铁中心直线接近第一个图形点，再走完闭合图形。

多路点文件：

```bash
ros2 run ur3_magnetic_control magnet_trajectory waypoints \
  --file /home/yc/waypoints.yaml
```

先将自己的路点保存到该文件。YAML/JSON 可写成：

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

用 `--motor-axis-parallel x|y|z` 指定最终轴线；正方形默认使 `tool0 +Z` 和电机轴
平行桌面，并在平移前完成对齐。`table` 模式要求电机轴位于 `tool0` 的 XY 平面，
不适用于当前电机。`--motor-axis-direction nearest`
（默认）会在正、负两个平行方向中选择当前姿态转角较小的一侧，也可以用
`positive` 或 `negative` 锁定符号。默认 `--axis-alignment-phase after`：先走完磁铁中心
位置路径，再保持磁铁中心不动、让 `tool0` 绕磁铁中心旋转；`before` 则先原地对轴，再以新
姿态走位置路径。两种方式都重新检查完整工具包络、碰撞、关节限位和净空。

轴向只约束两个姿态自由度，剩余的绕轴滚转可用 `--motor-axis-roll-deg` 选择。
它不会改变最终轴向，但可能避开腕部逆解边界。应先从 `0` 开始只规划；只有规划
表明最小转角姿态不可达时，才选择通过审核的非零值。工具角速度与关节速度的规划
上限均为 5°/s。

例如，最终轴平行世界 Y 且选择最近方向：

```bash
ros2 run ur3_magnetic_control magnet_trajectory point \
  --target-mm 120 10 350 \
  --planning-chunk-mm 40 \
  --motor-axis-parallel y --motor-axis-direction nearest \
  --axis-alignment-phase after
```

需要绕障的路径可保存为明确的多路点文件，先只规划审核；任何手动移动都会使旧计划
失效，不能重放历史轨迹。

## 输出和重新绘图

真实执行默认在 `/home/yc/UR3/experiments/<本地日期_时间>/` 新建一个不可覆盖的目录；
一次任务的所有路点写在同一份轨迹文件中。只规划输出写入 `/home/yc/UR3/plans/`。
可用 `--output` 指定其他位置。
临时动作可加 `--no-record`，运行结束即清理这次产生的规划和执行文件；
`--no-record` 不与 `--output` 同时使用。

- `plan.json`：目标、配置哈希、完整关节轨迹、规划末端 `tool0` 与磁铁中心轨迹和净空结果；
- `magnet_path.png`：请求/规划/实测 XY 路径以及 X/Y/Z 随时间曲线；
- `actual_magnet_path.jsonl`：同一任务的所有机械臂采样，含末端 `tool0` 位置与姿态、磁铁中心位置、电机角度、磁铁 N 极方向和磁铁姿态四元数；
- `motor_samples.jsonl`：真实执行期间的电机编码器采样与电脑时间戳；
- `h_robot/positions.csv`：按本次执行时间截取的 H 机器人相机观测；`h_robot/metadata.json` 记录源会话、帧数和检出帧数。未检测到 H 时没有有效 H 轨迹；
- `execution.json`：执行成功和最终误差摘要；
- `failure.json`：规划或执行失败原因。

相机原始连续会话仍保存在 `/home/yc/UR3/camera/tracking_sessions/`；实验目录中的 H 数据是按时间截取的副本。
两路数据使用电脑与相机消息时间戳对齐，没有硬件触发同步。
`config/ur3_system.yaml` 的 `calibration.motor_phase_sign` 已按 +90° 实测结果设为 `-1`：
编码器正转时磁铁 N 极从 tool0 +X 转向 -Y。
`motor_encoder_turns_per_magnet_turn` 已按当前直连结构设为 `1.0`。
后续执行的轨迹可记录磁铁 N 极方向和姿态四元数；旧记录若无同步编码器采样则无法补算。

可从保存结果重新绘图：

```bash
ros2 run ur3_magnetic_control magnet_trajectory plot \
  --input /home/yc/UR3/experiments/运行目录
```

也可把 `--input` 指向单个 `plan.json` 或 `actual_magnet_path.jsonl`，并用
`--output` 指定 PNG。

## 真实执行

真实执行会从最新静止状态重新规划，不会直接重放旧的 `plan.json`。
终端 1 启动后，终端 2 用下面一条命令移动到 `(88, 148)` mm，保持当前磁铁高度，
默认速度 5 mm/s。只有 `--execute` 会发送轨迹：

```bash
source /home/yc/UR3/ros2_env.sh && ros2 run ur3_magnetic_control magnet_trajectory point --target-xy-mm 88 148 --execute
```

同时以 10 rpm 旋转磁铁并移动其中心到 `(146,146,400)` mm：

```bash
source /home/yc/UR3/ros2_env.sh && ros2 run ur3_magnetic_control magnet_trajectory point --target-mm 146 146 400 --motor-rpm 10 --execute
```

ZE300 默认连接 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，地址 1、115200 baud。
端口或设备地址不同可用 `--motor-port`、`--motor-address`、`--motor-baud` 指定。
轨迹命令也可用 `--motor-position-deg` 或 `--motor-relative-deg` 在机械臂运动前发送位置指令；
这三个电机控制参数互斥。
运行前需关闭 ZE300_GUI 的串口连接。可单独读取和控制电机：

```bash
source /home/yc/UR3/ros2_env.sh
ros2 run ur3_magnetic_control ze300_motor status
ros2 run ur3_magnetic_control ze300_motor speed 10
ros2 run ur3_magnetic_control ze300_motor absolute 90
ros2 run ur3_magnetic_control ze300_motor relative -90
ros2 run ur3_magnetic_control ze300_motor home --wait
ros2 run ur3_magnetic_control ze300_motor speed 0
```

位置单位为度；`absolute` 使用多圈绝对位置，`home` 按最短转角回到电机已设定的原点；
`--wait` 会等到角度和转速都接近 0 再返回。
`off` 指令会关闭电机输出并使轴自由转动。

已在磁铁 N 极指向 tool0 `+X` 时保存单圈绝对角度到
`/home/yc/UR3/config/motor_origin.json`。电机断电后多圈角度会丢失；恢复使用保存的
单圈角度，选择最短转角。连接监视服务在当前用户登录后运行；电机重新应答时自动恢复，
轨迹程序占用串口时等待，不会在实验过程中插入命令。查看状态或手动恢复：

```bash
systemctl --user status ur3-motor-origin.service
source /home/yc/UR3/ros2_env.sh && ros2 run ur3_magnetic_control ze300_motor restore-origin
```

仅在重新确认磁铁 N 极方向并停止电机后，运行 `ze300_motor save-origin` 更新原点参照。

执行前核对示教器限速、TCP/负载、急停位置、人员和线缆。

执行前还会确认 UR Dashboard Stop 服务可用。如果亚克力或桌面净空失败、关节反馈
失去新鲜度，脚本先调用 Dashboard Stop 停止 External Control 程序，
然后才取消 MoveIt 动作。原因是取消 ROS 动作不能保证已进入 UR 控制器缓冲区的
轨迹立即停止。Dashboard Stop 仍不是安全等级急停；若现场仍在运动，必须使用
示教器停止或硬件急停，且不得自动重试。

## 自动检查范围

- 使用本机 UR3 标定哈希，拒绝通用或错误的 MoveIt 模型；
- 默认固定开始姿态；显式指定轴向时，以 1° 姿态路点绕磁铁中心旋转，并按当前
  `[0,0,97.5] mm` 偏置逐路点换算磁铁中心与 `tool0`；
- MoveIt 检查自碰撞、环境碰撞、关节跳变及带 0.15 rad 余量的关节限位，并对
  每段控制器样条的 1/4、1/2、3/4 内点再次做状态碰撞检查；
- 完整 URDF 连杆、磁铁圆柱和暂定完整工具包络均参与独立几何复核；
- 全局最低净空为板底 5 mm、左右侧各 10 mm、桌面 10 mm；
- 完整几何包络满足 `max(table_world Y) < 25 mm` 时，不受顶板底面与侧板约束；
- 笛卡尔采样最大 2 mm，单段最大 300 mm，总路径最大 1000 mm；
- 请求速度默认 5 mm/s，范围 0–20 mm/s（0 为静止）；规划关节速度和工具角速度最大 5°/s；
- 规划仍检查关节限位、碰撞和请求速度；
- 时间参数化后的磁铁路径与请求路径双向误差不超过 0.75 mm；
- 执行时实时检查完整几何与亚克力、桌面边界；实测速度和路径偏差不再触发停机。
  最终测量位置误差必须不超过 0.75 mm，指定轴向时最终轴误差必须不超过 0.25°。

未建模项仍包括平台立柱、松动物体和完整动态线缆形状。控制器取消轨迹也不等价于
安全等级急停，所以现场硬件安全设置始终优先。
