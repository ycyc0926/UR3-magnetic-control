# UR3 驱动 H 型磁机器人轨迹实验：完整交接

核验日期：2026-09-19，Asia/Shanghai；主要实时快照采于 17:24–17:30。
工作目录：/home/yc/UR3。本文为交接记录，不是实机执行授权。

## 0. 先读：本次边界与证据等级

用户最新要求是停止扩展工作，只读检查后生成本文件。不得启动新机械臂运动，不得重启或终止现有 ROS/Codex 进程，不得改变配置。**下一会话首先继承这一限制，不能把“继续任务”解释为自动恢复播放或执行旧轨迹。**

本次仅检查文件、Git、进程、网络、ROS 图和只读服务/订阅、控制器 RTDE 输出及 Dashboard 查询；解码已有录像并重算已有位置数据。没有发送关节轨迹、URScript、I/O、电机、播放、暂停、停止、解锁、TCP/负载/速度设置命令。唯一主动新增的任务文件为本文。短生命周期只读 ROS 客户端可能由 ROS 自动生成空诊断日志；原有跟踪进程继续自然追加日志，这不表示配置被修改。

证据分为：

- **实时读取**：仅证明对应时间的控制器/软件状态；恢复时必须重读。
- **落盘记录**：执行 report、CSV、录像、配置、源码和已有 README；历史数值不冒充当前状态。
- **用户报告**：实体电机转动、尺寸、净空等，没有传感器自动认证。
- **模型估算/待确认**：几何碰撞模型、相机平面映射等，不能作为物理安全保证。

最近提出但**未获同意、未规划验证、未执行**的动作：电机停止时，保持球心高度和轴向，水平移向当前 H 下方。旧估计为世界 ΔX≈−14.3 mm、ΔY≈+6.3 mm。用户随后要求交接，故不得默认执行。

## 1. 最终目标与验收标准

### 1.1 用户目标

使用现有设备：固定俯视工业相机识别亚克力板上的 H 型磁机器人；UR3 携带电机和直径 20 mm 球形永磁体在板下运动，电机旋转磁体，实现 H 的可观察、可重复轨迹运动，先完成方形或圆形，再考虑自主闭环。

机械臂/磁球走出一条轨迹，不等于 H 已走出该轨迹。网页绘制方形也不是轨迹执行。

### 1.2 验收条件

尚未约定正式边长/半径、允许误差、速度、重复次数和闭环等级，均为**待用户确认**。20 mm 方形只是已有界面参考；约 24 mm 是两步试验得到的候选边长，不是批准的目标。

最低功能验收应包括：

1. 保存 H 的完整实际轨迹、时间戳、目标路径、机械臂状态和电机操作记录；没有用预测值或手动重选跳变冒充真实位移。
2. H 连续完成用户选定的完整形状，不靠人工搬动 H 补齐各边；测量横向偏差、终点闭合误差、失锁和异常停止情况。
3. 区分“人工启停电机、分段监督的演示”与“自动闭环”；前者可作为阶段成果，不能宣称后者完成。
4. 全程全臂、完整工具及停车过程满足板底/侧板/桌面/支架/自碰撞约束。
5. 若要求无人连续闭环，必须有真实可用且验证过的电机停止接口、失锁/超时联锁；目前不具备。

不应自行承诺毫米级精度。现有相机映射的独立实物精度验证未完成。

## 2. 工作进度

### 已完成

- UR3 以太网通信、ROS 2 Humble 驱动及厂家运动学标定加载。
- 相机采集、H 黑色轮廓跟踪、浏览器预览、连续录像、人工电机事件标记、参考线/方形显示。
- 用 100 mm 探针采集桌面 W0/WX/WY，保存桌面世界坐标；测量亚克力下表面和 7.4 mm 厚度。
- 14 组标记/机器人样本拟合相机参数及相机—UR/桌面关系；保存板面映射。不是独立棋盘格完整内参验收。
- 用户确认电机到球心轴沿 tool0 −Y；球心 TCP、质量及部分外形尺寸已记录。
- 小角度找平：先下降约 2 mm，再保持法兰位置调整约 2.70°，球心随之改变。
- 后续扩大板底间隙并完成工具水平转向，使电机→球心轴约沿世界 +Y。成功记录在第 7 节；不需要再次转 90°。
- 旧轴向 +X 附近曾驱动 H 沿 +Y 前进两步；新轴向 +Y 下观察到一次主要向 −X 的位移。

### 正在进行但现已停留在诊断阶段

- 新轴向下 H 第一次动一下，保持结束位置再次试转却不前进的原因分析。
- 最新用户确认：第二次试验球形磁铁/连杆确实持续转动，H 没有前进；无需再次开机来确认这一事实。
- 下一候选诊断是改变水平相对位置，保持高度/轴向/转速不变；仍只是建议。

### 尚未开始或未完成

- 适用于当前姿态的水平对齐路径规划、完整验证及实机执行。
- 可重复的 −X/+X 推进、可控停止、转角过渡、完整方形/圆形。
- 磁场、磁化方向、可靠工作高度/水平偏移范围的辨识。
- H 真实朝向检测；当前 pose 的 orientation 是占位值。
- 相机误差的独立全工作区验证、完整工具包络和支撑物模型核验。
- 实体电机自动控制、编码器/转速/启停反馈及自动停止联锁。
- 视觉—机械臂—电机的真实自主闭环控制器。

## 3. 真实硬件、软件与运行进程

### 3.1 UR3 实时状态

17:26:46 通过控制器只读 RTDE/Dashboard 取得：

| 项目 | 核验结果 |
|---|---|
| 机器人 | UR3 / CB3；不是 UR3e |
| 控制器版本 | 3.15.7.0 |
| IP | UR：192.168.56.101；PC 有线 enp2s0：192.168.56.1/24 |
| Robot mode | RUNNING，数值 7；不表示正在执行运动 |
| Safety | NORMAL，数值 1 |
| 已加载程序 | /programs/external_control.urp |
| 程序直接查询 | running=false；programState=PAUSED external_control.urp；RTDE runtime_state=4 |
| 关节/TCP 速度 | 本次采样全部为 0 |
| TCP 偏置 | [0, −0.062, 0.041, 0, 0, 0]，位置 m，姿态旋转向量 rad |
| 控制器负载质量 | 0.32 kg |
| 控制器负载重心 | [0, −0.062, 0.041] m；仅确认设置值，不证明实际重心在球心 |
| target_speed_fraction | 1.0，即目标滑条比例 100% |
| speed_scaling | 0.0；当前程序暂停时有效缩放为零，不是允许任意速度 |

六关节按 shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3 顺序，17:26:46 读数：

~~~text
q [rad] = [-1.39302475, -1.85441763, -1.55566246,
            0.37499845,  2.97393894,  1.67647743]
actual_TCP_pose [m, rad] =
[0.05985509, -0.34126808, 0.41571949,
 -0.00608899, 1.56037575, 0.00573458]
~~~

这是 UR base 下的**活动 TCP**位姿，不能把前三项当作法兰位置。姿态是旋转向量，不是 roll/pitch/yaw 三个欧拉角。ROS /joint_states 的排列顺序不同，必须按 name 匹配，不能直接按数组下标使用。

**状态差异必须保留**：同次 ROS 订阅 /io_and_status_controller/robot_program_running 收到 true（一次 transient-local 消息），而直接 Dashboard 显示 PAUSED/running=false，ROS 速度缩放持续为 0。不能仅凭该 Bool 判断能执行；其语义/保留消息时效须在恢复前复核。本次没有为消除差异而播放程序、切换控制器或重启驱动。

17:29 左右读取 scaled controller 的 action 状态，只见历史 SUCCEEDED(4)、CANCELED(5)，未见 ACCEPTED/EXECUTING/CANCELING；这不是对未来无挂起目标的保证，播放前仍须重查。

用户曾确认 external_control.urp 只有 External Control 节点，无旧 MoveJ/MoveL；本次未读取示教器程序树，内容仍属用户报告，不能只靠文件名认定。

### 3.2 H、磁球和平台的相对位置

17:27:09 从当前关节及本机标定模型计算，数值约为：

| 项目 | 世界坐标/结果 |
|---|---|
| 球心 | X=216.892、Y=91.364、Z=431.981 mm |
| 电机→球心轴单位向量 | [0.0000241, 0.9999998, 0.0005756] |
| 轴线水平航向 | 89.999°；世界 +X 为 0°，+Y 为 90° |
| 轴线仰角 | 0.033°，近似平行桌面 |
| H 平面投影 | 同期相机约 X=202.62、Y=97.65 mm；Z 固定取板上表面 498.141 mm |
| H 相对球心投影 | 约 ΔX=−14.27、ΔY=+6.29 mm，水平相距约 15.6 mm |
| 球心到板上表面 | 垂直高差约 66.16 mm |
| 球顶到板下表面 | 490.741−(431.981+10)≈48.76 mm |
| 全臂/工具到板底最小模型间隙 | 约 27.45 mm，限制部位 wrist_3_link；不是球顶间隙 |

17:27 模型顶部世界高度（mm）：

| 几何 | 最高点 Z |
|---|---:|
| base_link_inertia | 102.238 |
| shoulder_link | 225.072 |
| upper_arm_link | 440.572 |
| forearm_link | 439.309 |
| wrist_1_link | 391.284 |
| wrist_2_link | 462.897 |
| wrist_3_link | 463.290 |
| 暂定完整工具包围盒 | 462.285 |
| 球体 | 441.981 |

最小模型左侧板、右侧板、桌面净空分别约 327.58、138.67、15.46 mm。桌面检查不把固定安装的基座接触当作运动碰撞。以上只涵盖模型中存在的几何，不包括未知立柱、线缆和杂物。

当前 Z 比转向前球心约 454.98 mm 低约 23 mm。后续诊断不能把旧高度时的驱动结果直接当作当前工况，也不能为“恢复磁力”直接抬回旧高度。

### 3.3 电机与相机

- 电机：现有步进电机 + 手动脉冲发生器；用户辨认面板单位为 rpm，最后确认设定 10 rpm。实际转速没有独立测量。65 是更早的设定，不应沿用。
- 10 rpm 名义上每圈 6 秒；没有证据证明每圈对应固定 H 位移。用户难以手动控制 1 秒，不应继续机械地要求 1 秒试转。
- 最后操作记录为停止；最新用户确认的是“刚才试验中持续转动”，不是当前实时电机状态反馈。恢复前现场确认**电机当前确已停止**。
- 正反转档的电气定义、从哪一端观察“顺时针”、电机型号/细分/脉冲比例和磁化轴仍待确认。不要把旧 +X 轴向下的顺逆时针与新 +Y 轴向直接混同。
- 相机照片标签：FLIR Blackfly S BFS-U3-50S5C-C，序列号 22165884。当前采集配置使用 Aravis/GStreamer，而非正在运行的 SpinView。
- 传感器采集 2448×2048 BayerRG8，转换为 1224×1024 BGR，目标 20 FPS；17:27 页面约 19.98 FPS，detected=true、stream_stale=false。
- 网页：http://127.0.0.1:8767/ 。GET /status 只读；命令按钮会改变跟踪/录像状态，但不会控制电机/机械臂。
- world_mapping_verified=false。图像静止抖动约 0.01 mm 不等于 0.01 mm 绝对精度。
- 当前没有正在录制的 clip；位置/诊断会话日志仍持续写入。

### 3.4 进程与 ROS 节点

以下 PID 是交接时快照，不是恢复时可盲信的固定 PID。全部保留运行。

| PID / PPID | 程序/节点 | 用途 |
|---|---|---|
| 42171 / 1306 | python3 -m ur3_magnetic_control.camera_node；/camera_node | 真实相机发布 /camera/image_raw |
| 42173 / 1306 | python3 -m ur3_magnetic_control.h_tracker_node；/h_tracker | H 跟踪、网页 8767、录像及日志 |
| 63482 / 54401 | ros2 launch ur_robot_driver ur_control.launch.py | 真实 UR 驱动 launch |
| 63509 / 63482 | ur_ros2_control_node；/controller_manager 及控制器节点 | 实机接口、125 Hz 控制循环 |
| 63511 / 63482 | /dashboard_client | Dashboard 服务，可含危险写操作，本次只读 |
| 63513 / 63482 | /ur_robot_state_helper | 状态辅助、模式服务 |
| 63515 / 63482 | /controller_stopper | 根据程序状态协调控制器 |
| 63517 / 63482 | /urscript_interface | URScript 执行接口；**不要调用 execute_script** |
| 63519 / 63482 | /robot_state_publisher | 机器人模型 TF |
| 63521 / 63482 | /trajectory_until_node | 条件轨迹接口；未授权使用 |
| 54401、80705 | Codex | 两个现存 Codex 进程；不要退出、重启或杀掉 |
| 55596 / 54401 | codex-code-mode-host | 当前工具宿主，保留 |

相机原 launch 日志记录 PID 42146，但该 launch 已不在进程表，两个子进程 PPID 为用户 systemd 1306，仍正常采集。**不要因 launch 不在就再启动一份相机/跟踪。**

UR launch 运行参数实查：

~~~text
ur_type:=ur3
robot_ip:=192.168.56.101
reverse_ip:=192.168.56.1
kinematics_params_file:=/home/yc/UR3/config/ur3_calibration.yaml
launch_rviz:=false
headless_mode:=false
initial_joint_controller:=scaled_joint_trajectory_controller
~~~

只读 list_controllers 核验：

- active：scaled_joint_trajectory_controller、joint_state_broadcaster、io_and_status_controller、speed_scaling_state_broadcaster、force_torque_sensor_broadcaster、tcp_pose_broadcaster、ur_configuration_controller、friction_model_controller。
- inactive：joint_trajectory_controller、forward_velocity_controller、forward_position_controller、forward_effort_controller、force_mode_controller、passthrough_trajectory_controller、freedrive_mode_controller、tool_contact_controller。
- 硬件组件 ur3/system 为 active；controller_manager.update_rate=125。
- 还发现 transform_listener_impl_* 辅助节点。临时 handoff_read_only_* 检查节点已自行退出，不是应保留启动的实验节点。
- **未发现当前运行的 move_group、acrylic_ceiling_guard、safety_supervisor、mock_motor 或 H 闭环控制节点。** 不要把磁铁高度限制当成已永久写入 UR 安全设置或有常驻安全节点兜底。

UR 主机监听 50001–50004；External Control 请求端口 50002。ROS_DOMAIN_ID=33，ROS_LOCALHOST_ONLY=0。本机 Wi-Fi 192.168.3.84/24 不是机械臂网卡。

软件实查：Linux 6.8.0-138-lowlatency，ulimit -r=99；ROS Humble；ur_robot_driver 2.14.0，ur_client_library 2.15.0，joint_trajectory_controller 2.54.0，MoveIt 2.5.10。驱动日志记录 Calibration checked successfully 和 SCHED_FIFO OK, priority 99。整理前 ROS2 环境说明中关于“尚在 generic 内核”的文字已过时，现已修正。

## 4. 坐标系、单位、方向与关键尺寸

### 4.1 坐标约定

- table_world：原点是带孔桌面上用户指定 W0，不是亚克力中心，也不是机器人基座原点。
- 按 /home/yc/UR3/docs/images/hardware/侧板.jpg 及用户确认：照片右侧为世界 +X，向前为 +Y；+Z 指向桌面上方，右手系。
- base：UR 控制器基座坐标。ROS 模型中的 world/base_link 不能未经转换就等同 table_world。
- tool0：本机标定模型中用于计算工具偏置的末端坐标；电机→球心轴为 [0,−1,0]，球心偏置 [0,−0.062,0.041] m。
- TCP 是 Tool Center Point。当前活动 TCP 为球心；旧 100 mm 探针 TCP 和 60 mm 标定纸 TCP 都是历史工具，不得恢复成活动 TCP。
- 旋转工具朝向（机械臂多个关节协调）与电机带磁球自转不同。绕世界 +Z 从上向下看逆时针为正；给腕部某一关节加 90°不等价于绕球心世界偏航。
- ROS/URScript 位置通常 m、角度 rad；文档/UI 多用 mm/deg；电机面板 rpm。每次接口转换显式注明单位。
- H 相机坐标 u 向图像右、v 向图像下，不能直接当世界 X/Y。H 翻滚离开平面时，映射只是轮廓投影，不是真实三维质心。

桌面标定来源：/home/yc/UR3/config/table_world_calibration.yaml。

~~~text
p_base = T_base_from_world × p_world

T_base_from_world =
[-0.999972620,  0.007397899, -0.000175681,  0.276097000]
[-0.007397883, -0.999972631, -0.000088533, -0.248236000]
[-0.000176331, -0.000087231,  0.999999981, -0.016152000]
[ 0,            0,            0,            1          ]
~~~

平移单位 m。世界 X/Y 大致与基座 X/Y 反向，不能忽略小旋转后直接改符号。

探针三点 base 坐标（m）：W0=[0.276097,−0.248236,−0.016152]，WX=[0.151335,−0.249159,−0.016174]，WY=[0.276765,−0.373809,−0.016163]。两方向长度约 124.765、125.575 mm，夹角 89.880916°。

### 4.2 尺寸

| 参数 | 数值、依据与限制 |
|---|---|
| 亚克力下表面 | 世界 Z=490.741 mm；100 mm 探针单点测高，假定板面平行桌面 |
| 亚克力厚度 | 7.4 mm，取代早期口述 5 mm |
| 亚克力上表面 | 世界 Z=498.141 mm |
| 白纸厚度 | 配置 null，待确认；H 接触面如含纸层须计入误差 |
| 侧板内面 | 世界 X=−140 mm、+590 mm；用户确认量的是内表面 |
| 两侧净宽 | 730 mm；模型按无限 YZ 平面限制，不代表所有支架均已建模 |
| 磁球直径/半径 | 20/10 mm |
| 电机端面 | 43×43 mm |
| 电机轴向长度 | 42 mm |
| 固定件厚度 | 沿电机轴 8 mm，中面经过法兰中心，用户确认 |
| 固定件另外两维 | 用户称与电机差不多，无精确最大外形，待确认 |
| 条件电机外形 tool0 | min=[−21.5,4,19.5]、max=[21.5,46,62.5] mm；placement_confirmed=false |
| 暂定整工具包围盒 tool0 | min=[−30,−75,−5]、max=[30,50,70] mm；不是实测外形 |
| 工具包络完整性 | verified_encloses_all_rigid_parts=false，未包括线缆 |
| H 自身尺寸/质量/磁化分布 | 未找到可靠已确认参数，待确认 |

相机标定标记：黑色正方形外边长 100 mm，不含白边；纸面与法兰平行，名义中心距法兰 +Z 60 mm；实物解码 DICT_ARUCO_ORIGINAL ID 450，不可从 aruco-9.svg 文件名猜 ID。

### 4.3 相机映射

权威文件：/home/yc/UR3/config/camera_robot_calibration.yaml。已有内参：

~~~text
K = [[3625.088677, 0, 1200.123248],
     [0, 3630.425985, 1067.173693],
     [0, 0, 1]]
D = [-0.274982454144, 1.568575307653, 0, 0, 0]
图像尺寸 = 2448×2048
~~~

14 样本内参重投影 RMS 0.290781 px；机器人—相机重投影 RMS 1.234756 px、max 3.302854 px；TCP/标记位置 RMS 0.382017 mm、max 0.945815 mm；sample_010 ArUco 解码失败有记录。拟合误差不是独立定位精度。

使用已有 PlaneMapper：先将半分辨率坐标恢复全传感器像素，去畸变，再映射到 Z=0.498141 m 的板面。不要对原始畸变像素直接乘单应矩阵，也不要把桌面 Z=0 的单应关系用于板上的 H。相机不必正对世界原点，但任何相机/镜头/板面变化都要求复核映射。

## 5. 不得违反的安全约束

1. **所有运动时刻，所有 UR link 碰撞几何的最高点及完整工具最高点，均须低于亚克力实际下表面，并保留经评估的余量。** 检查的不只是关节原点、link 中心、法兰或 TCP。球心下降不自动保证肘/腕下降。
2. Z=490.741 mm 是物理模型上限；当前各 link 高度不是固定上限。单点平面假设、倾斜、挠度和标定误差必须考虑，不得拿模型未接触等同实际安全。
3. 检查全路径、控制器真实插值、关节限位/跳变、自碰撞、工具碰撞、桌面、两侧板、立柱、连接件、线缆、现场杂物与停车过程，而非只检查起终点。
4. geometry_draft/status=incomplete_draft，execution_allowed=false；侧板 diagnostic 配置也 execution_allowed=false。**禁止为让脚本通过而把 false 改 true。**
5. **2026-09-20 起的唯一全局净空策略：板底≥5 mm、左右侧各≥10 mm、桌面≥10 mm。** 正常轨迹、停车模型和在线检查全部读取 `config/ur3_system.yaml:safety.clearance_policy_m`，不得在运动代码或命令行单独覆盖。历史报告中的 3/18/20/23/100 mm 数值只描述旧审核，配置哈希已变化，不能直接重放，必须重新规划。
6. 最近专项 yaw 模型检查通过不意味着实际工具包络已全面核验；不得把 scoped review 复用到水平追踪或抬升上。
7. 当前程序暂停不等于驱动退出或 UR 断电。保持现场急停可用、人位于运动区域外；不要在人手伸入板下、调整 H、量间隙时运动。
8. 网页标记、停止录像、录像超时、失锁、关闭网页、停止 ROS 都**不保证电机停止**。当前电机必须实体操作，无自动联锁。
9. 不擅自回零、切换 TCP/负载、设置速度滑条、解除保护停、加载/播放旧程序、发送直接 URScript、切换控制器、同时启用多个控制路径。
10. 不执行“速度无限制/所有阈值 100°/s”。现有源码实测速度中止阈值为 lower=10°/s、yaw=9°/s，不是规划速度、不是 UR 原生安全限速。规划仍限 lower≤1.1°/s、yaw≤8°/s、诊断下降≤0.2°/s，规划加速度≤10°/s²。
11. 监测数据过期、数值无效、跟踪误差超限、运动方向不符、出现碰撞/接触迹象时不得继续下一段或自动重试。真实运动时需要事先约定并验证停止方案，不能在危险中才设计停止。
12. 恢复任务前协调两个 Codex 会话，避免同时发指令；本次不通过杀进程取得控制权。

## 6. 文件清单、修改目的与 Git diff

### 6.1 Git 核验结果

/home/yc/UR3 根目录没有 .git；git rev-parse/status/diff 均不能给出根目录变更基线，报“不是 git 仓库”。**不能写成“工作区 clean”，也不能凭文件 mtime 判定是谁改了哪些行。**

只发现嵌套仓库 /home/yc/UR3/vendor/Universal_Robots_ROS2_Driver-2.14.0/.git：

- HEAD eca2357c9fee6e3403ab5aacc5ca4bcc3fe48279，tag 2.14.0。
- git status --short：M ur_calibration/src/calibration_correction.cpp。
- 未暂存 diff：1 file changed, 3 insertions(+)；已暂存 diff 为空。
- 绝对路径 /home/yc/UR3/vendor/Universal_Robots_ROS2_Driver-2.14.0/ur_calibration/src/calibration_correction.cpp：在 pipeline.run() 前补 pipeline.init()，并有两行注释，适配 ur_client_library 2.15 将初始化拆出的 API。
- 本次未修改或提交该仓库。

下表是已存在的任务实现及用途，并非全部由 Git 证实的新增/修改清单。除上述嵌套 diff 与本次新增本文外，历史逐行修改范围/作者在缺乏基线时**待确认**。

### 6.2 关键实现与配置（绝对路径）

| 路径 | 用途/当前实现 |
|---|---|
| /home/yc/UR3/ros2_env.sh | source Humble/工作区，设置 domain 33、包索引 |
| /home/yc/UR3/config/ur3_calibration.yaml | 本机厂家运动学，hash=calib_18089309548208516197 |
| /home/yc/UR3/config/table_world_calibration.yaml | 探针三点、世界变换、板底及板厚 |
| /home/yc/UR3/config/camera_robot_calibration.yaml | 相机内外参、板面单应性和拟合质量 |
| /home/yc/UR3/config/camera_tool_marker.yaml | 标定纸安装/标记数据，非当前 TCP |
| /home/yc/UR3/config/ur3_system.yaml | 系统参数、球心偏置/负载/轴向；其中 target_fps=30 不是当前采集配置的 20 FPS |
| /home/yc/UR3/config/acrylic_side_boundaries.yaml | 两侧板内面，只用于离线诊断 |
| /home/yc/UR3/config/magnet_tool_geometry_draft.yaml | 电机/固定件测量与未验证工具包络 |
| /home/yc/UR3/robot/ur3_realtime_monitor.py | 独立 RTDE 输出读取；无运动输入 |
| /home/yc/UR3/robot/ur3_linear_move.py | 旧直接 URScript MoveL；当前板下实验不要使用执行模式 |
| /home/yc/UR3/robot/calibrated_moveit.launch.py | 使用本机标定的 MoveIt；目前没有运行，不能擅自 launch |
| /home/yc/UR3/robot/guarded_level_workflow.py | 历史找平工作流，包含执行能力，不是通用 XY 跟随器 |
| /home/yc/UR3/robot/preview_yaw_geometry.py | RTDE 只读、几何 IK 扫描及球心固定偏航计算 |
| /home/yc/UR3/robot/preview_yaw_clearance.py | 世界平面、整臂/工具、FCL 自碰撞诊断 |
| /home/yc/UR3/robot/prepare_yaw_pilot.py | 原高度 5°/完整偏航计时草案 |
| /home/yc/UR3/robot/prepare_lower_yaw.py | 下降后偏航离线 IK/时间参数/插值审查 |
| /home/yc/UR3/robot/run_checked_yaw.py | 原高度偏航前置检查/执行器；曾因几何与停车依据不足而保留，不适合现在盲重放 |
| /home/yc/UR3/robot/bounded_lower_yaw.py | 一次只执行 lower 或 yaw；实测速度、反馈、几何、独立 RTDE 监测；lower 阈值现 10、yaw 9°/s |
| /home/yc/UR3/robot/record_rtde_outputs.py | 独立输出流记录及过期/错误检查，不发运动或 RTDE 输入 |
| /home/yc/UR3/robot/offline_collision/check_self_collision.cpp | 内存 MoveIt/FCL 检查器，附工具箱/正碰撞对照 |
| /home/yc/UR3/robot/offline_collision/sample_timed_trajectory.cpp | 调用已安装 JTC 插值库离线采样 |
| /home/yc/UR3/robot/offline_collision/CMakeLists.txt | 两个独立离线检查器构建 |
| /home/yc/UR3/camera/start_h_tracking.sh | 恢复采集 ROI/格式后启动 H 视觉；会改易失相机参数，当前不要重复运行 |
| /home/yc/UR3/camera/h_vision.launch.py | camera_node + h_tracker_node，无机械臂/电机控制 |
| /home/yc/UR3/camera/start_flir_camera.sh | 旧全分辨率预览启动器；不要与现有采集争用相机 |
| /home/yc/UR3/camera/extract_marker_black_square.py | 提取实物标记黑框 |
| /home/yc/UR3/camera/solve_camera_robot_calibration.py | 从保存样本估计内参与外参，打印结果供审查，不自动覆盖配置 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/config/flir_vision.yaml | 当前进程实际读取的 20 FPS/Aravis 配置 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/camera_node.py | 相机图像采集发布 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/h_tracking.py | PlaneMapper、暗目标锁定、参考路径 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/h_tracker_node.py | 检测发布、会话日志和 UI 命令处理 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/h_preview_web.py | 8767 网页、显式按钮事件监听；修复 command 名称冲突，显示后台错误 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/h_recording.py | MJPG、真实接收时间戳、容量/时长边界，无电机停止能力 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/ceiling_geometry.py | URDF/FK/各 link 碰撞面高度计算 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/acrylic_ceiling_guard.py | 几何监控实现，当前未运行，不能当作常驻联锁 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/cartesian_line_move.py | 历史 MoveIt 直线规划/有条件执行，不等于完整磁驱动 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/cartesian_orientation_move.py | 姿态运动实现，使用前重新审查 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/level_magnet_axis.py | 轴线找平辅助 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/mock_motor.py | 模拟电机，不是实体电机接口 |
| /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control/safety_supervisor.py | 原框架监控；当前未启动，也不能替代实体联锁 |
| /home/yc/UR3/docs/history/H机器人方形试验流程_20260919.md | 历史分阶段操作说明，含已过时工况，不作为最新运动授权 |
| /home/yc/UR3/robot/README.md | 找平/旧运动说明；4.43 mm 间隙是历史值 |
| /home/yc/UR3/camera/README.md | UI、采集、跟踪与测试说明 |
| /home/yc/UR3/docs/setup/ROS2.md | ROS 2 环境与安装说明 |
| /home/yc/UR3/docs/history/HANDOFF_H_MAGNET_PATH_20260919.md | 本次交接文档（历史归档） |

机械臂测试已统一移至 `/home/yc/UR3/robot/tests/`；视觉/UI 测试位于
`/home/yc/UR3/ros2_ws/src/ur3_magnetic_control/test/`。

### 6.3 核验时关键文件 SHA-256

用于发现交接后变化，不作为实机执行批准：

~~~text
config/acrylic_side_boundaries.yaml
  ba86aebafbe762173a62ed8f53bebaaf262450a775f3a49142a478cd6bf3568c
config/table_world_calibration.yaml
  c467854341d24aea691f9287b383f1d6eea06f3e564096b85af316fdffd0f4ba
config/magnet_tool_geometry_draft.yaml
  720fa4c41c6f76f200abdca624b291329851298617c1e7eefa151b7ba5ef13a7
config/camera_robot_calibration.yaml
  5bcd0c123a0b08766d28a10804963257639177656a8ab98e4210c897fd84441d
config/ur3_calibration.yaml
  c073bb8f4b019474b8fc96b9ac9ad7abed2d92b404c1bd3d267e369f54501faa
config/ur3_system.yaml
  36a38efc3648a8d1444054e98bc635dd33f64d0a11ee5ef171c10eb4187b7ba4
robot/bounded_lower_yaw.py
  537606474eac5fb281991e8a9c2f46c4d4608e698421f7a5a6abd4e4c3b724b9
robot/prepare_lower_yaw.py
  1507b2f38f4c08eaa7bce83f93168b9397850748a378d9cc6e64763d2d9b4cb9
robot/record_rtde_outputs.py
  a08850570e36d0867cd414b209fe8a32716ca80a26f22a4fd069bec46f42e290
~~~

所有相对路径在本小节都以 /home/yc/UR3 为根。

## 7. 已有命令、采样、成功/失败验证及证据

### 7.1 本次交接实际执行的只读检查

- date -Is、pwd、ls、rg/find、sed/head/tail；查看任务文件、现有日志和报告。
- git rev-parse/status/diff；根目录失败（非 Git），嵌套 vendor diff 成功。
- ps、ip -brief address、ss -ltnp、uname -r、ulimit -r、dpkg-query、df/du、sha256sum。
- source /home/yc/UR3/ros2_env.sh；ros2 node/topic/service list --no-daemon。
- 临时 rclpy 客户端：订阅 joint_states、robot_program_running、speed_scaling、h_robot/detected 和 scaled action status；调用 ListControllers、ListHardwareComponents、GetParameters(update_rate)；没有调用设置/运动服务。
- 独立 RTDE 仅 SETUP_OUTPUTS / START / PAUSE 当前输出连接；读取 q/qd/TCP/offset/payload/cog/speed/runtime。此 PAUSE 只是本客户端输出流，不是暂停机器人程序。
- Dashboard 只发 robotmode、safetystatus、running、programState、get loaded program。
- GET http://127.0.0.1:8767/status 使用 curl --noproxy '*'，避免代理。
- Python/OpenCV 只读解码第二个新轴向录像，并按 frame_times.csv 与 positions.csv 的 host_frame_time_ns 精确匹配，计算首 3 s/末 5 s 中位数。

本次检查自身的非硬件失败：ros2 action list 不支持 --no-daemon；一次服务短超时；摘要脚本误取不存在的 HardwareComponentState.plugin_name，已改为现有字段后只读查询成功。驱动日志追加过 list_controllers 回复超时告警。没有据此重启服务或推断硬件故障。

### 7.2 历史构建/测试

- 当时 `ros2_ws/log/` 中保存过本地标定包和控制包的构建日志；这些可再生成日志已在
  2026-09-20 目录整理时清理，不影响 `src/`、`build/` 或 `install/`。
- /home/yc/UR3/robot/offline_collision/build/check_self_collision 和 sample_timed_trajectory 已存在。
- /home/yc/UR3/robot/planning_checks/yaw_pilot_5deg_20260919/README.md 记录当时“14 项离线单元测试通过”，但不是当前所有文件的完整测试日志。
- 本次 AST 统计：`robot/tests/test_*.py` 共 47 个 test_* 方法；视觉测试 11 个、独立浏览器测试 3 个。**方法数量不是通过数量。当前 47 项全部通过的原始运行日志未在已查工作区找到，待确认；此次没有重跑测试、构建或启动测试浏览器。**
- 浏览器测试需要独立 Firefox profile/Marionette；不得附着用户现用浏览器。未设置 H_UI_TEST_MARIONETTE_PORT 时会 skip，不能把 skip 宣称功能通过。
- 精确历史 shell 命令若没有 command.log 或进程命令行，待确认，不凭 report 路径反推“已运行的原命令”。

### 7.3 机械臂运动记录

下表目录均在 /home/yc/UR3/robot/planning_checks/，不要重放里面的轨迹。

| 记录/证据 | 结果 |
|---|---|
| /home/yc/UR3/robot/planning_checks/level_magnet_axis_20260919/level_execution_20260919.json | 找平完成；FK/TCP 位置误差约 0.0723 mm，角度误差 0.01087°；最终轴倾角 0.01181°，当时最小模型间隙 4.428 mm |
| yaw_positive_y_with_sides_20260919_checked/summary.json、self_collision.csv | 原高度几何扫描 921 点；左右间隙约 196.09/137.94 mm，板底约 4.409 mm；仅离散模型检查 |
| yaw_pilot_5deg_20260919/pilot_audit.json | 5°原高度草案 2876 点，23 s 含停留；未作为实机完成记录 |
| yaw_full_smooth_45s_20260919_checked/live_preflight_held.json | motion_sent=false；工具包络/立柱/物理间隙/停车验证不足，原高度执行被保留 |
| lower20_execution_20260919/report.json | 曾发送下降，因 Unexpected measured velocity 中止；未记录触发瞬时数值，不能补猜 |
| lower20_execution_20260919/post_stop_verification.json | 中止后 376 帧速度为 0；实际只下降约 2.947762 mm，板底间隙约 7.384 mm，并非下降了 20 mm |
| lower3_diagnostic_execution_20260919/report.json | RTDE 记录器 timed out，0 samples，motion_sent=false |
| lower3_diagnostic_execution_20260919_v2/report.json | 随后诊断下降成功，实际约 2.999579 mm，1841 个独立 RTDE 样本，无 recorder error |
| lower14_execution_20260919/report.json | 中止：Measured wrist_1_joint speed 1.619110 deg/s exceeds 1.500 deg/s limit；failure_feedback 和独立 RTDE 同步记录，stop_reply=Stopped |
| lower14_execution_20260919/final_stopped_state.json | 关节/TCP 速度为 0，安全 NORMAL，running=false；不得原轨迹自动重试 |
| lower10_finish_yaw_20260919/audit.json | 新起点重新规划：再降 10 mm，转约 91.692788°；8230 插值检查采样，0 碰撞采样、0 关节越界；该 audit 仍 execution_allowed=false |
| lower10_finish_execution_20260919/report.json | completed=true；实际下降 10.010619 mm，2454 独立 RTDE 样本；最大实测关节速度约 1.174164°/s |
| finish_yaw_execution_20260919/report.json | completed=true；5878 在线检查样本、6046 独立 RTDE 样本；最大实测关节速度 7.466584°/s；最小在线模型板底间隙 27.369050 mm；最终 heading=89.995872°、elevation=0.032356° |

最后成功草案 phases：lower=[1,16] s、hold=[16,18]、yaw=[18,63]、final_hold=[63,65]。实际执行分两个单独阶段，没有自动串接和返回原高度。名义下降运动 15 s、偏航运动 45 s，不含各段停留。

最后 yaw 离线检查 5973 阶段样本、8226 停车敏感性样本；工具每面放大 5 mm 后最小工具板底模型间隙约 23.387 mm。停车模型用延迟 0/0.1/0.2 s 和减速度 1/4 rad/s²；是敏感性计算，不是实测制动距离或安全认证。

下沉中止触发的是本地反馈阈值，尤其 lower14 同时有 ROS/直接 RTDE 证据，不能简单称为 ROS 假数据，也未证明机械硬件故障。用户要求后，lower 实测中止阈值改为 10°/s，yaw 保持 9°/s；源码/测试验证了目前设置，但没有提高规划速度到 100°/s。

驱动成功/取消记录：
/home/yc/.ros/log/ur_ros2_control_node_63509_1789805193574.log；
/home/yc/.ros/log/2026-09-19-16-06-32-826715-111-63482/launch.log。
其中可见两个取消和后续 Goal reached, success!；不需要重做运动来验证这些历史结果。

### 7.4 H 录像与分析

统一会话根目录：
/home/yc/UR3/camera/tracking_sessions/20260919_142716_351773。

会话中 positions.csv、events.jsonl、diagnostics.jsonl 持续追加；每个 clip 的 metadata.json、frame_times.csv、unannotated.avi 保存该段。视频是半分辨率有损 MJPG，不是原始 RAW；时间以接收时间戳为准，不把 AVI 标称 20 FPS 当精确时钟。

| clip 子目录 | 证据及结果 |
|---|---|
| clip_20260919_143620_059068 | analysis.md：1312 帧；用户报告 10 rpm、与此前相反转向；ΔX=+9.114、ΔY=+76.896 mm；后段转向/侧漂，电机实际运行时间未知 |
| clip_20260919_150356_459073 | analysis.md：524 帧；用户明确实际约 6 s，实体/网页人工同步；两步 ΔX=+2.745、ΔY=+23.444 mm；不是 1 s 运动，也未测到翻滚中停止的制动距离 |
| clip_20260919_151413_159058 | analysis.md：768 帧；两步 ΔX=+1.892、ΔY=+23.440 mm；两次相近不能当固定步长模型 |
| clip_20260919_170931_359100 | analysis.md：新轴向 +Y 后首次，674 帧均 tracked；ΔX=−12.645、ΔY=+5.910 mm；净位移 13.958 mm；H 自身明显转向后发生一次位移 |
| clip_20260919_171504_569393 | 最新重复试验，本次重新核验 618 CSV 行/618 解码帧/618 tracked；没有单独 analysis.md，结论记录于本交接文档 |

最新重复试验详细数值（同一 host_frame_time_ns 关联 positions.csv，前 3 s/末 5 s 中位数）：

~~~text
录像跨度             30.949927069 s
最大接收帧间隔        0.101664371 s
起点 XY              [202.623605, 97.645100] mm
终点 XY              [202.602915, 97.645876] mm
净变化               [−0.020690, +0.000777] mm
净距离               0.020705 mm
全片距起点最大变化    0.042647 mm
~~~

属于视觉波动量级，没有有效推进；不代表获得 0.02 mm 精度。

最新用户确认“确实持续转动，H 没前进”，排除该次“电机根本没转”的解释。没有测磁场，不能由球体旋转直接证明所需方向/幅值的交变磁场成立。

最新网页启动/停止标记相隔约 11.301366 s，来源为 operator_click_not_hardware_feedback；用户没有确认本次精确实际运行时长。上次新轴向试验用户说不超过 6 s，而网页标记相隔 7.901670 s。保留两种来源，不擅自校正或把网页间隔当实体时长。

目前未证实停止源于磁力不足、偏移、H 姿态、摩擦、磁化方向或电机失步中的哪一项。相机平面位置日志不能单独识别这些机制。

## 8. 技术决策、依据、假设与不采用的方案

- 保留现有设备，先解决稳定推进/换向，再做形状。没有购买或接入 OpenArm/CAN 伺服；换电机不是当前已证实的必需条件。
- 采用桌面世界系，H 在板面具有固定非零 Z；不要求相机视野中心与世界原点重合。已有外参/板面映射承担转换。
- 保留现有相机参数作为初始映射，不因用户没做独立棋盘格就写“完全没有内参”；但也不把有限标记拟合当最终内参验收。
- 机械臂转向使用本机标定 FK + SciPy least_squares 连续 IK、MoveIt/FCL 离线碰撞、已安装 JTC 插值库及 ROS FollowJointTrajectory；不是单关节硬转 90°。
- 板下原高度只余约 4.4 mm，选择先下降再定球心水平偏航；不自动回到原高度。
- 完整工具未精测时，只在特定已审查下降/定球心 yaw 上做保守包络、5 mm 放大和停车敏感性核验；不把模型元数据改成已验证。
- 不采用“让 H 自然停下来作为正常终点”、按秒数换算固定距离、仅凭两个约 23.4 mm 结果推定每圈步长。
- 不要求用户为了确认轴是否转再开电机；该信息已由用户回答。
- 不采用原地连续补转/加速/盲目抬磁铁排障，也不直接启动完整方形。
- 暂不开发/启用常开自动追踪。先做单变量、短距离、有停止条件的相对位置试验，且仍需新授权。
- 当前假设包括：板面固定且近平行桌面、工具偏置测量可用、暂定工具箱近似包住刚性工具、相机未挪动、现场线缆净空足够。这些不是永久保证。
- 用户过去要求“尽快/放宽速度”没有取消安全几何约束，也不覆盖最新“仅交接”的明确限制。

参考论文文件：
/home/yc/UR3/docs/references/SIYU_LIU_Adaptive_Path_Planning_for_Autonomous_Navigation_of_Reconfigurable_Magnetic_Microswarms.pdf。
设备相似不等于 H 的磁化/摩擦/运动机理与微粒群相同；不得直接套用论文参数。本文未为交接重新开展文献研究。

## 9. 已知问题、风险、阻塞与待用户确认

按优先级：

1. **授权边界**：仅交接有效；水平对齐建议尚未获同意。恢复时先确认用户希望只读诊断、离线开发还是具体实机动作。
2. **程序状态**：当前 PAUSED、有效缩放 0，ROS Bool true；不自动播放，不假定 active controller 就可运动；重查 action 是否有未完成目标。
3. **H 不持续推进**：第二段 618 帧无有效位移，球体确实旋转；机制未定。保持新工况数据，不重复无信息试转。
4. **场强/间距变化**：为安全下降约 23 mm 后，球心到 H 平面约 66 mm，较早成功 +Y 两步工况不同；水平偏移约 16 mm，但尚未证明因此停止。
5. **模型不完整**：固定件最大范围、螺钉/接插件/杆体、支撑立柱、线缆以及平台倾斜/挠度未完成实测核验。用户称线缆不受影响，不等于无限运动范围保证。
6. **真实 CoG**：控制器实际设在球心；物理负载重心是否正确，待确认，不能因质量 0.32 kg 较小就忽略。
7. **视觉可用性**：只有轮廓追踪，无 H 分类/朝向；平面假设不适用于翻滚离面三维位置；world_mapping_verified=false。边缘 ROI、反光和目标变形可能导致失锁。
8. **电机无自动停机**：未知型号/说明书/控制接口、磁化轴、真实 rpm/相位；当前设定与停止状态须现场确认。UI 操作绝不是实体反馈。
9. **验收未量化**：方形边长/圆半径、容许误差、是否允许人工电机启停、重复次数待约定。
10. **并发控制风险**：两个 Codex 进程存在；协调唯一操作者，不擅自停止别人的会话或驱动。
11. **测试记录缺口**：47 项机器人测试源码存在但完整通过日志待确认；有 14 项旧 README 记录，不等于全套当前代码结果。
12. **现场不通过软件推断**：当前人手是否撤离、H 是否被搬动、纸是否皱/有摩擦变化、电机当前停机、球形磁铁固定可靠程度均需现场确认。

## 10. 下一位 Codex 的精确步骤（按优先级）

以下是接手路线，不是在本文生成时继续执行。每个可能扩展任务/修改文件/运动的步骤，都受用户新授权限制。**不允许从 P0 自动串接到 P5。**

### P0：读取交接并核对现存运行环境

- 前置：用户仅让接手/查看，尚无运动授权。
- 建议命令：

~~~bash
cd /home/yc/UR3
sed -n '1,260p' /home/yc/UR3/docs/history/HANDOFF_H_MAGNET_PATH_20260919.md
sed -n '261,520p' /home/yc/UR3/docs/history/HANDOFF_H_MAGNET_PATH_20260919.md
sed -n '521,900p' /home/yc/UR3/docs/history/HANDOFF_H_MAGNET_PATH_20260919.md
source /home/yc/UR3/ros2_env.sh
ps -eo pid,ppid,stat,args | rg 'ros2|ur_ros2_control|camera_node|h_tracker|codex'
curl --noproxy '*' --max-time 5 -s http://127.0.0.1:8767/status
git -C /home/yc/UR3/vendor/Universal_Robots_ROS2_Driver-2.14.0 diff --stat
~~~

- 预期：已有驱动与两个视觉节点，UI 可读；无需 launch。
- 停止条件：服务缺失、多个采集器/执行器、配置或场景变化、有人在操作机械臂。先报告，不“自动修好”或杀进程。
- 真实机械臂运动：这些命令不会主动触发。

### P1：重新读取控制器/ROS 状态

- 前置：只读访问可用；确认不同时连接多个不明控制程序。读数仅用于诊断。
- 建议命令：

~~~bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 /home/yc/UR3/robot/ur3_realtime_monitor.py --duration 2 --display-hz 1
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 - <<'PY'
import socket
for command in ('robotmode', 'safetystatus', 'running', 'programState', 'get loaded program'):
    with socket.create_connection(('192.168.56.101', 29999), timeout=2) as sock:
        stream = sock.makefile('rwb', buffering=0)
        stream.readline()
        stream.write((command + '\n').encode())
        print(command, stream.readline().decode().strip())
PY
~~~

需要 ROS 控制器列表时，不用 daemon 重启解决发现延迟：

~~~bash
source /home/yc/UR3/ros2_env.sh
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 - <<'PY'
import rclpy
from controller_manager_msgs.srv import ListControllers
rclpy.init()
node = rclpy.create_node('handoff_read_only_controller_check', enable_rosout=False)
try:
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('控制器查询不可用；只报告，不启动或重启')
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    if not future.done() or future.result() is None:
        raise RuntimeError('只读查询超时；不自动修复')
    print([(c.name, c.state) for c in future.result().controller])
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
~~~

- 预期：静止、安全 NORMAL；当前暂停不需要“恢复”为运行。动作状态也需读 /scaled_joint_trajectory_controller/follow_joint_trajectory/_action/status，拒绝在有未知活动目标时播放。
- 停止条件：非零未知运动、保护停/故障、状态过期、程序不明、命名空间/domain 不符。不发送 stop/unlock/play 来让检查通过；若现场有紧迫危险，提示现场用既定急停方案。
- 真实机械臂运动：上述命令不主动触发。RTDE 输出流 start/pause 不等于程序播放/暂停。

### P2：固定诊断基线，确认下一步授权

- 前置：P0/P1 完成；用户确认电机停、H 未再移动、场景未改变。
- 建议命令：GET /status；读取第 7.4 节 clip 的 metadata.json、frame_times.csv、会话 positions.csv。不要点击 reset/path/record 等按钮来“测试按钮”，避免改现场状态。
- 预期：能够说明 H 当前投影、球心世界位置、高差、偏移和证据有效性。
- 向用户提出的选择：先离线检查“电机停止、固定 Z 和轴向的水平对齐”是否可行；不是要求马上播放 External Control。
- 停止条件：用户未授权新任务、H 未稳定或不是同一目标、world 映射变化、实物净空不能确认。
- 真实机械臂运动：无。

### P3：若获准，离线准备新的水平对齐规划

- 前置：明确获准离线工作；新鲜状态和现场几何确认；目标由新测量计算，不能照抄 −14.3/+6.3 mm。
- 建议先读：

~~~bash
sed -n '1,245p' /home/yc/UR3/robot/prepare_lower_yaw.py
sed -n '1,235p' /home/yc/UR3/robot/bounded_lower_yaw.py
sed -n '1,180p' /home/yc/UR3/robot/offline_collision/README.md
~~~

- 要求：使用本机标定及球心偏置，保持现有球心 Z 和法兰姿态，连续 IK；检查全臂/工具/平台/侧板/自碰撞、完整 JTC 插值、关节速度/加速度、误差与停止过程。保存到新的未占用目录，不覆盖历史证据。
- **目前没有已审查、可直接执行此次 XY 对齐的命令。待实现并审查，不能拿 bounded_lower_yaw 的 --stage lower/yaw 代替。** 该脚本的水平漂移约束只有 0.7 mm，设计上会拒绝约 16 mm XY 运动；不得仅删除该约束来放行。
- 可复用离线检查器，但必须先读其输入 schema；不能把旧 timed_path_NOT_AUTHORIZED.json 直接送机器人。
- 预期：只得到新动作审查报告、预计净空/速度/偏移和未解假设，execution_allowed 保持 false。
- 停止条件：任一约束失败、IK 分支跳变、FCL 阳性对照失败、配置不一致、工具/立柱未确认。报告并停止，不切换到无检查接口。
- 真实机械臂运动：纯离线实现/运算应无；新代码必须确认没有发布器/动作发送或真实硬件初始化副作用。

### P4：离线回归与运动前审核

- 前置：用户允许软件验证；保证测试仅离线，不连接用户浏览器或真实命令接口。
- 建议命令（未来需要时执行，本次未运行）：

~~~bash
source /home/yc/UR3/ros2_env.sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/yc/UR3/robot:$PYTHONPATH /usr/bin/python3 -m unittest discover -s /home/yc/UR3/robot/tests -p 'test_*.py' -v
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s /home/yc/UR3/ros2_ws/src/ur3_magnetic_control/test -p 'test_h_vision.py' -v
~~~

- 预期：记录实际通过/失败/跳过数量、代码/配置哈希；新 XY 功能必须补自己的越界/失效测试。旧 47 项通过不证明新路径安全。
- 停止条件：失败或依赖问题；不自动安装升级/重启当前环境，不降低门限凑通过。
- 真实机械臂运动：现有这些测试源码设计为离线；新加测试必须重新审查。视觉测试可能启动独立临时本地 HTTP 服务/写临时文件，不操作现有服务。

### P5：只有具体动作获准后，才考虑水平对齐实机单段

- 前置：P3/P4、完整清单（第 11 节）通过；用户确认此次具体位移/速度/工况及现场监督；电机停止；无未知活动 action；当前程序状态已核对。
- 建议命令：**暂无可直接执行的安全命令，待新 XY 执行器和审查报告完成后单独列出并取得许可。** 不提供旧动作 --execute 作为捷径；不在本交接恢复流程里自动调用 Dashboard play。
- 预期：仅一个限定 XY 动作，Z/轴向保持，持续记录控制器及相机，结束核验静止。不自动串接电机或下一边。
- 停止条件：反馈陈旧、H 异常被静磁场牵动、几何/速度/跟随误差不合格、人员进入、任何接触迹象。即使电机停，移动永磁体也可能影响 H；不得失锁后盲目追移。
- 真实机械臂运动：**会**；目前未授权、未执行。

### P6：对齐后再讨论一次受控驱动试验及形状实验

- 前置：完成对齐后重新确认 H 位置；用户同意单次试验；实体电机停止办法明确；人员监督。
- 建议命令/操作：网页开始录像，确认帧数增长；现场按约定实体启停并同步点击事件标记，保存停止后观察段。不存在已验证的终端电机启停命令。
- 预期：检验“仅改变水平相对位置是否恢复推进”，不是预设会成功。
- 停止条件：一次试验仍无有效推进、失锁、异常翻滚/接近边缘。停止后分析，不连续补转、加速或自动抬球。
- 真实机械臂运动：试验中保持机械臂静止；**电机/磁球及 H 会运动**。
- 之后才决定工作高度/磁化方向/摩擦等辨识方案。直行、停止和转角未可重复前，不实施方形/圆形闭环。

## 11. 最小恢复命令与运动前清单

### 11.1 当前进程仍存在时：无需启动服务

~~~bash
cd /home/yc/UR3
source /home/yc/UR3/ros2_env.sh
curl --noproxy '*' --max-time 5 -s http://127.0.0.1:8767/status
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 /home/yc/UR3/robot/ur3_realtime_monitor.py --duration 2 --display-hz 1
~~~

这就是目前最小接手入口；结合 P1 的 Dashboard 查询。浏览器查看既有页面即可，不需要重启 Codex、ROS daemon、相机或 UR 驱动。

### 11.2 仅未来确认相关服务确已退出且用户另行批准恢复时

以下是**冷启动参考，当前禁止执行**。两项各自在独立终端启动，不把它们和播放/运动串接：

~~~bash
# 真实 UR 驱动；连接真实硬件，不是模拟。
source /home/yc/UR3/ros2_env.sh
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 robot_ip:=192.168.56.101 reverse_ip:=192.168.56.1 \
  kinematics_params_file:=/home/yc/UR3/config/ur3_calibration.yaml \
  launch_rviz:=false headless_mode:=false \
  initial_joint_controller:=scaled_joint_trajectory_controller
~~~

~~~bash
# 仅相机与 H 跟踪；会设置相机易失采集参数、创建新会话。
bash /home/yc/UR3/camera/start_h_tracking.sh
~~~

驱动启动不是具体运动授权；现有机器人若程序已在运行或有其它客户端，重新连接也可能产生控制影响，故冷启动前必须现场审查。相机脚本只检查部分旧预览程序，不足以排除已有 camera_node，先查进程/8767/相机占用。不要自动启动 mock_system、旧 flir_vision 的绿色检测器、MoveIt 或 SpinView 混入本实验。

### 11.3 任何真实动作前必须逐项确认

- [ ] 用户已解除“只交接”限制，并批准这一次明确方向、距离/角度、速度、停止方式；不是泛化使用旧许可。
- [ ] 唯一操作者，其他 Codex/脚本没有并发命令；没有未知 ACCEPTED/EXECUTING/CANCELING 目标。
- [ ] 电机实体停止；现场人员退出区域，急停可及；H/线缆/工具固定/杂物/支架状况确认。
- [ ] UR3 型号和 IP 正确；程序树仍只有 External Control；不得回零或执行旧 MoveJ/MoveL。
- [ ] Dashboard、RTDE、ROS 状态新鲜且一致可解释；安全 NORMAL；暂停状态不自动恢复。
- [ ] 活动 TCP=[0,−62,41] mm、姿态偏置 0，负载 0.32 kg；重心真实性核验或明确处理，不能偷偷改配置。
- [ ] 本机 calibration hash 正确；FK 与实时 TCP 一致；关节数组按名字映射；m/mm、rad/deg 正确。
- [ ] table_world/相机/板面未变；使用 Z_bottom=490.741 mm 与厚度 7.4 mm，不用旧 5 mm。
- [ ] 工具全包络、立柱与现场障碍已核验；全 link 最高点检查，而非只看小球或关节原点。
- [ ] 完整轨迹与真实控制器插值、限位/分支/自碰撞/各平面余量及停车过程通过；起点来自当前姿态。
- [ ] 实测速度、加速度、跟踪误差、反馈超时和独立 RTDE 故障会中止；不取消阈值或统一设 100°/s。
- [ ] 视觉更新/目标有效性/边缘范围已核验；失锁不会自动停电机这一限制有实际处置方案。
- [ ] 每段录像、事件与控制器状态可记录；失败不自动重试、不自动串联、不自动回原高度。

## 12. 可直接粘贴到新 Codex 会话的启动提示词

~~~text
请接手 /home/yc/UR3 的 H 型磁机器人轨迹实验。先完整阅读
/home/yc/UR3/docs/history/HANDOFF_H_MAGNET_PATH_20260919.md，再只读核对现场软件状态。

当前授权仅是读取交接、核对并报告。不要启动真实机械臂运动，不要播放/恢复
external_control.urp，不要启停电机，不要重启或终止 ROS/Codex/相机进程，
不要改变配置，也不要重放历史轨迹。后续开发和具体运动分别向我确认。

交接时 UR3 静止、安全 NORMAL，external_control.urp 为 PAUSED，
RTDE 有效速度缩放为 0、目标滑条比例 100%；ROS robot_program_running
曾显示 true，须与直接控制器状态核对。UR 驱动和相机/H 跟踪仍在运行；
有两个 Codex 进程，勿擅自终止任何一个，也不要形成并发控制。

工具轴 tool0 −Y 已转到世界 +Y，球心约 [216.89,91.36,431.98] mm；
H 平面投影约 [202.62,97.65] mm。板底世界 Z=490.741 mm，
板厚 7.4 mm。所有 link/完整工具在全路径及停车过程中都必须保持板底以下，
还要检查侧板 X=−140/+590 mm、桌面、支架和自碰撞。
工具包络仍是未完全实测的草案，禁止把 execution_allowed 或 verified 标记改真来放行。

当前问题：新轴向下 H 先向 −X 动一次，随后重复试验磁球确实持续转动，
H 无有效前进；618 帧保持跟踪。原因未确定，不能认定磁力不足。
上一助手建议电机停时保持高度和姿态，水平移向 H 下方
（旧估计 ΔX≈−14.3、ΔY≈+6.3 mm），但我尚未批准，尚无已审查的
XY 执行器或路径。不要为验证电机旋转再让我开机，不要盲目加速或抬高磁铁。

先报告：当前节点/进程、控制器状态、相机跟踪和与交接快照的差异，
再提出一个最小、单变量的下一步及需要我确认的内容。不能确认就标“待确认”。
网页按钮/停止录像/失锁不会停止实体电机；蓝色方形只是参考图，不是闭环控制。
~~~

交接到此为止；本文不触发任何下一步实验。
