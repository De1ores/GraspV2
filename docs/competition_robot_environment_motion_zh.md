# GraspV2 比赛真机环境与 Motion 接口

本文记录 GraspV2 在 X2 Ultra 比赛真机上的已验证执行环境、环境加载顺序和 Motion 控制契约。
它是现场联调基线，不替代每次开机的只读检查。topic、服务或消息 schema 与本文不一致时，
停止执行并确认机器人固件配套的 AimDK overlay，不能靠修改 Python 导入路径或放宽安全门限继续。

## 1. 比赛机环境快照

2026-08-09 在比赛机 `agi@192.168.110.233` 上读取到的环境如下：

| 项目 | 实测值/项目约束 |
| --- | --- |
| 机器人 | X2 Ultra，双臂各 7 轴 |
| 抓取侧 | 右臂；仅右侧安装 OmniPicker |
| 登录用户 | `agi` |
| 当时地址 | `192.168.110.233`；地址可能随现场网络改变，不属于接口契约 |
| OS | `lx2501_3_t2d5-soc1-v0.5.15` |
| Agi | `test-lx2501_3_t2d5-soc1-v1.0.0.40-match-rc2` |
| 系统架构 | Jetson/Orin `aarch64` |
| ROS | ROS 2 Humble，`/opt/ros/humble/setup.bash` |
| 系统 Python | `/usr/bin/python3` 3.10 |
| AimDK overlay | 比赛包实测使用 `/home/agi/aimdk/install/setup.bash` |
| 项目目录 | `/home/agi/graspv2-x2-ultra-offline` |
| DDS | `rmw_fastrtps_cpp`、`ROS_LOCALHOST_ONLY=0` |

版本值是一次实测快照。每次升级系统或切换比赛机后都重新执行本页第 5 节检查；不要仅凭目录名
判断 AimDK 兼容。GraspV2 会验证实际消息字段和本机 native type support，避免在 `aarch64`
上误用 `install_host` 的 x86_64 类型库。

## 2. 环境加载与一键入口

比赛机项目目录和环境入口：

```bash
cd /home/agi/graspv2-x2-ultra-offline
export GRASPV2_RUNTIME_PROFILE=competition
source tools/setup_x2_mc_env.sh
```

在实测比赛镜像中，即使不显式设置环境变量，程序也只在同时满足 `agi` 用户、`aarch64`
架构和 `/home/agi/aimdk/install/setup.bash` 存在时自动判定为 `competition`。比赛模式严格优先且
只接受 `/home/agi/aimdk/install/setup.bash`；它缺失或 native type support 不匹配时停止，不会
偷偷切换到 SVT 测试机 overlay。


脚本按固定顺序加载：

```text
/opt/ros/humble/setup.bash
→ 与固件匹配的 AimDK install/setup.bash
→ 当前项目 install/local_setup.bash
→ RMW/Fast DDS 环境
```

SVT/开发测试环境显式使用：

```bash
export GRASPV2_RUNTIME_PROFILE=test
source tools/setup_x2_mc_env.sh
```

测试模式才会检查仓库离线 overlay、`/home/svt/Raicom2026/...` 等测试机候选。一次进程确定
profile 后不会从比赛机跨环境回退到测试机；这是为了避免现场网络变化时误控另一台机器人。

### 2.1 Animation 文件位置

比赛机上 GraspV2 与 MC 在同一台主机，校验后的 CSV 会原子安装到
`/tmp/graspv2_mc_animation.csv`，不经过 SSH。测试模式才依次尝试
`agi@10.0.200.40`、`agi@10.0.1.40` 上传 CSV。两个测试地址和测试密码只属于测试 fallback，
不属于比赛机接口契约。


## 3. Motion 能力分层

GraspV2 不把“安装了 AimDK”视为所有 Motion 接口都可用，而是按实际 schema 区分能力：

| 能力 | 接口 | 当前用途 | 可用条件 |
| --- | --- | --- | --- |
| MC action | `GetMcAction`、`SetMcAction` | 读取稳定模式，进入/恢复上半身分控 | overlay 含对应服务 |
| MC input source | `GetCurrentInputSource`、`SetMcInputSource` | 注册、启用并确认控制来源 | overlay 含对应服务 |
| MC upper-body | `UpperBodyCommandArray` → `/mc/upper_body_command` | 50 Hz 双臂+头部轨迹，MC 保持平衡 | 仅兼容旧 overlay/固件 |
| MC animation | `SetMcPresetMotion(ani_path)` | 播放校验过的 20 列动作 CSV；分控的运动前回退 | AimDK v1.0.0 支持 |
| HAL joint | `JointCommandArray` → arm/waist/head command | 显式低层后备 | 必须另行确认原生控制权 |
| OmniPicker | `HandCommandArray` → hand command | 右夹爪开闭 | v1.0.0 支持 |

官方 AimDK v1.0.0 overlay 的预期能力为 `animation, base, hal-joint, omnipicker`，不包含旧版
`UpperBodyCommandArray`。比赛机若要使用 MC upper-body 分控，必须选择与当前固件配套且通过
`--require-capability upper-body` 的旧 overlay；不要把两版消息包叠加到同一个 shell。

比赛机默认先使用本机 MC upper-body 控制接口。schema、服务、模式、输入源、订阅者或起点等
任一运动前检查失败时，自动切换到本机 MC animation。若当前 overlay/固件确实不提供旧
`UpperBodyCommandArray`，这也会作为运动前能力失败进入 animation。输入源激活时的保持帧和
开始抓取前的空夹爪闭合不算计划轨迹开始，后续准备失败仍可安全切换。计划抓取轨迹一旦启动，
禁止从未知中间姿态重新播放完整 animation；视觉失败也禁止使用旧目标重新抓取。SVT 测试机
保留相同安全边界，但 animation 文件通过测试网络上传。

## 4. GraspV2 使用的接口

### 4.1 Motion topic

| Topic | ROS 类型 | 方向 | QoS/语义 |
| --- | --- | --- | --- |
| `/mc/upper_body_command` | `aimdk_msgs/msg/UpperBodyCommandArray` | GraspV2 → MC | `RELIABLE + VOLATILE`，50 Hz |
| `/aima/hal/joint/arm/state` | `aimdk_msgs/msg/JointStateArray` | HAL → GraspV2 | `BEST_EFFORT + VOLATILE` |
| `/aima/hal/joint/head/state` | `aimdk_msgs/msg/JointStateArray` | HAL → GraspV2 | 读取并保持当前头部 2 轴 |
| `/aima/hal/joint/waist/state` | `aimdk_msgs/msg/JointStateArray` | HAL → GraspV2 | 健康/低层端口；默认抓取不规划腰部 |
| `/aima/hal/joint/arm/command` | `aimdk_msgs/msg/JointCommandArray` | GraspV2 → HAL | 仅显式 `hal-joint` |
| `/aima/hal/joint/head/command` | `aimdk_msgs/msg/JointCommandArray` | GraspV2 → HAL | 仅显式 `hal-joint` |
| `/aima/hal/joint/waist/command` | `aimdk_msgs/msg/JointCommandArray` | GraspV2 → HAL | 仅显式 `hal-joint` |
| `/aima/hal/joint/hand/command` | `aimdk_msgs/msg/HandCommandArray` | GraspV2 → HAL | `BEST_EFFORT + TRANSIENT_LOCAL`，50 Hz |
| `/aima/hal/joint/hand/state` | `aimdk_msgs/msg/HandStateArray` | HAL → GraspV2 | 反馈可按配置设为必需 |

夹爪命令不在 20 列 animation CSV 中。比赛机本机 upper-body 主通道和 animation 执行器都
必须调用项目根目录的
`omnipicker_hand_student.py`，通过其中 `OmniPickerStudentNode.publish_command("right", p)`
发布右夹爪命令；不得另写一套比赛机专用 hand publisher。主通道通过该 Python 文件的 CLI
执行每个夹爪事件，并在 SDK 发布期间持续发送 upper-body 保持帧。比赛模式会强制要求该 SDK
可运行，且开始 animation 前的初始夹爪命令必须成功；运动过程中若某个夹爪事件失败，MC 仍
先完成 CSV 内置的安全返回，随后以失败状态报告。测试模式保留 best-effort 行为，便于未接
控制线的测试机验证手臂 animation。

MC 上半身消息固定为：

```text
source="graspv2"
head_pos=[head axis 0, head axis 1]
arm_pos=[left arm 7, right arm 7]
hand_sub_mode=0
hand_pos=[]
```

14 轴顺序为左右各
`shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll`。
当前抓取轨迹只改变右臂 7 轴，左臂使用进入分控前的实时反馈保持；头部同样保持实时反馈。
腰部不包含在 `UpperBodyCommandArray` 中，默认仍由 MC 平衡控制，因此当前 IK 抓取不会主动改变
腰部。只有显式启用低层 HAL 并提供腰部命令时，腰部才进入 GraspV2 的控制范围。

### 4.2 MC 服务

服务名采用真机实际 namespace：

| 服务 | 作用 | 关键字段 |
| --- | --- | --- |
| `/aimdk_5Fmsgs/srv/GetMcAction` | 读取当前 MC action/status | 要求执行前为 `STAND_DEFAULT/100` |
| `/aimdk_5Fmsgs/srv/SetMcAction` | 切换/恢复 action | `source="rc"`，进入 `UPPERBODY_REMOTE_SPLIT`，退出恢复 `STAND_DEFAULT` |
| `/aimdk_5Fmsgs/srv/SetMcInputSource` | ADD/MODIFY/ENABLE 二开来源 | `McInputSource{name, priority, timeout}` |
| `/aimdk_5Fmsgs/srv/GetCurrentInputSource` | 查询 MC 实际仲裁结果 | 返回当前来源，而不是仅返回注册结果 |
| `/aimdk_5Fmsgs/srv/SetMcPresetMotion` | 播放 animation CSV | `interrupt=false`、`ani_path=<机器人本地绝对路径>` |

### 4.3 输入源与控制权

比赛机一次故障中，`GetCurrentInputSource` 实测为 `app_proxy/60/1000 ms`；当时发布消息使用的
来源未被 MC 选中，右肩 roll 目标继续增大而机械臂保持原位，最终触发
`arm tracking error 0.355 rad exceeds 0.350`。因此不能通过增大跟踪门限处理该错误。

当前配置注册独立来源：

```text
name=graspv2
priority=65
timeout=1000 ms
```

它高于已观测的 `app_proxy/60`，但低于官方 `vr/70` 和 `rc/80`，保留遥控器/VR 的高优先级
接管能力。执行流程为：

```text
确认 STAND_DEFAULT/100 和关节健康/静止
→ ADD（已存在则 MODIFY）并 ENABLE graspv2
→ 进入 UPPERBODY_REMOTE_SPLIT/100
→ 只发布当前实时姿态的保持帧
→ GetCurrentInputSource 必须返回 graspv2
→ 才开始推进轨迹时间
→ 正常或异常退出均恢复 STAND_DEFAULT/100
```

停止发布超过 1000 ms 后来源超时，MC 可重新选择其他有效来源。若更高优先级来源正在活动，
激活检查会在运动前失败。未知、未启用或未被选中的 `source` 命令可能被 MC 丢弃，即使 topic
已有订阅者也不代表取得控制权。

## 5. 比赛前只读验收

环境和 schema 检查，不连接控制端：

```bash
cd /home/agi/graspv2-x2-ultra-offline
source tools/setup_x2_mc_env.sh

printf 'arch='; uname -m
/usr/bin/python3 --version
printf 'aimdk='; printf '%s\n' "$GRASPV2_AIMDK_SETUP"
printf 'profile='; printf '%s\n' "$GRASPV2_RUNTIME_PROFILE"
printf 'rmw='; printf '%s\n' "$RMW_IMPLEMENTATION"

PYTHONNOUSERSITE=1 /usr/bin/python3 tools/check_aimdk_overlay.py
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/check_aimdk_overlay.py \
  --require-capability upper-body
```

第二条 capability 检查只应在使用 MC 分控时通过。官方 v1.0.0 没有 upper-body schema 时，
它失败是明确的版本能力结果，应使用 animation 路径或切换正确的固件配套 overlay。

只读真机检查 MC、14 轴手臂、头部、当前输入源和右夹爪00接口：

```bash
ros2 run graspv2 x2_aimdk_hardware preflight \
  --component all --transport upper-body
```

若当前 overlay 没有 upper-body，只检查 HAL/OmniPicker：

```bash
ros2 run graspv2 x2_aimdk_hardware preflight \
  --component all --transport hal-joint
```

`preflight` 不切换 MC mode、不注册输入源、不发布关节或夹爪命令。重点确认输出包含 14 个手臂
关节、头部 2 轴、`STAND_DEFAULT/100` 和现场当前输入源；输入源在只读检查时仍是
`app_proxy` 并不等于失败，真正执行会在运动前注册并验证 `graspv2`。

也可以直接查看 ROS 图，仍然只读：

```bash
ros2 topic info -v /mc/upper_body_command
ros2 topic info -v /aima/hal/joint/arm/state
ros2 service type /aimdk_5Fmsgs/srv/GetMcAction
ros2 service type /aimdk_5Fmsgs/srv/GetCurrentInputSource
```

## 6. 安全门限和失败判定

当前默认值集中在 `config/x2_aimdk_hardware.json`：

| 检查 | 当前值 |
| --- | --- |
| 发布频率 | 50 Hz |
| 反馈等待 | 2.0 s |
| 最大反馈年龄 | 0.25 s |
| 起始最大速度 | 0.10 rad/s |
| 轨迹起点最大误差 | 0.20 rad |
| 运动中最大跟踪误差 | 当前仓库 0.25 rad；不得为绕过控制权错误而增大 |
| 最大温度 | 80 °C |
| 服务发现 | 15 s |
| 单次服务超时/重试 | 0.50 s / 8 次 |
| MC mode 确认 | 5.0 s |

执行过程中检查每个关节的实际值、目标值、最大误差关节和轨迹时刻。超限立即停止推进并恢复
稳定模式。若错误接近“静止初始位到目标”的差值，优先检查输入源仲裁、MC mode 和消息
source，不要先调整 stiffness 或误差门限。

直接 `hal-joint` 不经过上述 MC upper-body 仲裁，也不保证 MC 替 GraspV2 维持平衡；它只用于
现场已经独立解决原生控制权、机器人有可靠支撑且明确选择低层控制的情况，完整流水线不会
自动降级到它。

## 7. 配置和实现位置

- `config/x2_aimdk_hardware.json`：topic、服务、输入源、频率、QoS 对应参数和门限；
- `tools/setup_x2_mc_env.sh`：ROS → AimDK → GraspV2 的统一 source 顺序；
- `tools/detect_runtime_profile.sh`：严格区分 competition 本机和 test fallback；
- `tools/select_aimdk_setup.sh`：overlay 候选选择和 schema/native 类型探测；
- `tools/check_aimdk_overlay.py`：各 Motion capability 的严格字段校验；
- `graspv2/aimdk_hardware.py`：MC split、HAL、输入源仲裁、跟踪保护和 OmniPicker 适配；
- `graspv2/mc_custom_grasp.py`：`SetMcPresetMotion(ani_path)` animation 前检与播放；
- `docs/x2_hardware_interfaces_zh.md`：RGB-D、OmniPicker 和完整抓取状态机的通用接口契约。
