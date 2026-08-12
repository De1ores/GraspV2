# graspV2

面向 X2 Ultra、右臂和右侧 OmniPicker 的视觉抓取工程。流程使用官方
`x2_ik_sdk` 求解 IK，通过 MuJoCo 检查关节限位、桌面间隙和整条轨迹碰撞，并在真机执行时
加入 RGB-D 复检和故障关闭。

完整安装、打包、运行和故障排查统一见
[`docs/package_install_run_debug_zh.md`](docs/package_install_run_debug_zh.md)。本页只保留常用入口。

## 快速开始

开发机无头仿真，不连接机器人：

```bash
./run.sh --no-vision --headless
```

使用当前视觉结果规划并打开 Viewer：

```bash
./run.sh --vision-result output/result.json
```

完整流水线只识别和规划：

```bash
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --plan-only
```

真机执行必须确认相机标定并输入 `RUN`：

```bash
source tools/setup_x2_mc_env.sh
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --execute --confirm-calibrated
```

三个入口均提供完整参数说明：

```bash
./run.sh --help
./run_vision.sh --help
./run_full_grasp_pipeline.sh --help
```

## 运行边界

| 入口 | 作用 | 是否接触真机 |
| --- | --- | --- |
| `run.sh --mode sim` | 官方 IK、MuJoCo 规划和回放 | 否 |
| `run.sh --mode animation` | 生成或校验 MC animation CSV | 仅加 `--execute` 后 |
| `run_vision.sh` | RGB-D 采集和 YOLOE 识别 | 只读相机 |
| `run_full_grasp_pipeline.sh --plan-only` | 采集、识别和三段规划 | 不发布控制命令 |
| `run_full_grasp_pipeline.sh --execute` | 视觉闭环抓取 | 是，需双重确认 |

真机前必须依次通过：环境导入、RGB-D、无头规划、AimDK 只读前检和 `--plan-only`。不要通过
放宽碰撞、起点或跟踪门限来绕过失败。

## 安装

开发机第一次运行 `run.sh` 会创建 `.venv` 并安装规划依赖。

Jetson/Orin 联网部署可下载并构建官方 X2 AimDK v1.0.0 消息包：

```bash
./tools/fetch_official_aimdk_x2.sh
./tools/build_graspv2_with_installed_aimdk.sh
source tools/setup_x2_mc_env.sh
```

官方 SDK 与机器人固件必须匹配。v1.0.0 提供 animation、HAL joint 和 OmniPicker 接口，但
没有旧版 `UpperBodyCommandArray`。下载脚本不会覆盖系统 AimDK；版本说明见
[官方更新日志](https://x2-aimdk.agibot.com/zh-cn/latest/changelog.html)。

完全离线包首次安装：

```bash
./install_offline.sh
./offline_run.sh --target-class cup --plan-only
```

上传到 `svt@192.168.1.240`、安装和真机执行的最短命令见
[`DEPLOY_SVT.md`](DEPLOY_SVT.md)。

## RGB-D 与识别

完整流水线默认使用 `auto`：先订阅 X2 官方 RGB、Depth 和两套 CameraInfo ROS topic；四个
topic 在超时前无法同步时，自动改用同一相机的本地 Orbbec SDK。数据格式、配准或标定校验
失败时不会回退，任何情况都不会拿历史图片代替实时采集。

```bash
# 默认策略：X2 topic 超时后回退本地 SDK
./run_vision.sh --capture-backend auto --capture-only

# X2 官方 topic
./run_vision.sh --capture-backend x2-aimdk --capture-only

# 测试机显式调用本地 Orbbec SDK 二进制
./run_vision.sh --capture-backend orbbec-sdk --capture-only

# 仅离线复用已存在的 color.png、depth.png、camera.json
./run_vision.sh --capture-backend existing \
  --classes cup --target-class cup --device 0
```

注意：单独运行 `run_vision.sh` 时默认后端是 `existing`；完整流水线默认是 `auto`。显式指定
`x2-aimdk` 可关闭 SDK 回退，便于排查 topic。

图像方向默认 `auto`：SDK/ROS 原始帧先以 `0°` 运行，只有桌面拟合明确失败（专用退出码 42）
才同步旋转 RGB、Depth、内参和畸变参数到 `180°` 重试。目标类别缺失、模型错误或其他深度门
失败都不会触发旋转。`auto` 将 `0°/180°` 分别作为正装/倒装假设，并让通过桌面门的假设使用
标称外参；显式 `calibrated` 才直接使用标定文件中的 `capture_image_rotation_deg`。
正装和倒装机器仍须分别保存实测外参；方向回退不能替代标定。

## 仿真和 Animation

常用仿真命令：

```bash
./run.sh --demo-scene
./run.sh --target 0.38 -0.30 0.92
./run.sh --vision-result output/result.json --headless
```

规划会生成接近、抬升和受控放回/返回三段轨迹。接近段不再使用随机 RRT 绕行：机械臂先
在机器人一侧向右外移 6 cm 并抬高到确定性安全预备点，再在高位移到物体顶部上方 3 cm。
夹爪完全打开后下降到物体中值高度上方 1 cm，并按视觉半径闭合；抬升、保持约 2.5 秒后
夹紧放回，再退回 SD 初始位。MuJoCo 碰撞/边检查仍作为安全门。右侧 OmniPicker 始终使用
实测 TCP；相机全局偏移不能代替 TCP offset。详细标定方法见
[`docs/tool_offset_calibration_zh.md`](docs/tool_offset_calibration_zh.md)。

位置-only 官方 IK 若达到最大迭代但真实 FK 位置残差不超过 `0.05 m`，规划器会比较全部
deterministic seed，选择位置误差最小且通过 MuJoCo 状态碰撞和上一轨迹边检查的关节点继续
规划。超过 5 cm、候选碰撞、边无效或带姿态约束时仍失败。报告会同时写入请求点、实际最近
点、误差和 `nearest_ik_fallback_used`；终端出现 `IK nearest-point fallback: ACCEPTED` 才表示
启用了该路径。后续稠密轨迹、桌面间隙、速度、抬升和返回安全门不会跳过。

生成或播放 MC animation：

```bash
./run.sh --mode animation
./run.sh --mode animation --animation /path/to/action.csv
./run.sh --mode animation --execute
```

animation CSV 保持 20 列手臂动作并内置返回默认位路径；夹爪明确调用仓库内
`omnipicker_hand_student.py` SDK：播放前执行 `open right`，到达目标保持段时并行执行
`close right`。新生成动作会在目标位保持约 2 秒供夹爪闭合。比赛 profile 强制 SDK 加载和
初始命令成功，阶段命令失败会在 MC 安全返回后报告失败；测试 profile 只输出 warning，便于
未接控制线时验证手臂 animation。测试模式可用 `--no-gripper` 显式只播放手臂。真机播放仍
严格检查
`STAND_DEFAULT`、14 轴健康/静止、起点、文件校验和与最终返回。

## AimDK 控制与安全回退

只读前检：

```bash
source tools/setup_x2_mc_env.sh
ros2 run graspv2 x2_aimdk_hardware preflight \
  --component all --transport hal-joint
```

单条已验证轨迹默认先尝试旧 MC upper-body 分控。若在计划轨迹运动开始之前失败，
自动切换到预先生成的 MC animation：

```bash
ros2 run graspv2 x2_aimdk_hardware trajectory \
  --trajectory output/planned_trajectory.json \
  --execute --confirm-control-authority
```

使用 `--upper-body-fallback none` 可关闭回退。完整抓取会预先生成与仿真物理阶段一致的
animation：安全预备/下降、全开、按目标半径闭合、抬升、悬停、受控放下、松开、张开撤离、
空夹爪闭合和回默认位。competition 先使用比赛机本机 upper-body，任一运动前接口/模式/
控制权/起点/初始夹爪检查失败时回退到本机 animation；输入源 HOLD 帧不阻止回退，计划轨迹
启动后才禁止重放。比赛机主通道夹爪动作直接调用 `omnipicker_hand_student.py`，SDK 运行时
持续保持上肢目标。test 保留相同的安全边界。MC 单条 animation
无法在中途等待视觉结果，
因此不包含“失败即暂停”的中间视觉门，日志会明确标记。以下情况绝不自动切换：

运行环境分为 `competition` 和 `test`。比赛机必须在本机 `agi/aarch64` 环境运行，使用
`/home/agi/aimdk/install/setup.bash`，并把 CSV 原子安装到本机 `/tmp`；不会进行 SSH 上传。
其他环境进入测试 fallback，CSV 才依次上传到 `agi@10.0.200.40` 和 `agi@10.0.1.40`，默认密码
为 `1` 并通过 OpenSSH askpass 自动输入。可用 `GRASPV2_ROBOT_ADDRESS`、
`GRASPV2_ROBOT_FALLBACK_ADDRESS`、`GRASPV2_ROBOT_USER` 和
`GRASPV2_ROBOT_PASSWORD` 覆盖测试参数，也可用 `GRASPV2_RUNTIME_PROFILE` 显式固定环境。
一次执行不会从比赛机跨环境切换到测试机。

- 计划 upper-body 轨迹已经启动（输入源激活 HOLD 帧和初始空夹爪命令不算）；
- 需要低层 `hal-joint` 控制；
- 运动开始后的跟踪、视觉或夹爪状态机失败。

部分机器人仍发布旧版 `coil_temp/motor_temp` 两个字节，而本机 v1.0 消息把同一位置解释成
`uint16 error_code`。程序只有在整帧至少四个关节都能解码为合理温度时才按温度处理，并继续
执行超温保护；单个非零值仍按真实关节错误停止。

## 配置与输出

| 路径 | 内容 |
| --- | --- |
| `config/mujoco_camera_calibration.json` | 相机外参、方向约定和视觉门限 |
| `config/tool_pose_offset.json` | OmniPicker 基座到真实夹持中心 TCP |
| `config/x2_aimdk_hardware.json` | ROS topic、服务、控制和夹爪参数 |
| `output/result.json` | 桌面、目标和抓取中心 |
| `output/planned_trajectory.json` | 接近轨迹 |
| `output/planned_lift.json` | 抬升轨迹 |
| `output/planned_return.json` | 返回默认位轨迹 |
| `output/planning_report*.json` | IK、碰撞、速度和 FK 对齐报告 |
| `output/grasp_status.json` | 真机视觉抓取状态 |

视觉 schema v2 中：`surface_point_mujoco_m` 只用于深度调试，
`object_center_mujoco_m` 用于跟踪，`gripper_center_mujoco_m` 才是 IK/TCP 目标。

## 文档

- [安装、打包、运行与故障排查](docs/package_install_run_debug_zh.md)
- [比赛真机环境与 Motion 接口](docs/competition_robot_environment_motion_zh.md)
- [X2 topic、消息和控制契约](docs/x2_hardware_interfaces_zh.md)
- [TCP 与工具偏移标定](docs/tool_offset_calibration_zh.md)
- [现场最短命令](run.md)
- [官方 IK SDK 说明](x2_ik_sdk/README.md)
