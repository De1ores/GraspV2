# X2 真机接口接入说明

本文说明 graspV2 为上肢、OmniPicker 和 RGB-D 深度数据预留的硬件边界。规划与
MuJoCo 仍是纯离线模块；只有显式运行真机桥接命令时才会加载 ROS 2/AimDK。

## 接口对应关系

| 功能 | AimDK 接口 | graspV2 入口 |
| --- | --- | --- |
| MC 上半身分控 | `/mc/upper_body_command` (`UpperBodyCommandArray`) | `trajectory --transport upper-body` |
| MC 模式切换 | `GetMcAction`、`SetMcAction` | 自动检查 `STAND_DEFAULT`，进入/退出 `UPPERBODY_REMOTE_SPLIT` |
| 双臂命令/反馈 | `/aima/hal/joint/arm/command`、`state` | `x2_aimdk_hardware` |
| 腰部命令/反馈 | `/aima/hal/joint/waist/command`、`state` | `command_joint_group("waist", ...)` |
| 头部命令/反馈 | `/aima/hal/joint/head/command`、`state` | `command_joint_group("head", ...)` |
| OmniPicker 命令/反馈 | `/aima/hal/joint/hand/command`、`state` | `x2_aimdk_hardware omnipicker` |
| X2 RGB 图像 | `/aima/hal/sensor/rgbd_head_front/rgb_image` | `run_vision.sh --capture-backend x2-aimdk` |
| X2 深度图像 | `/aima/hal/sensor/rgbd_head_front/depth_image` | 同上 |
| RGB/Depth 内参 | 对应 `rgb_camera_info`、`depth_camera_info` | 同时订阅并写入 `camera.json` |

topic、控制增益、反馈超时和夹爪参数集中在
`config/x2_aimdk_hardware.json`。如真机固件 namespace 不同，只修改这份配置，不要改规划
代码。

比赛参考 `omnipicker_hand_student.py` 的夹爪专用契约已单独落实：逻辑关节名为
`left_claw_joint` / `right_claw_joint`，command QoS 为 `BEST_EFFORT + TRANSIENT_LOCAL`
（depth 10），以 50 Hz 连续发布 2 秒；非目标侧使用 `HandType.NONE` 且命令列表为空。
比赛配置默认 `require_feedback=false`，与参考程序的 command-only 契约一致；如果 state topic
存在仍会检查新鲜反馈和 fault。现场确认反馈稳定后可改为 `true`，此时没有反馈或最终位置
超差都会阻止成功返回。

AimDK v1.0 已取消深度点云，graspV2 因此只依赖标准 `sensor_msgs/Image` 和两套
`CameraInfo`。X2 文档将 RGB 与深度帧都标记为 `rgbd_head_front`；内参不同时，采集器
会把深度逐点重投影到 RGB 像素网格。若真机报告两个不同 `frame_id`，程序会拒绝拼接，
必须先提供真实的 depth-to-RGB 外参。

参考官方文档：

- [关节控制](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/joint_control.html)
- [末端执行器控制](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/endeffector.html)
- [传感器与 RGB-D topic](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/hal/sensor.html)

## 三种深度采集方式

### X2 真机 AimDK topic

只采一帧，不运行识别：

```bash
./run_vision.sh --capture-backend x2-aimdk --capture-only
```

采集后直接识别：

```bash
./run_vision.sh --capture-backend x2-aimdk \
  --classes bottle --target-class bottle --device 0
```

可通过 `--color-topic`、`--depth-topic`、`--rgb-camera-info-topic` 和
`--depth-camera-info-topic` 临时覆盖配置。

官方文档要求原始 RGB-D 大带宽 topic 只在相机所在开发计算单元 PC2 使用，不要跨计算
单元订阅。

### 当前测试机手动 Orbbec SDK

保留现有 C++ SDK 采集器：

```bash
./tools/build_orbbec_capture.sh
./run_vision.sh --capture-backend orbbec-sdk --capture-only
```

也可以完全手动调用 SDK，再让 graspV2 只消费结果：

```bash
./build/orbbec_capture --output ./output --warmup 20
./run_vision.sh --capture-backend existing \
  --classes bottle --target-class bottle --device 0
```

三个后端都使用相同文件契约：`color.png`、对齐到 RGB 的 `depth.png`、`camera.json`，
因此后面的 YOLOE、桌面拟合和抓取规划不需要区分相机来源。

注意：当前 `config/mujoco_camera_calibration.json` 的说明与变换来自测试机 Orbbec。
X2 头部 RGB-D 必须单独完成 camera-to-MuJoCo/robot 外参标定，并通过
`--camera-calibration PATH` 或 YOLOE 的 `--calibration PATH` 指定；不能复用测试机外参。

完整流水线也支持选择后端：

```bash
# X2 真机 topic（默认）
./run_full_grasp_pipeline.sh --target-class bottle --robot ultra

# 当前测试机直接调用 Orbbec SDK
./run_full_grasp_pipeline.sh --target-class bottle --robot ultra \
  --capture-backend orbbec-sdk \
  --verification-capture-backend orbbec-sdk

# 已经手动采好 RGB-D 文件，但尚未识别
./run_full_grasp_pipeline.sh --target-class bottle --robot ultra \
  --capture-backend existing
```

`existing` 只适用于初始离线规划。真机闭合后和抬升后必须重新采集，默认回到 X2 topic；
测试机可显式选择 `--verification-capture-backend orbbec-sdk`。任何检查点都不会复用旧的
`result.json`。

## 上肢与 OmniPicker 真机桥接

先构建 v0.9.0.7 兼容消息 overlay；其中已经包含 MC、`UpperBodyCommandArray`、
`SetMcAction`、`JointCommandArray`、`HandCommandArray` 和对应状态消息：

```bash
./tools/build_aimdk0907_overlay.sh
source tools/setup_x2_mc_env.sh
```

只读检查双臂和双 OmniPicker：

```bash
ros2 run graspv2 x2_aimdk_hardware preflight \
  --robot ultra --component all --transport upper-body
```

轨迹和夹爪命令默认只做本地校验，不发布：

```bash
ros2 run graspv2 x2_aimdk_hardware trajectory \
  --robot ultra --trajectory output/planned_trajectory_ultra.json \
  --transport upper-body

ros2 run graspv2 x2_aimdk_hardware omnipicker \
  --side right --action open
```

### 上半身 MC 分控（默认）

`trajectory` 默认使用 `--transport upper-body`。执行时会按下面顺序工作：

1. 从 HAL 读取手臂和头部反馈，校验反馈新鲜度、静止速度、故障、温度和轨迹起点误差；
2. 通过 `GetMcAction` 要求当前为 `STAND_DEFAULT/100`；
3. 通过 `SetMcAction` 进入 `UPPERBODY_REMOTE_SPLIT`，确认
   `/mc/upper_body_command` 已有 MC 订阅者；
4. 以 50 Hz 发布头 2 轴、双臂固定 14 槽的目标，并监测实时跟踪误差；
5. 正常完成或异常退出时，都尝试恢复 `STAND_DEFAULT`。

Youth 的 10 个物理臂轴会映射到固定 14 槽，缺少的两侧 wrist pitch/roll 槽保持 0；Ultra
直接使用 14 轴。上半身消息构造器也支持 `hand_sub_mode=1` 的 `[左, 右]` OmniPicker
开合量，当前独立夹爪 CLI 仍走更容易单独验证反馈的 HAL hand 接口。

真机发布仍必须显式增加两个确认参数：

```bash
ros2 run graspv2 x2_aimdk_hardware trajectory \
  --robot ultra --trajectory output/planned_trajectory_ultra.json \
  --transport upper-body \
  --execute --confirm-control-authority
```

旧测试机固件如果没有 `SetMcAction` 或 `/mc/upper_body_command` 订阅者，preflight/执行会明确
失败，不会静默退到 HAL。当前 v0.9 的 MC animation 播放链路保持不变，可继续使用
`run.sh --mode animation`。

### HAL 关节后备与 OmniPicker

低层双臂后备可显式选择 `--transport hal-joint`；OmniPicker 独立命令走 hand HAL：

```bash
ros2 run graspv2 x2_aimdk_hardware preflight \
  --robot ultra --component upper-body --transport hal-joint

ros2 run graspv2 x2_aimdk_hardware trajectory \
  --robot ultra --trajectory output/planned_trajectory_ultra.json \
  --transport hal-joint

ros2 run graspv2 x2_aimdk_hardware omnipicker \
  --side right --action close
```

实际发布必须显式增加 `--execute --confirm-control-authority`。对于比赛 OmniPicker，这个
确认表示遵守赛项已经配置好的机器人模式和夹爪链路，不自行停止/重启 MC，不修改手型、
设备 ID、CAN/EtherCAT 或固件，并已确认现场安全。对于 `hal-joint` 低层上肢后备，则还
表示现场已经完成以下工作：

1. 根据实际固件和系统集成方案处理原生 MC 的控制权；
2. 吊架或固定底座、实体急停和人员隔离均就绪；
3. MC 停止后仍有人/上层控制器负责所有维持机器人安全所需的关节；
4. 轨迹起点与实时反馈误差在配置阈值内。

比赛参考明确规定 OmniPicker 的机器人模式和设备参数已由工作人员配置，参赛程序不得
自行停止或重启 MC。`hal-joint` 则属于另一条低层上肢后备：全身站立机器人若没有其他
控制器接管腿和躯干会失去平衡。graspV2 不会自动停止 MC，也不会把命令失败后的自动重试
当成安全恢复。

### 视觉闭环抓取路径

`run_full_grasp_pipeline.sh` 现在生成两条独立验证轨迹：接近轨迹，以及从抓取点沿桌面法向
抬升的轨迹。抬升默认 `0.10 m`，运动时间严格为 `2.0 s`。真机入口在一次持续的
`UPPERBODY_REMOTE_SPLIT` 会话内执行：

1. 打开目标侧 OmniPicker；
2. 走碰撞验证的预抓取/笛卡尔接近轨迹；
3. 闭合夹爪，同时继续以 50 Hz 发布上肢保持帧；
4. 重新采集 RGB-D，要求同类目标仍位于计划抓取区；
5. 执行两秒抬升轨迹；
6. 再次采集 RGB-D，要求目标沿抬升方向移动至少默认高度的 60%，同时满足目标位置误差和
   横向漂移门限；仍停在桌面上的目标会判为抓取失败；
7. 成功或视觉判定失败时，均沿已验证轨迹下降、松爪并逆向撤回，再恢复稳定站立模式。

视觉采集/推理在线程中运行，控制线程在两个检查点持续发布末端保持帧，而不是在推理期间
停止控制。每次执行写入 `output/grasp_status.json`，包含两次视觉检查的坐标、误差和最终
`success`。视觉超时、目标消失、类别变化、抬升位移不足或漂移过大都会返回失败。

离线检查与真机执行示例：

```bash
# 规划、碰撞检查和状态机契约检查；不采集检查点、不连接机器人
./run_full_grasp_pipeline.sh --target-class bottle --robot ultra --use-existing-vision

# 真机；仍需现场标定确认并输入 RUN
./run_full_grasp_pipeline.sh --target-class bottle --robot ultra \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --execute --confirm-calibrated
```

当前视觉仍提供 3D 表面点而非完整 6D 抓取姿态，因此夹爪方向继续由现有 IK/工具标定定义；
透明、反光或被夹爪完全遮挡的物体可能无法通过视觉门限，程序会安全判失败，不会把“未看见”
当作已经抓牢。旧的 hand-free MC animation 入口仍保留用于单独轨迹回放，但不再是完整抓取
流水线的执行后端。

OmniPicker 发布示例仅用于完成上述现场安全准备之后：

```bash
ros2 run graspv2 x2_aimdk_hardware omnipicker \
  --side right --action close \
  --execute --confirm-control-authority
```

Python 集成层位于 `graspv2.hardware_contract` 与 `graspv2.aimdk_hardware`：

- `UpperBodyControl.command_joint_group()` 暴露 `arm`、`waist`、`head` 三组端口；
- `UpperBodyControl.command_upper_body()` 暴露 MC 分控的头/双臂/手统一帧；
- `OmniPickerControl.command_omnipicker()` 使用归一化 `0.0～1.0` 行程；
- `create_aimdk_hardware_node()` 是唯一绑定 `rclpy`/`aimdk_msgs` 的适配器工厂。

`future_upper` profile 仍保持禁用。它代表另一套尚未提供 URDF、基座变换和关节契约的比赛
纯上肢机型，不能仅因 HAL topic 已预留就复用 Ultra 模型。拿到比赛机资料后，只需补充
`RobotProfile` 和对应 IK/URDF；上述硬件端口无需重写。
