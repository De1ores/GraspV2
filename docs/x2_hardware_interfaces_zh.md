# X2 真机接口契约

本文只记录 GraspV2 与 X2/AimDK 的技术边界。安装、常用命令和故障处理统一见
[`package_install_run_debug_zh.md`](package_install_run_debug_zh.md)。规划与 MuJoCo 是离线模块；
只有显式真机命令才加载控制接口。

比赛机的实测 OS/Agi/AimDK 环境、统一 source 顺序、Motion 消息布局和输入源仲裁见
[`competition_robot_environment_motion_zh.md`](competition_robot_environment_motion_zh.md)。

## 1. 接口对应关系

| 功能 | AimDK/ROS 接口 | GraspV2 入口 |
| --- | --- | --- |
| RGB 图像 | `/aima/hal/sensor/rgbd_head_front/rgb_image` | `run_vision.sh --capture-backend x2-aimdk` |
| 深度图像 | `/aima/hal/sensor/rgbd_head_front/depth_image` | 同上 |
| RGB/Depth 内参 | 对应 `rgb_camera_info`、`depth_camera_info` | 同步采集并写入 `camera.json` |
| 旧 MC 上半身分控 | `/mc/upper_body_command`、`UpperBodyCommandArray` | `trajectory --transport upper-body` |
| MC 模式/输入源 | `Get/SetMcAction`、`Get/SetMcInputSource` | 分控进入、激活和恢复 |
| MC animation | `SetMcPresetMotion(ani_path)` | animation 播放及安全回退 |
| 双臂 HAL | `/aima/hal/joint/arm/command`、`state` | 显式 `hal-joint` |
| 腰/头 HAL | 对应 `/waist/`、`/head/` command/state | 保持与健康检查 |
| OmniPicker | `/aima/hal/joint/hand/command`、`state` | 右侧夹爪命令和反馈 |

所有 topic、服务、频率和门限集中在
[`config/x2_aimdk_hardware.json`](../config/x2_aimdk_hardware.json)。现场 namespace 不同时只修改
该配置，不要把 topic 写死到规划器。

## 2. RGB-D 契约

`x2-aimdk` 后端使用系统 Python 和 `rclpy` 订阅官方 ROS topic，不直接启动 X2 相机 C/C++
SDK。采集器要求：

- RGB、Depth 和两套 CameraInfo 都在超时前到达；
- RGB/Depth 时间差不超过配置门限；
- Image 与对应 CameraInfo 的尺寸和 `frame_id` 一致；
- RGB 与 Depth `frame_id` 相同，否则缺少真实外参，拒绝拼接；
- 深度转换为 `uint16` 后配准到 RGB 像素网格。

完整流水线默认使用 `auto`：先走 `x2-aimdk`，四个 topic 未能在超时前同步时，采集器以
专用退出码 20 通知入口切换到同一相机的本地 Orbbec SDK。数据契约失败不触发回退；显式
`x2-aimdk` 也不回退。任何真机执行都禁止 `existing`，因此不会使用历史图片。

图像方向默认 `auto`：每次实时采集先保持 `0°`，只有桌面拟合失败状态 42 才同步切换 RGB、
Depth、内参和畸变参数到 `180°`。自动模式分别把两者当作正装/倒装安装假设，使用通过3D
桌面门的假设和标称外参；不会套用另一台机器的方向字段做二次补偿。目标未识别及其他视觉错误不触发旋转。显式
`calibrated` 才直接读取正装 `upright/0` 或倒装 `inverted/180` 配置。完整流水线的初次识别和
两次视觉复检使用同一策略和标定文件；方向切换不更换采集设备，也不能替代外参标定。

输出 `camera.json` 记录源 topic、编码、深度比例、配准方式、时间差和最终图像方向。

## 3. OmniPicker 契约

比赛机只在右侧安装 OmniPicker，逻辑关节为 `right_claw_joint`：

- 左侧 `HandType.NONE`，命令数组为空；
- 右侧 `HandType` 为 OmniPicker，归一化位置 `0=闭合、1=全开`；
- command QoS 为 `BEST_EFFORT + TRANSIENT_LOCAL`，depth 10；
- 默认 50 Hz 连续发布 2 秒；
- `require_feedback=false` 时允许比赛参考程序的 command-only 模式，但若收到新鲜反馈仍检查
  fault 和掉落状态；
- 设置 `require_feedback=true` 后，无反馈或最终位置超差都会失败。

设备 ID、CAN/EtherCAT、手型和固件不由 GraspV2 修改。

animation 的夹爪事件统一调用项目根目录 `omnipicker_hand_student.py` 中的
`OmniPickerStudentNode.publish_command("right", position)`，不在 20 列 CSV 中伪造 hand
列。比赛机 profile 要求 SDK 可加载且初始命令成功；测试 profile 保留 command-only/
未接控制线时的 best-effort 验证行为。

## 4. AimDK 版本能力

官方 AimDK v1.0.0 的项目实测能力为：

```text
animation, base, hal-joint, omnipicker
```

它没有旧 `UpperBodyCommandArray`。旧固件/overlay 只有通过
`tools/check_aimdk_overlay.py --require-capability upper-body` 后才能使用 MC 分控。SDK 与机器人
固件必须配套，不能只为通过导入而混用消息包。

参考官方文档：

- [SDK 获取与固件匹配](https://x2-aimdk.agibot.com/zh-cn/latest/get_sdk/index.html)
- [更新日志](https://x2-aimdk.agibot.com/zh-cn/latest/changelog.html)
- [关节控制](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/joint_control.html)
- [末端执行器控制](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/endeffector.html)
- [传感器接口](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/hal/sensor.html)

## 5. MC upper-body 状态机

单条轨迹使用旧 MC 分控时按以下顺序执行：

1. 校验双臂、头部反馈新鲜度、故障、温度、静止速度和轨迹起点；
2. 要求当前 MC 为 `STAND_DEFAULT/100`；
3. 注册并启用 `graspv2` 输入源；
4. 进入 `UPPERBODY_REMOTE_SPLIT` 并确认 command topic 有订阅者；
5. 持续发布当前保持帧，等待 MC 选中 `graspv2`；
6. 以 50 Hz 发布头部 2 轴和双臂固定 14 槽目标，同时检查跟踪误差；
7. 正常或异常退出均尝试恢复 `STAND_DEFAULT`。

输入源默认优先级为 65、超时为 1000 ms。停止发布后 MC 应自动交还控制；更高优先级安全
来源仍可接管。

### Competition 本机 upper-body 与 animation 回退

程序在连接机器人前把同一条官方 IK + MuJoCo 验证轨迹转换为 MC animation。CSV 保持
20 列手臂格式，播放器调用仓库内 `omnipicker_hand_student.py` SDK。完整抓取 animation
包含移动前全开 3 秒、安全预备、下降、按半径闭合、抬升/悬停、受控放下、松开、张开撤离、空夹爪
闭合和回默认位；不包含可阻塞播放的中间视觉门。程序显式记录计划轨迹是否开始；输入源激活
HOLD 帧及开始运动前持续 3 秒的全开夹爪命令仍允许关闭分控节点并启动 animation。

`GRASPV2_RUNTIME_PROFILE=competition` 时，完整抓取先尝试比赛机本机 upper-body；任一运动前
能力、服务、模式、输入源、订阅者、起点或初始夹爪检查失败时进入本机 animation。`test`
profile 使用测试机 upper-body 和 SSH animation。两者都只允许在计划轨迹开始前切换；运动
开始后或视觉失败时禁止从头重放。比赛机主通道的夹爪事件也直接调用仓库
`omnipicker_hand_student.py`，SDK 运行期间持续发送 upper-body 保持帧。

- 计划抓取轨迹已经启动；
- animation 前检不满足稳定模式、健康、静止或默认起点；
- 调用者选择低层 HAL；
- 运动开始后的视觉、跟踪或夹爪错误。

现场还存在一种 AimDK 混合 overlay：机器人按旧布局发布相邻的 `coil_temp/motor_temp`，本机
v1.0 类型却把两个字节显示为一个 `error_code`，常见十进制值约为 12000。程序只在整帧至少
四个关节都能解码为 `5–125°C` 的成对温度时启用兼容解释，并照常执行 80°C 超温门；单个
非零错误码不会被兼容逻辑吞掉。

## 6. HAL 边界

`hal-joint` 是显式低层后备，绝不自动启用。使用它意味着现场已经处理原生 MC 控制权，并
有人或上层控制器负责所有维持机器人安全所需的关节；吊架/固定底座、实体急停和人员隔离
必须就绪。

## 7. 完整视觉抓取状态机

完整 `grasp` 使用 MC upper-body 和独立 hand HAL：

```text
闭合空夹爪在机器人一侧向外 6 cm 并抬高到安全预备点
→ 保持高位移动到物体上方预抓点
→ 在物体估计顶部上方 3 cm 处完全打开
→ 垂直下降到物体中值高度上方 1 cm
→ 按视觉半径夹紧和稳定等待
→ 闭合后视觉门
→ 垂直抬升
→ 防掉落视觉门
→ 在抬升点保持约 2.5 秒
→ 夹紧状态沿抬升轨迹受控下降并放回原位
→ 完全打开释放并等待物体稳定
→ 全开垂直退回预抓点
→ 闭合空夹爪并返回 SD 初始位
```

视觉检查期间持续发布保持帧。任何运动开始后的识别、位置、跟踪或控制错误都保持故障关闭。
只允许在计划轨迹开始前降级为单条 animation；它会做尽力而为的张开/闭合和原路
返回，但没有中间视觉门，不能宣称等价于完整 `grasp` 状态机。

## 8. 代码边界

- `graspv2.hardware_contract`：ROS 无关的 topic、服务、消息布局和控制参数；
- `graspv2.aimdk_hardware`：唯一绑定 `rclpy/aimdk_msgs` 的真机控制适配器；
- `vision/ros_rgbd_capture.py`：X2 ROS RGB-D 订阅、同步、配准和落盘；
- `tools/orbbec_capture.cpp`：测试机本地 Orbbec SDK 采集器；
- `graspv2.mc_custom_grasp`：MC animation 前检和 `SetMcPresetMotion` 客户端。
