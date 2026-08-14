# GraspV2 安装、运行与调试指南

这是项目的中文主使用文档，覆盖开发机、Jetson/Orin 联网部署和完全离线部署。默认工作目录
为仓库根目录：

```bash
cd /home/GraspV2
```

现场执行固定面向 X2 Ultra、右臂和右侧 OmniPicker。topic、消息和状态机细节见
[`x2_hardware_interfaces_zh.md`](x2_hardware_interfaces_zh.md)，TCP 标定见
[`tool_offset_calibration_zh.md`](tool_offset_calibration_zh.md)，比赛机环境和 Motion 联调基线见
[`competition_robot_environment_motion_zh.md`](competition_robot_environment_motion_zh.md)。

## 1. 安全边界与执行顺序

按以下顺序放行，不要从安装完成直接跳到真机抓取：

1. Python、模型和 CUDA 导入；
2. RGB-D 采集与目标识别；
3. 官方 IK 与 MuJoCo 无头规划；
4. AimDK 只读前检；
5. 完整流水线 `--plan-only`；
6. 确认急停、人员隔离、相机/TCP 标定后真机执行。

`run.sh`、视觉采集和 `--plan-only` 不发布机器人控制命令。完整抓取需要同时提供
`--execute --confirm-calibrated`，并在现场输入 `RUN`。

真机执行前必须确认：

- 机器人由可靠支撑或站立控制保持安全，急停可触达；
- 人员、线缆和无关物体离开机械臂及夹爪运动范围；
- AimDK 与机器人固件匹配，MC 模式策略已经现场确认；
- 相机外参、RGB-D 对齐、图像方向和右侧 TCP 均为当前机器实测值；
- 右侧 OmniPicker 正常，归一化开度为 `0=闭合、1=全开`；
- 无头规划、只读前检和 `--plan-only` 全部通过。

## 2. 入口、配置和输出

### 2.1 入口

| 文件 | 用途 |
| --- | --- |
| `run.sh` | 官方 IK、MuJoCo 规划、Viewer 和 animation |
| `run_vision.sh` | X2/Orbbec RGB-D 采集与 YOLOE 识别 |
| `run_full_grasp_pipeline.sh` | 采集、识别、三段规划和视觉闭环抓取 |
| `offline_run.sh` | 离线包统一运行入口 |
| `install_offline.sh` | 离线创建环境并构建 ROS 包 |
| `tools/fetch_official_aimdk_x2.sh` | 下载、校验和构建官方 AimDK v1.0.0 消息包 |

### 2.2 配置

| 文件 | 必须复核的内容 |
| --- | --- |
| `config/mujoco_camera_calibration.json` | 相机外参、图像方向和视觉门限 |
| `config/tool_pose_offset.json` | OmniPicker 基座到真实夹持中心 TCP |
| `config/x2_aimdk_hardware.json` | ROS topic、服务、控制和右夹爪参数 |
| `config/fastdds_wired.xml` | 现场有线网卡 Fast DDS 配置 |

相机全局偏移会一起移动桌面、物体中心和夹爪中心；TCP offset 只描述工具固定变换，两者不能
互相代偿。

### 2.3 主要输出

| 路径 | 内容 |
| --- | --- |
| `output/color.png`、`depth.png`、`camera.json` | 本次 RGB-D 帧和相机信息 |
| `output/annotated.png`、`result.json` | 识别可视化和目标/桌面结果 |
| `output/planned_trajectory.json` | 接近轨迹 |
| `output/planned_lift.json` | 抬升轨迹 |
| `output/planned_return.json` | 返回默认位轨迹 |
| `output/planning_report*.json` | IK、碰撞、速度、桌面间隙和 FK 对齐报告 |
| `output/mc_animation.csv` | MC animation 20 列 CSV |
| `output/grasp_status.json` | 真机闭环状态 |

视觉 schema v2 中，`surface_point_mujoco_m` 只用于深度调试，
`object_center_mujoco_m` 用于视觉跟踪，`gripper_center_mujoco_m` 是 IK/TCP 目标。

## 3. 安装

### 3.1 开发机仿真

开发机不需要 ROS/AimDK。第一次运行会创建 `.venv` 并安装 `requirements.txt`：

```bash
./run.sh --no-vision --headless
```

有图形桌面时查看内置场景：

```bash
./run.sh --demo-scene
```

WSL2 默认使用软件 OpenGL；只有 `glxinfo -B` 已确认硬件渲染正常时才设置
`GRASPV2_WSL_SOFTWARE_GL=0`。

### 3.2 Jetson/Orin 联网安装

系统前提：Ubuntu 22.04、`aarch64`、系统 Python 3.10、ROS 2 Humble、CUDA、pip、venv 和
colcon。

```bash
uname -m
/usr/bin/python3 --version
test -f /opt/ros/humble/setup.bash
```

安装规划和视觉环境：

```bash
/usr/bin/python3 -m venv .planning-venv
.planning-venv/bin/python -m pip install -r requirements.txt

GRASPV2_VENV="$PWD/.vision-venv" \
  ./tools/setup_orin_vision_env.sh
```

下载官方 X2 AimDK v1.0.0 消息包并构建项目：

```bash
./tools/fetch_official_aimdk_x2.sh
./tools/build_graspv2_with_installed_aimdk.sh
source tools/setup_x2_mc_env.sh
```

下载归档固定校验值：

```text
5bbcf724d54fb28f153db0d272f9acb7906bb1d2cac7dd7ccdc699a5c7eeab35
```

SDK 保存在被 Git 忽略的 `offline/runtime/aimdk-x2-v1.0.0/`，不会覆盖系统目录。SDK 与固件
必须配套；旧固件应显式使用它随附的 overlay：

```bash
export GRASPV2_AIMDK_SETUP=/path/to/aimdk/install/setup.bash
./tools/build_graspv2_with_installed_aimdk.sh
source tools/setup_x2_mc_env.sh
```

官方 v1.0.0 提供 animation、HAL joint 和 OmniPicker 接口，但不包含旧
`UpperBodyCommandArray`。这是能力差异，不是安装损坏。

运行环境必须分开：比赛机使用 `GRASPV2_RUNTIME_PROFILE=competition`，严格选择
`/home/agi/aimdk/install/setup.bash`；SVT/开发测试机使用 `GRASPV2_RUNTIME_PROFILE=test`，
才检查仓库离线 overlay、`/home/svt/Raicom2026/...` 等候选。仍可用
`GRASPV2_AIMDK_SETUP` 显式固定与固件匹配的 overlay。一次执行不会从比赛机跨环境回退到
测试机。

### 3.3 Jetson/Orin 完全离线安装

开发机上传到现场机器人：

```bash
cd /home/GraspV2
scp dist/graspv2-x2-ultra-offline.tar.gz{,.sha256} \
  svt@192.168.1.240:/home/svt/
```

在机器人上校验、解压并安装：

```bash
ssh svt@192.168.1.240
cd /home/svt
sha256sum --check graspv2-x2-ultra-offline.tar.gz.sha256
tar -xzf graspv2-x2-ultra-offline.tar.gz
mv graspv2-x2-ultra-offline graspV2
cd graspV2
./install_offline.sh
```

安装器验证归档清单，只从本地 wheelhouse 创建 `.planning-venv` 和 `.vision-venv`，解压
cuSPARSELt、验证 AimDK、构建 `graspv2`，并运行一次无 OpenGL MuJoCo 自检。成功输出包含：

```text
headless_model_check=PASS
Offline GraspV2 installation complete
```

环境损坏时可运行 `./install_offline.sh --force`；旧虚拟环境会移动到时间戳备份目录。
现场最短上传、部署和执行命令统一见 [`DEPLOY_SVT.md`](../DEPLOY_SVT.md)。

## 4. 打包与部署

### 4.1 联网现场机源码包

打包前确认配置和模型完整，排除虚拟环境、编译产物和旧输出：

```bash
cd /home
tar \
  --exclude='GraspV2/.git' \
  --exclude='GraspV2/.venv' \
  --exclude='GraspV2/.planning-venv' \
  --exclude='GraspV2/.vision-venv' \
  --exclude='GraspV2/.runtime' \
  --exclude='GraspV2/build' \
  --exclude='GraspV2/install' \
  --exclude='GraspV2/log' \
  --exclude='GraspV2/output' \
  --exclude='GraspV2/offline/runtime' \
  -czf /tmp/graspv2-source.tar.gz GraspV2
sha256sum /tmp/graspv2-source.tar.gz \
  > /tmp/graspv2-source.tar.gz.sha256
```

相机标定应按机器人单独传输，不要固化到通用源码包。

### 4.2 完整离线包

制包机先准备固定模型和 AArch64 依赖：

```bash
./tools/fetch_offline_dependencies.sh
./tools/check_offline_assets.py
GRASPV2_BUNDLE_NAME=graspv2-x2-ultra-offline \
  ./tools/build_offline_bundle.sh
```

下载器会保存大文件的 `.part` 断点和 pip 缓存，重复执行会续传或复用已完成资产。验收脚本逐项
检查固定版本的 ARM64 wheel，以及 cuSPARSELt、YOLOE 和 MobileCLIP 的 SHA-256。输出位于
`dist/`，包含 `.tar.gz` 和对应 `.sha256`。构建脚本会排除虚拟环境及其备份、旧输出、pip
缓存和本地 AimDK 下载，为归档内每个文件生成 `offline/MANIFEST.sha256`。离线包不会安装或
覆盖系统 ROS、AimDK、CUDA 驱动或机器人固件。

## 5. 分层验收

### 5.1 环境和无头规划

```bash
.planning-venv/bin/python -c \
  'import mujoco,numpy,pinocchio,x2_ik_sdk; print("planning: PASS")'
.vision-venv/bin/python -c \
  'import cv2,torch,ultralytics; print("cuda:", torch.cuda.is_available())'
./run.sh --no-vision --headless
```

最后一条命令不会接触机器人，但会加载官方 URDF、编译 MuJoCo 模型并完成碰撞规划。

### 5.2 AimDK 能力

```bash
source tools/setup_x2_mc_env.sh
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/check_aimdk_overlay.py
```

官方 v1.0.0 预期能力：

```text
animation, base, hal-joint, omnipicker
```

旧 overlay 要验证 MC 分控时使用：

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/check_aimdk_overlay.py \
  --require-capability upper-body
```

只读真机前检：

```bash
ros2 run graspv2 x2_aimdk_hardware preflight \
  --component all --transport hal-joint
```

## 6. RGB-D 采集与识别

### 6.1 四种后端

| 后端 | 行为 | 使用场景 |
| --- | --- | --- |
| `auto` | 先订阅 X2 topic，仅在同步超时后回退本地 SDK | 完整流水线默认 |
| `x2-aimdk` | 只订阅 X2 官方 RGB/Depth/CameraInfo ROS topic | topic 调试 |
| `orbbec-sdk` | 只调用 `build/orbbec_capture` 本地 SDK 二进制 | SDK 调试 |
| `existing` | 读取已有 `color.png`、`depth.png`、`camera.json` | 离线调试 |

完整流水线默认 `auto`；单独运行 `run_vision.sh` 时默认 `existing`。`auto` 只对“RGB、Depth
或 CameraInfo 在超时前未能同步”回退 SDK；图像格式、frame_id、配准等数据契约错误仍直接
退出。真机执行禁止 `existing`，不会使用旧图片兜底。

测试自动回退：

```bash
./run_vision.sh --capture-backend auto --capture-only
```

只测试 X2 topic、禁止回退：

```bash
source tools/setup_x2_mc_env.sh
./run_vision.sh --capture-backend x2-aimdk --capture-only
```

只测试 Orbbec SDK：

```bash
./tools/build_orbbec_capture.sh
./run_vision.sh --capture-backend orbbec-sdk --capture-only
```

### 6.2 图像方向

默认 `--image-rotation-deg auto`：先保持相机原始 `0°` 表示；仅当桌面拟合抛出专用失败状态
42 时，才同步旋转 RGB、Depth、内参和畸变参数到 `180°` 重试。目标类别缺失、模型加载、
候选过滤或其他深度错误不会触发方向切换。`0°` 是正装假设，`180°` 是倒装假设；自动模式
对每个假设直接使用标称外参，避免再按另一台机器的方向配置额外补偿一次180°。

显式使用 `--image-rotation-deg calibrated` 时，才从 `--camera-calibration` 指定的本机标定文件
直接读取安装方向。每台机器的标定文件必须同时包含以下一致字段：

```json
{
  "model_values": {
    "camera_mount_orientation": "upright",
    "capture_image_rotation_deg": 0
  }
}
```

现场机倒装对应：

```json
{
  "model_values": {
    "camera_mount_orientation": "inverted",
    "capture_image_rotation_deg": 180
  }
}
```

方向字段矛盾时流程会在采集前退出。`T_mujoco_camera_nominal` 和
`point_offset_mujoco_m` 仍须在对应机器上实测，不能把测试机外参直接复制到现场机。

建议分别保存，例如：

```bash
# 测试机
./run_full_grasp_pipeline.sh --target-class cup \
  --camera-calibration config/camera_test_upright.json --plan-only

# 现场机
./run_full_grasp_pipeline.sh --target-class cup \
  --camera-calibration config/camera_field_inverted.json --plan-only
```

也可在每台机器的启动环境中固定 `GRASPV2_CAMERA_CALIBRATION`，完整流水线及两次真机视觉
复检会继承同一文件；命令行 `--camera-calibration` 仍可显式覆盖。

可在不采集图像时检查配置选择的方向：

```bash
/usr/bin/python3 vision/rgbd_orientation.py \
  --calibration config/camera_test_upright.json \
  --print-calibrated-rotation
```

`--image-rotation-deg auto` 的判据只来自桌面拟合，不能用目标是否识别成功来判定安装方向。
若 `0°` 和 `180°` 都无法识别桌面，流程恢复 `0°` 后失败；若 `180°` 已识别桌面但后续目标
门失败，则保留已验证的 `180°` 并报告后续错误。

`--capture-only` 不运行桌面识别，因此 `auto` 保持 `0°` 并标记为 `auto-unverified`；
`calibrated` 会记录配置方向。固定方向用于调试：

```bash
./run_vision.sh --capture-backend x2-aimdk \
  --image-rotation-deg 0 --capture-only
```

方向重试不是相机后端回退。相机发生物理拆装或翻转后仍需更新安装方向并重新标定外参。

### 6.3 重复识别已有帧

```bash
./run_vision.sh \
  --capture-backend existing \
  --classes "orange-capped pill bottle" \
  --target-class "orange-capped pill bottle" \
  --conf 0.20 --device 0 \
  --calibration /path/to/x2_camera_calibration.json
```

成功后检查 `output/annotated.png`、`result.json` 和 `camera.json.image_orientation`。

### 6.4 IK 最近可达点

当前抓取使用位置-only 官方 IK。若 SDK 因 `max iterations reached` 返回失败，但 FK 计算的
位置误差不超过 `0.05 m`，规划器不会直接使用“最后一次”结果，而会检查全部 seed，并选择
误差最小且同时通过 MuJoCo 状态碰撞和上一轨迹边检查的候选。规划报告保留：

- `requested_gripper_center_world_m`：视觉请求的夹爪中心；
- `gripper_center_world_m`：实际采用的最近可达中心；
- `nearest_ik_fallback_maximum_error_m`：采用候选中的最大位置残差；
- `nearest_ik_fallbacks`：每个回退 waypoint 的请求点、实际点和误差。

阈值只对位置残差生效，不接受姿态约束失败；也不会绕过桌面间隙、碰撞、轨迹边、速度、抬升
进度或返回默认位检查。误差大于 5 cm 时仍输出 `official IK found no solution`。

## 7. 规划、Animation 与真机执行

### 7.1 仿真规划

```bash
# 使用当前视觉结果并打开 Viewer
./run.sh --vision-result output/result.json

# Orin、SSH 或 CI
./run.sh --vision-result output/result.json --headless

# 生成完整接近、抬升和返回轨迹，不连接机器人
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --use-existing-vision --plan-only
```

规划报告至少应满足：`verified_collision_free=true`、IK 后端为官方 SDK、TCP 与现场一致、
桌面间隙合格、三段轨迹首尾连续且 FK 对齐误差合理。

### 7.2 MC animation

```bash
./run.sh --mode animation
./run.sh --mode animation --animation /path/to/action.csv
./run.sh --mode animation --execute
```

20 列 CSV 本身不包含手部列，内部包含返回默认位路径；播放器明确调用仓库内的
`omnipicker_hand_student.py` SDK，在任何机械臂动作请求前持续 3 秒执行 `open right`、目标保持段开始时并行执行
`close right`。新生成动作默认在目标位保持约 2 秒。SDK、DDS、控制器或控制线失败只记录
warning，不会中断手臂播放和反向返回；必要时可加 `--no-gripper` 只播放手臂。真机播放前仍
会检查 `STAND_DEFAULT/100`、14 轴健康/静止、默认起点、速度、文件校验和和最终返回。

比赛机 profile 会在本机原子安装 CSV 到 `/tmp/graspv2_mc_animation.csv`，并强制要求上述
OmniPicker SDK 加载及初始命令成功。测试 profile 才通过 SSH 上传到测试机器人，并保留未接
夹爪控制线时的 best-effort 行为。

### 7.3 upper-body 自动回退

单条已验证轨迹默认先尝试旧 MC upper-body 分控。接口或服务在计划轨迹开始之前失败时，
自动执行预先生成的 animation：

```bash
ros2 run graspv2 x2_aimdk_hardware trajectory \
  --trajectory output/planned_trajectory.json \
  --upper-body-fallback animation \
  --execute --confirm-control-authority
```

`animation` 是默认值；`--upper-body-fallback none` 可关闭。安全边界：

- 计划 upper-body 轨迹开始后禁止跨控制器重放；输入源激活 HOLD 帧不算计划运动；
- animation 自身前检不通过时拒绝运动；
- 不会自动选择低层 `hal-joint`；
- 完整视觉抓取只在计划轨迹开始前回退，移动前全开夹爪命令不阻止回退。回退 animation 包含移动前全开 3 秒、安全预备/下降、
  按目标半径闭合、抬升、悬停、受控放下、松开、张开撤离、空夹爪闭合和回默认位；
  单条 MC animation 无法等待视觉结果，日志会明确提示该路径没有中间视觉门。

### 7.4 完整视觉抓取

联网部署：

```bash
source tools/setup_x2_mc_env.sh
./run_full_grasp_pipeline.sh \
  --target-class "orange-capped pill bottle" \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --execute --confirm-calibrated
```

离线部署使用同样参数调用 `./offline_run.sh`。看到提示后输入 `RUN`。流程为：现场采集 →
类别门 → MuJoCo 三段规划 → 机器人侧向外抬高到安全预备点 → 高位移动到物体顶部上方
3 cm → 完全打开 → 垂直下降到物体
中值高度上方 1 cm → 按半径夹紧 → 视觉门 → 抬升 → 防掉落视觉门 → 保持约 2.5 秒 →
夹紧下降放回 → 完全打开释放 → 全开退回预抓点 → 闭合空夹爪 → 验证返回。

完整 `grasp` 优先使用旧 `UpperBodyCommandArray` 完成视觉状态机。官方 v1.0.0 没有该消息
或接管准备失败时，只要计划轨迹尚未开始，就自动运行预先生成的 animation；这个降级
结果不冒充包含视觉门和三段恢复语义的完整抓取。

## 8. 常见故障

| 现象 | 优先检查 |
| --- | --- |
| 找不到 AimDK overlay | `GRASPV2_AIMDK_SETUP`、SDK/固件版本、`select_aimdk_setup.sh` |
| `Missing graspv2 install` | 重新运行构建脚本并 source `setup_x2_mc_env.sh` |
| RGB-D 超时 | 四个 topic、发布频率、DDS 网卡、CameraInfo、超时参数 |
| 两个方向都识别失败 | 驱动是否已旋转、RGB-D 对齐、桌面遮挡、外参和类别文本 |
| 彩色正确但深度错位 | 两套 CameraInfo、时间戳、frame_id 和 `depth_registration` |
| CUDA 不可用 | Jetson PyTorch、cuSPARSELt、CUPTI、`LD_LIBRARY_PATH` |
| YOLOE 无目标 | 模型、MobileCLIP、类别、置信度、深度范围和拒绝原因 |
| 桌面拟合失败 | ROI、遮挡、外参、倾角、高度和桌面尺寸门限 |
| IK 不可达或碰撞 | TCP、相机外参、工作空间、桌面盒和默认起点 |
| 真机起点/跟踪失败 | 模式、关节顺序、输入源、50 Hz、过热/故障和机械阻挡 |

### 8.1 X2 相机失败但没有切换 Orbbec

先确认使用了默认 `--capture-backend auto`；显式 `x2-aimdk` 不会回退。自动回退只响应
topic 同步超时（采集器退出码 20），数据格式、frame_id、配准等错误会保留原错误并停止。
若提示 `Orbbec SDK capture binary is missing`，运行 `./tools/build_orbbec_capture.sh`；仍找不到
SDK 时设置 `GRASPV2_ORBBEC_SDK_ROOT`。真机执行不会用旧图片兜底。

### 8.2 upper-body 没有回退 animation

- `animation fallback blocked after planned motion started ...`：计划轨迹已经开始，安全门禁止切换；
- `trajectory has no planning verification`：轨迹未通过官方 IK 和 MuJoCo 门；
- animation 前检失败：回退已启动，但机器人模式、健康或起点不安全；
- 完整 `grasp` 只有在计划轨迹开始前回退；运动开始后不会跨控制器重放。
- 约 `12000` 的多关节 `error_code` 可能是旧温度双字节被 v1.0 类型合并显示；程序要求整帧
  模式匹配才解码，单个非零错误仍会停止。

### 8.3 不要通过放宽门限解决的问题

不要首先提高 `maximum_start_error_rad`、`maximum_tracking_error_rad`，也不要随意移动视觉
点、降低桌面间隙或绕过 MuJoCo。应先检查真实标定、ROS 数据、关节顺序、默认姿态和机械
状态。

## 9. 问题归档、升级和回滚

故障后先保存本次 `output/`，避免下一次运行覆盖：

```bash
debug_dir=/tmp/graspv2-debug
mkdir -p "$debug_dir"
cp -a output "$debug_dir/"
cp config/mujoco_camera_calibration.json "$debug_dir/"
cp config/tool_pose_offset.json "$debug_dir/"
cp config/x2_aimdk_hardware.json "$debug_dir/"
tar -C /tmp -czf /tmp/graspv2-debug.tar.gz graspv2-debug
sha256sum /tmp/graspv2-debug.tar.gz
```

同时记录命令、代码/归档 SHA-256、标定 SHA-256、ROS/AimDK/CUDA/PyTorch 版本、图像方向和
终端日志。现场图片与机器人坐标可能是敏感数据，传出前按数据管理要求处理。

升级不要覆盖正在工作的目录。新版本使用独立目录，复制并复核现场标定，重新安装/构建，
再按第 5 节逐层验收。保留上一个完整目录；代码、模型、相机外参、TCP 和硬件配置必须作为
一个部署单元一起回滚。

## 10. 现场最短检查清单

每次开机：

```bash
source tools/setup_x2_mc_env.sh
./run.sh --no-vision --headless
./run_vision.sh --capture-backend x2-aimdk --capture-only
ros2 run graspv2 x2_aimdk_hardware preflight \
  --component all --transport hal-joint
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --plan-only
```

全部通过并完成现场安全确认后，才把最后一条改为：

```bash
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --execute --confirm-calibrated
```
