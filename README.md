# graspV2

使用官方 `x2_ik_sdk` 做 IK，并在 MuJoCo 中完成关节限位、桌面距离和整条轨迹碰撞
检查。仿真与真机统一使用 X2 Ultra（每臂 7 轴），现场仅右手安装 OmniPicker。

## 基本命令

仓库只提供一个公开入口：

```bash
./run.sh [参数]
```

不加参数时等价于：

```bash
./run.sh --mode sim \
  --vision-result output/result.json
```

它默认复用上一次视觉识别结果：以 `grasp_point_mujoco_m` 为目标，把
`table_plane_mujoco.mujoco_collision_box` 作为 MuJoCo 障碍物，然后完成 Ultra 右臂
规划和整条轨迹碰撞验证，随后默认打开 MuJoCo Viewer，显示实际机器人、视觉桌子、
地面和灯光。不会连接机器人或发送控制指令；Orin/SSH 使用 `--headless` 关闭窗口。
若 `output/result.json` 不存在，命令会明确报错，
不会悄悄改用假目标。

首次运行会在仓库内创建 `.venv` 并安装 Pinocchio、MuJoCo 等依赖。

在 WSL2/WSLg 中可直接运行相同命令。项目默认使用 `llvmpipe` 软件 OpenGL，避免
GPU/D3D12 不可用时 MuJoCo Viewer 在退出阶段崩溃；这不影响 IK、碰撞检查和轨迹结果，
但窗口渲染速度会慢一些。确认 `glxinfo -B` 显示硬件加速后，可用
`GRASPV2_WSL_SOFTWARE_GL=0 ./run.sh --no-vision` 恢复 WSLg 硬件渲染。

查看命令内置帮助：

```bash
./run.sh --help
```

## 两种运行模式

### 仿真规划 `--mode sim`

执行流程为：官方 IK → 预抓取点 → 碰撞安全 IK → RRT/直线路径 → minimum-jerk
采样 → MuJoCo 密集碰撞检查 → 写出轨迹和报告。

当前 MuJoCo 模式会执行并回放右臂运动学轨迹、桌面碰撞和间隙检查；目标物仍是
display-only，右侧 OmniPicker 也尚未做闭合接触与物体抬升的动力学仿真。完整的
打开/闭合/视觉确认/抬升状态机属于显式确认后的真机执行路径。

```bash
# Ultra 右臂、上次视觉抓取点和视觉桌面，规划后打开 Viewer
./run.sh

# 使用手工目标覆盖视觉抓取点；默认视觉桌面障碍仍保留
./run.sh --target 0.38 -0.30 0.92

# 完全禁用视觉输入和桌面障碍，运行内置可达目标冒烟测试
./run.sh --no-vision

# 无图形桌面时只运行规划和验证
./run.sh --headless

# 显式禁用视觉桌子，改用内置目标（仍默认打开 Viewer）
./run.sh --no-vision

# 显式指定实际机器人全身 URDF（通常无需指定，会自动发现）
./run.sh --no-vision \
  --robot-urdf ~/x2_arm_sim/urdf/x2_ultra.urdf
```

手工坐标使用仿真世界坐标系：`+X` 向机器人前方、`+Y` 向机器人左侧、`+Z`
向上，单位均为米。

仿真的运动学始终来自官方 `x2_ik_sdk` URDF。Viewer 的实际机器人外观会自动从
`GRASPV2_ROBOT_URDF`、`~/x2_arm_sim/urdf/x2_ultra.urdf`，以及
`~/下载`/`~/Downloads` 下的 `X2_URDF-v1.3.0` 中查找。实际 URDF 的全身 visual mesh
按同名 link 挂到官方运动学链；规划碰撞仍使用独立保守代理，避免 visual 网格更新改变
安全判定。若机器上没有实际 URDF，headless 规划仍可运行并明确报告只使用代理模型。

右侧夹爪外观固定读取项目内的 `robot_description/urdf/robot_urdf.xacro` 和
`robot_description/meshes/*.STL`，并挂载到官方 IK URDF 的 `R_` link。MuJoCo 不再给
左侧挂载 OmniPicker，因此夹爪外观和碰撞几何与现场单右夹爪机器人一致；原来的橙色夹爪
方盒只保留为不可见的保守碰撞代理。
`base_link.STL` 超过 MuJoCo 的 20 万面限制，项目使用
`robot_description/meshes/mujoco/base_link.STL` 缓存；替换源 STL 后运行
`./.venv/bin/python tools/prepare_omnipicker_meshes.py` 重新生成。

### Animation `--mode animation`

不加 `--execute` 时完全离线，不连接机器人。

```bash
# 将默认已验证轨迹转换成 output/mc_animation.csv
./run.sh --mode animation

# 指定 Ultra 轨迹 JSON，且必须有碰撞验证记录
./run.sh --mode animation \
  --trajectory output/ultra_trajectory.json

# 校验并准备重放已有 CSV，不修改该 CSV
./run.sh --mode animation \
  --animation /path/to/action.csv
```

如果默认轨迹存在，animation 模式会复用这条已验证轨迹，保留“本地动作重放”的行为。
如果轨迹不存在，则先按默认视觉结果执行仿真规划；传入 `--vision-result`、`--target`
或 `--no-vision` 时也会强制重新规划。生成的 CSV 包含静止 lead-in、正向动作、保持、
逆序退回和 MC 默认姿态，完整使用 Ultra 的 14 个手臂关节列。

## 使用视觉结果

默认视觉文件是 `output/result.json`，因此通常直接运行：

```bash
./run.sh
```

也可显式指定其他视觉 JSON。文件必须包含 `grasp_point_mujoco_m`、桌面平面以及
`mujoco_collision_box`：

```bash
./run.sh \
  --vision-result output/result.json
```

同时提供 `--target` 时，手工坐标覆盖视觉抓取点，但仍使用视觉 JSON 中的桌面碰撞盒。
不必重复写默认视觉文件路径：

```bash
./run.sh --target 0.38 -0.30 0.92
```

MuJoCo Viewer 在规划成功后默认启动；其中显示的桌面顶部就是碰撞检查使用的障碍物，
四条桌腿只用于场景显示。场景的地面、渐变天空、主光、补光和默认相机参考
`~/x2_arm_sim/scene.xml`。若当前视觉抓取点不可达，仍会打开静态场景供检查，关闭窗口
后以失败状态退出；不会把不可达目标替换成假目标。Orin、SSH 或 CI 环境使用
`--headless`。

### 桌面 3D 校准与 tool/TCP offset

`config/mujoco_camera_calibration.json` 中：

```json
"point_offset_mujoco_m": [0.0, 0.0, 0.0]
```

不是 tool/TCP offset。它是在相机点经过 `T_mujoco_camera_nominal` 变换后，对所有视觉
点统一增加的 MuJoCo 世界坐标平移校准量；抓取点、桌面中心和桌面范围会一起移动。
例如 X 填 `-0.02` 表示把全部视觉结果沿机器人前后方向修正 `-2 cm`，只能根据实测
标定填写，不能为让 IK 成功而随意调整。

桌面检测现在使用完整的 3D 流程：限定高度与倾角的 RANSAC 平面、图像空间连通区域
筛选、平面坐标系中的有向矩形拟合。矩形中心、旋转、四角和 MuJoCo 碰撞盒由同一个
拟合结果生成，不再用散点分位数中心拼接碰撞盒。可在同一标定文件的
`table_detection` 中调整 `roi_uv_fraction`、`height_range_mujoco_m`、
`max_tilt_from_horizontal_deg`、`distance_threshold_m` 和桌面边长允许范围。

实际抓取中心 TCP 在 `config/tool_pose_offset.json` 的 `right` 项调整：

```json
"right": {
  "parent_frame": "R_omnipicker_base_link",
  "translation_m": [0.0, 0.0, 0.0],
  "rpy_rad": [0.0, 0.0, 0.0]
}
```

`translation_m` 和 `rpy_rad` 都是在 OmniPicker 基座的局部坐标系中表达，旋转单位是
弧度；修改后重启 `run.sh`。同一固定 TCP 帧同时用于 Pinocchio IK/FK、MuJoCo 红色 TCP
标记和 FK 对齐检查，因此不要再用相机的 `point_offset_mujoco_m` 补偿工具长度。
规划报告的 `active_tcp_offset` 会记录实际生效值。`left` 项仅为官方双臂运动学模型保留，
不会用于抓取规划或夹爪命令；右侧默认值为全零，待用 CAD 或实测值替换。

各字段对应的局部坐标方向、RPY 顺序、毫米级标定步骤、常用角度换算以及待抓物碰撞
策略，见 `docs/tool_offset_calibration_zh.md`。

主要规划参数：

- `--table-clearance 0.025`：机器人代理几何到桌面的最小允许距离，单位米；范围
  `0`～`0.10`。
- `--approach-distance 0.075`：预抓取点沿桌面法向高于目标的距离，单位米；范围
  `0.01`～`0.30`。
- 比赛机只有右手安装 OmniPicker，抓取固定使用右臂；右臂不可达时会安全失败，不会改用
  左臂。旧命令中的 `--side right` 仍可使用。
- `--robot-urdf PATH`：覆盖自动发现的实际机器人全身 visual URDF。
- `--vision-result PATH`：覆盖默认 `output/result.json`。
- `--no-vision`：明确禁用视觉目标和桌面障碍；不能和 `--vision-result` 同用。

## 输出文件

默认输出：

- `output/planned_trajectory.json`：关节轨迹、机型、手臂和规划验证元数据。
- `output/planning_report.json`：IK 误差、最小桌面距离、碰撞检查次数、RRT 信息、
  速度和 SDK/MuJoCo FK 对齐误差。
- `output/mc_animation.csv`：MC `animation_player` 使用的 20 列 CSV。

可分别通过下面参数修改路径：

```bash
./run.sh \
  --trajectory output/ultra.json \
  --report output/ultra_report.json
```

`--speed-scale` 只影响生成的 animation，范围为 `0 < 值 <= 1`，默认 `0.5`；数值越
小，动作越慢。它不修改原始规划 JSON。

## X2 真机硬件接口与双相机后端

项目已把真实硬件隔离为独立 AimDK 适配层，规划和 MuJoCo 不会隐式加载 ROS 或连接
机器人：

- 上肢：默认接 `/mc/upper_body_command` 的 MC 分控接口，并保留
  `arm`、`waist`、`head` HAL 端口；
- OmniPicker：使用 `HandCommandArray/HandStateArray` 的右侧夹爪端口；
- 深度视觉：X2 默认订阅官方 `rgbd_head_front` RGB、Depth 和两套 CameraInfo；
- 相机方向：真机 RGB-D 倒装，采集后默认将彩色图、深度图和内参同步旋转 `180°`；
- 测试机：保留直接调用现有 Orbbec SDK 二进制，以及消费手动采集文件的模式。

```bash
# X2 官方 RGB-D topic
./run_vision.sh --capture-backend x2-aimdk --capture-only

# 相机改为正装后关闭倒装修正
./run_vision.sh --capture-backend x2-aimdk --image-rotation-deg 0 --capture-only

# 当前测试机直接调用相机 SDK
./run_vision.sh --capture-backend orbbec-sdk --capture-only

# 已手动调用 SDK 并写入 output/ 后继续识别
./run_vision.sh --capture-backend existing \
  --classes "orange-capped pill bottle" \
  --target-class "orange-capped pill bottle" --device 0

# 真机只读接口检查，不发布任何控制命令
source tools/setup_x2_mc_env.sh
ros2 run graspv2 x2_aimdk_hardware preflight --component all
```

上半身轨迹默认使用 `UpperBodyCommandArray`，自动校验稳定站立模式并进入/退出
`UPPERBODY_REMOTE_SPLIT`；`--transport hal-joint` 是低层后备。topic、服务和控制参数集中在
`config/x2_aimdk_hardware.json`。所有真机发布默认禁用，并且需要两个显式执行确认参数。
接线、构建 overlay、相机外参隔离和安全前提详见
`docs/x2_hardware_interfaces_zh.md`。

YOLOE 默认比赛类别为 `cup`、`orange-capped pill bottle` 和
`bag of corn bread`。在按置信度选目标之前，视觉流程会先要求掩膜中心位于投影桌面区域，
并要求该处的稳健深度点至少高出桌面 `0.01 m`；同时用三维桌面范围做交叉检查。阈值位于
`config/mujoco_camera_calibration.json` 的 `candidate_filter`，每个候选的通过状态和拒绝原因
都会写入 `output/result.json`。

比赛夹爪契约只命令 `right_claw_joint`（左侧命令数组保持为空），并使用
`BEST_EFFORT + TRANSIENT_LOCAL` QoS。完整抓取流水线现在使用 UpperBody 分控和独立 hand
HAL 组成状态机：打开 → 接近 → 闭合 → 视觉确认 → 两秒抬升 → 视觉防掉落确认 → 下降释放
→ 撤回。视觉检查期间持续发布上肢保持帧；仍在桌面、位置漂移过大或无法重新识别的目标均
判为失败。

```bash
# 只规划接近/抬升并检查完整状态机契约，不连接机器人
./run_full_grasp_pipeline.sh --target-class "orange-capped pill bottle" \
  --use-existing-vision

# 真机必须提供独立标定确认并再次输入 RUN
# 执行入口会强制现场重新采集 RGB-D，不接受 --use-existing-vision
./run_full_grasp_pipeline.sh --target-class "orange-capped pill bottle" \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --execute --confirm-calibrated
```

默认抬升 `0.10 m / 2.0 s`，视觉门限和测试机 `orbbec-sdk` 检查点后端均可通过完整流水线
参数覆盖。状态结果写到 `output/grasp_status.json`；详细时序和失败恢复见
`docs/x2_hardware_interfaces_zh.md`。

## 真机 Animation 播放

真机播放必须显式增加 `--execute`：

```bash
./run.sh --mode animation --execute
```

程序按以下顺序执行：

1. 校验轨迹或已有 CSV 的格式、关节限位、速度和最终默认姿态。
2. 只读检查 MC 为稳定站立状态、手臂无故障、温度和速度正常。
3. 严格校验真机收到 Ultra 的 14 轴手臂反馈。
4. 显示本地文件、SHA-256 和真机临时路径，等待输入 `RUN`。
5. 上传到真机 `/tmp`，校验 SHA-256 和 `0644` 权限。
6. 通过 `SetMcPresetMotion(ani_path)` 请求一次不打断其他动作的播放。
7. 只读观察动作启动以及最终回到默认姿态。

第一次真机执行必须使用吊架并确保实体急停可用。机器人需先通过自身操作方式进入稳定
站立状态。`--yes` 会跳过最终 `RUN` 确认，只应在外部已有等效安全确认时使用：

```bash
./run.sh --mode animation --execute --yes
```

## 参数限制

- `--animation` 只能用于 animation 模式，且不能和 `--target`、
  `--vision-result`、`--no-vision` 同用，因为已有 CSV 不会重新规划。
- sim 模式默认打开 Viewer；`--headless` 只做规划和验证，不打开窗口。
- `--execute` 只能用于 animation 模式。
- `--yes` 必须和 `--execute` 同用。
- `--robot ultra` 保留为旧命令兼容参数；其他机型会在参数解析阶段被拒绝。

命令退出码：`0` 表示成功，`1` 表示规划、校验或执行失败，`2` 表示命令行参数错误。
