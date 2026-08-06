# graspV2 比赛真机运行命令

本文按比赛机目录 `/home/svt/Raicom2026-old/graspV2` 编写。所有命令都应在比赛机终端中
手工执行；部署代码本身不会启动相机或机器人动作。

## 0. 现场安全约束

- OmniPicker 的模式、设备 ID、CAN/EtherCAT 和固件由赛项预先配置，不要停止或重启 MC，
  不要修改这些参数。
- 第一次动作必须使用吊架/固定底座，保证实体急停可用，并清空机械臂运动范围。
- X2 头部相机必须使用其自己的 camera-to-robot 标定文件，不能复用测试机 Orbbec 外参。
- 不带 `--execute` 时不会发布机器人控制命令；真机执行还需要
  `--confirm-calibrated` 并手工输入 `RUN`。

## 1. 进入目录并构建

```bash
cd /home/svt/Raicom2026-old/graspV2
chmod +x run.sh run_vision.sh run_full_grasp_pipeline.sh tools/*.sh
./tools/build_graspv2_with_installed_aimdk.sh
source tools/setup_x2_mc_env.sh
```

每次打开新终端都重新执行：

```bash
cd /home/svt/Raicom2026-old/graspV2
source tools/setup_x2_mc_env.sh
```

## 2. X2 官方 RGB-D topic 检查

默认接口：

```text
/aima/hal/sensor/rgbd_head_front/rgb_image
/aima/hal/sensor/rgbd_head_front/depth_image
/aima/hal/sensor/rgbd_head_front/rgb_camera_info
/aima/hal/sensor/rgbd_head_front/depth_camera_info
```

只读查看 topic，不控制机器人：

```bash
ros2 topic info /aima/hal/sensor/rgbd_head_front/rgb_image --no-daemon
ros2 topic info /aima/hal/sensor/rgbd_head_front/depth_image --no-daemon
ros2 topic info /aima/hal/sensor/rgbd_head_front/rgb_camera_info --no-daemon
ros2 topic info /aima/hal/sensor/rgbd_head_front/depth_camera_info --no-daemon
```

采集一组 RGB/Depth 和两套内参，不运行识别：

```bash
./run_vision.sh --capture-backend x2-aimdk --capture-only
```

识别一次目标，以瓶子为例：

```bash
./run_vision.sh --capture-backend x2-aimdk \
  --classes bottle --target-class bottle --conf 0.20 --device 0 \
  --calibration /absolute/path/to/x2_camera_calibration.json
```

## 3. 上肢和 OmniPicker 只读预检

```bash
source tools/setup_x2_mc_env.sh
ros2 run graspv2 x2_aimdk_hardware preflight \
  --robot ultra --component all --transport upper-body
```

预检必须确认：

- MC 当前为 `STAND_DEFAULT/100`；
- `/mc/upper_body_command`、手臂/头部反馈和 hand command consumer 可用；
- 手臂反馈新鲜、静止、无故障且温度正常；
- OmniPicker topic 为 `/aima/hal/joint/hand/command`，比赛命令名为
  `left_claw_joint/right_claw_joint`。

夹爪命令 dry-run，不发布：

```bash
ros2 run graspv2 x2_aimdk_hardware omnipicker --side right --action open
ros2 run graspv2 x2_aimdk_hardware omnipicker --side right --action close
```

## 4. 离线规划和完整状态机 dry-run

只生成并验证接近轨迹和 `0.10 m / 2.0 s` 抬升轨迹：

```bash
./run_full_grasp_pipeline.sh \
  --target-class bottle \
  --robot ultra \
  --side auto \
  --capture-backend x2-aimdk \
  --camera-calibration /absolute/path/to/x2_camera_calibration.json \
  --plan-only
```

去掉 `--plan-only` 会继续检查完整抓取状态机契约，但仍不连接控制接口、不重新采集两个视觉
检查点：

```bash
./run_full_grasp_pipeline.sh \
  --target-class bottle \
  --robot ultra \
  --side auto \
  --capture-backend x2-aimdk \
  --verification-capture-backend x2-aimdk \
  --camera-calibration /absolute/path/to/x2_camera_calibration.json
```

## 5. 比赛真机视觉闭环抓取

确认第 2～4 节全部通过后执行：

```bash
cd /home/svt/Raicom2026-old/graspV2
source tools/setup_x2_mc_env.sh

./run_full_grasp_pipeline.sh \
  --target-class bottle \
  --robot ultra \
  --side auto \
  --capture-backend x2-aimdk \
  --verification-capture-backend x2-aimdk \
  --camera-calibration /absolute/path/to/x2_camera_calibration.json \
  --lift-height 0.10 \
  --lift-duration 2.0 \
  --verification-timeout 45 \
  --execute \
  --confirm-calibrated
```

看到提示后再次检查急停、人员和目标位置，手工输入：

```text
RUN
```

真机状态机顺序：

1. 打开目标侧 OmniPicker；
2. 执行碰撞验证的预抓取和接近轨迹；
3. 闭合夹爪；
4. 重新采集 RGB-D，确认同类目标仍处于抓取区；
5. 沿桌面法向抬升 `0.10 m`，动作持续 `2.0 s`；
6. 再次采集 RGB-D，确认目标跟随夹爪抬升且没有掉回桌面；
7. 成功或视觉失败时均沿验证路径下降、松爪、撤回，然后恢复稳定模式。

视觉采集和推理期间，上肢控制线程会继续以 50 Hz 发布保持帧。目标消失、类别错误、位移
不足、横向漂移过大或检查超时都会判失败。

执行结果：

```bash
cat output/grasp_status.json
```

成功时应包含：

```json
{
  "state": "complete",
  "success": true
}
```

## 6. 当前测试机手动 Orbbec SDK 模式

构建并只采集：

```bash
./tools/build_orbbec_capture.sh
./run_vision.sh --capture-backend orbbec-sdk --capture-only
```

使用 Orbbec SDK 完成初始采集和两个真机视觉检查点：

```bash
./run_full_grasp_pipeline.sh \
  --target-class bottle \
  --robot ultra \
  --capture-backend orbbec-sdk \
  --verification-capture-backend orbbec-sdk \
  --camera-calibration config/mujoco_camera_calibration.json
```

已经手工采集到 `output/color.png`、`output/depth.png`、`output/camera.json` 时，可用：

```bash
./run_full_grasp_pipeline.sh \
  --target-class bottle \
  --robot ultra \
  --capture-backend existing \
  --verification-capture-backend orbbec-sdk \
  --camera-calibration config/mujoco_camera_calibration.json
```

`existing` 只用于初始规划；闭合后和抬升后始终要求新鲜采集，不能复用旧的
`output/result.json`。

## 7. 常用参数

```text
--robot youth|ultra
--side auto|left|right
--vision-conf 0.20
--table-clearance 0.025
--lift-height 0.10
--lift-duration 2.0
--close-target-tolerance 0.08
--lifted-target-tolerance 0.10
--minimum-lift-ratio 0.60
--maximum-lateral-drift 0.08
--verification-timeout 45
```

除非完成新的实机标定和安全验证，不要放宽视觉/碰撞门限，也不要使用 `--yes` 跳过最终
`RUN` 确认。
