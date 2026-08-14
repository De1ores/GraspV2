# GraspV2 本地计算 / X2 Orin 轻量部署

## 1. 部署边界

拆分后的数据流为：

```text
X2 Orin: AimDK RGB-D topic 订阅
    -> SSH 复制 color.png / depth.png / camera.json
本地工作站: YOLOE + 深度几何 + X2 官方 IK + MuJoCo 安全检查
    -> SSH 复制 manifest.json / grasp_animation.csv
X2 Orin: SHA-256 校验 + AimDK 只读前检 + OmniPicker/MC 最终执行
```

Orin 端不安装 Torch、YOLO、MuJoCo、Pinocchio、X2 IK SDK 或模型权重。机器人端只使用
`/usr/bin/python3`、ROS 2 Humble、随机器人固件安装的 AimDK、系统 OpenCV/NumPy 和一小组
控制/采集脚本。

相机接口来自 X2 AimDK 自带的 `echo_camera_rgbd.py` 示例，订阅：

- `/aima/hal/sensor/rgbd_head_front/rgb_image`
- `/aima/hal/sensor/rgbd_head_front/depth_image`
- `/aima/hal/sensor/rgbd_head_front/rgb_camera_info`
- `/aima/hal/sensor/rgbd_head_front/depth_camera_info`

四个 topic 均使用传感器 QoS（`BEST_EFFORT + VOLATILE`）。远程入口会优先使用
`GRASPV2_AIMDK_SETUP`，否则检查 `/home/agi/aimdk`、`/home/agi/AimDK`、`/opt/aimdk`，最后在
`/home/agi` 下有限深度搜索 `install/setup.bash`；候选目录只有通过
`rclpy/sensor_msgs/aimdk_msgs` 实际导入测试才会使用。

## 2. 本地环境安装

工作站需要能通过 SSH 访问机器人，并建议使用带 CUDA 的 NVIDIA GPU。联网安装：

```bash
cd /home/GraspV2
./tools/install_local_split.sh
```

如果 PyTorch 需要指定与本机 CUDA 对应的 wheel 源：

```bash
GRASPV2_TORCH_INDEX_URL=https://download.pytorch.org/whl/cu126 \
  ./tools/install_local_split.sh
```

脚本分别创建 `.planning-venv` 与 `.vision-venv`，所有重计算依赖只安装在本地。

## 3. Orin 轻量部署

机器人默认账户为 `agi@IP`。密码默认值 `1` 只通过 OpenSSH askpass 环境传入，不写进部署包、
配置或命令行；生产环境建议改用 SSH key，或者临时设置
`GRASPV2_ROBOT_PASSWORD`。

```bash
cd /home/GraspV2
./tools/deploy_robot_edge.sh --robot-host 10.0.200.40
```

部署目标默认为 `/home/agi/graspv2-edge`。成功输出应包含：

```text
robot_edge_imports=PASS
Remote GraspV2 edge installation complete
No Torch, YOLO, MuJoCo, Pinocchio or X2 IK packages were installed.
```

若机器人 AimDK 不在自动搜索范围内：

```bash
export GRASPV2_AIMDK_SETUP=/实际目录/install/setup.bash
./tools/install_remote_split.sh
```

这条变量应设置在机器人端。也可把它写入机器人服务的环境配置，但不要指向与固件不匹配的
overlay。

## 4. 分步检查

只从机器人采集一帧，不运行识别：

```bash
./run_vision.sh --capture-backend x2-remote \
  --robot-host 10.0.200.40 --capture-only
```

完整本地识别、规划、生成执行包，但不接触控制接口：

```bash
./run_split_grasp_pipeline.sh \
  --robot-host 10.0.200.40 \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json
```

执行包位于 `output/edge_execution/`，只包含：

- `grasp_animation.csv`：MC 完整动作和返回路径；
- `manifest.json`：机器人/夹爪事件、速度、源文件与 animation SHA-256。

可以单独执行机器人端只读前检：

```bash
./tools/remote_execute.sh --robot-host 10.0.200.40 \
  --bundle output/edge_execution --preflight
```

## 5. 真机运行

确认相机外参、TCP、机器人稳定站立、吊架/急停和周边空间后：

```bash
./run_split_grasp_pipeline.sh \
  --robot-host 10.0.200.40 \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --execute --confirm-calibrated
```

本地会先完成采集、识别、IK/MuJoCo 规划和执行包校验，再运行机器人端只读前检，最后要求输入
`RUN`。只有最后一步会请求 MC 播放。现场自动流程可显式加 `--yes`，首次测试不要使用。

## 6. 安全差异

轻量拆分模式使用单条 MC animation，MC 接管前进、抓取、抬升、放回和返回默认位。它不会在
闭合后、抬升后暂停等待本地视觉判断，因此不等价于 `run_full_grasp_pipeline.sh --execute` 的
两次中途视觉门。SHA-256、官方 IK/MuJoCo 证据、初始视觉、MC 模式、14 轴健康/静止、起点、
温度、夹爪初始化和最终返回检查仍保留。

如果任务必须在“闭合后看一次、抬升后再看一次”，应使用原本的同机闭环执行，或后续部署
一个保持 ROS 控制会话的双向状态机；不要把原子 animation 宣称为视觉闭环。
