# svt Orin 最短部署命令

目标：`svt@192.168.1.240`，项目目录：`/home/svt/graspV2`。
比赛真机的实测 OS/Agi/AimDK 环境、统一 source 顺序和 Motion 接口见
[`docs/competition_robot_environment_motion_zh.md`](docs/competition_robot_environment_motion_zh.md)。

## 1. 开发机打包并上传

```bash
cd /home/GraspV2
./tools/fetch_offline_dependencies.sh
./tools/build_offline_bundle.sh
scp dist/graspv2-x2-ultra-offline.tar.gz{,.sha256} \
  svt@192.168.1.240:/home/svt/
```

## 2. 机器人首次安装

```bash
ssh svt@192.168.1.240
cd /home/svt
sha256sum --check graspv2-x2-ultra-offline.tar.gz.sha256
tar -xzf graspv2-x2-ultra-offline.tar.gz
mv graspv2-x2-ultra-offline graspV2
cd graspV2
./install_offline.sh
```

成功时会看到 `headless_model_check=PASS`、`gpu=Orin` 和
`Offline GraspV2 installation complete`。安装器检测到 Orbbec SDK 时会同时生成
`build/orbbec_capture`；运行默认先等 X2 topic，超时后自动回退 SDK。图像先保持 `0°`，只有
桌面识别失败才同步旋转 RGB-D 到 `180°` 重试。

## 3. 识别和规划，不移动机器人

```bash
cd /home/svt/graspV2
export GRASPV2_RUNTIME_PROFILE=test
./offline_run.sh \
  --target-class cup \
  --camera-calibration config/mujoco_camera_calibration.json \
  --plan-only
```

## 4. 真机执行

确认机器人稳定站立、相机外参、TCP、吊架和急停后执行：

```bash
cd /home/svt/graspV2
export GRASPV2_RUNTIME_PROFILE=test
./offline_run.sh \
  --target-class cup \
  --camera-calibration config/mujoco_camera_calibration.json \
  --execute --confirm-calibrated  --yes
```

orange-capped pill
./offline_run.sh --target-class " bottle" \
  --camera-calibration config/mujoco_camera_calibration.json \
  --execute --confirm-calibrated --yes

./offline_run.sh --target-class "bag of corn bread" \
  --camera-calibration config/mujoco_camera_calibration.json \
  --execute --confirm-calibrated --yes
终端提示后输入 `RUN`。首次测试不要添加 `--yes`。带空格的类别必须加引号，例如
`--target-class "orange-capped pill bottle"`。

本页是 SVT 测试机部署，因此固定 `GRASPV2_RUNTIME_PROFILE=test`。animation 上传默认先连接
`agi@10.0.200.40`，连接或认证失败后自动尝试
`agi@10.0.1.40`。两个地址默认使用密码 `1`，由 OpenSSH askpass 自动提供，不需要安装
`sshpass`，也不会在终端交互询问。现场参数可通过环境变量覆盖：

```bash
export GRASPV2_ROBOT_ADDRESS=10.0.200.40
export GRASPV2_ROBOT_FALLBACK_ADDRESS=10.0.1.40
export GRASPV2_ROBOT_USER=agi
export GRASPV2_ROBOT_PASSWORD=1
```

这些地址和密码只属于测试模式。比赛机 `agi/aarch64` 使用本机 MC 和
`/home/agi/aimdk/install/setup.bash`，不会访问上述测试地址。

执行命令会预先生成 MC animation 安全回退。只要计划轨迹尚未开始，旧 upper-body 接管失败
就会自动切换；输入源激活 HOLD 帧和移动前全开夹爪命令不阻止回退。已经开始运动后仍立即停止，
不会跨控制器重放。回退的
机械臂和夹爪阶段与仿真一致，包括移动前全开 3 秒、安全预备、下降、按半径闭合、抬升/悬停、受控放下、
松开、撤离和回默认位。单条 MC animation 不能等待中间视觉判定，终端会明确提示
`MC animation fallback`。

单独执行 `./run.sh --mode animation --execute` 时，程序会调用仓库内
`omnipicker_hand_student.py` SDK，在任何 animation 移动请求前持续 3 秒执行 `open right`、到达目标保持段时并行执行
`close right`。测试机未接夹爪控制线时只提示 warning，手臂 animation 和返回动作仍继续。
