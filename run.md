# graspV2 现场速查

完整说明和故障处理见
[`docs/package_install_run_debug_zh.md`](docs/package_install_run_debug_zh.md)。本页只保留现场命令。
比赛机的 OS/Agi/AimDK 快照和 Motion 接口详见
[`docs/competition_robot_environment_motion_zh.md`](docs/competition_robot_environment_motion_zh.md)。

## 首次部署

联网机：

```bash
cd /path/to/GraspV2
chmod +x run.sh run_vision.sh run_full_grasp_pipeline.sh offline_run.sh tools/*.sh
./tools/fetch_official_aimdk_x2.sh
./tools/build_graspv2_with_installed_aimdk.sh
source tools/setup_x2_mc_env.sh
```

完全离线包：

```bash
cd /path/to/graspv2-x2-ultra-offline
./install_offline.sh
source tools/setup_x2_mc_env.sh
```

AimDK 必须与机器人固件匹配；旧固件先设置
`GRASPV2_AIMDK_SETUP=/path/to/aimdk/install/setup.bash`。

比赛机固定本机执行：

```bash
export GRASPV2_RUNTIME_PROFILE=competition
```

SVT/开发测试机固定测试 fallback：

```bash
export GRASPV2_RUNTIME_PROFILE=test
```

competition 使用 `/home/agi/aimdk/install/setup.bash` 和本机 `/tmp` animation，不会连接测试
IP；test 才使用配置的 SSH 地址。夹爪统一调用项目根目录的
`omnipicker_hand_student.py`。

## 每次开机检查

```bash
source tools/setup_x2_mc_env.sh

# 不连接机器人控制端
./run.sh --no-vision --headless

# 只读相机；失败不会自动切换 Orbbec 或历史图片
./run_vision.sh --capture-backend x2-aimdk --capture-only

# 只读 AimDK/关节/夹爪前检
ros2 run graspv2 x2_aimdk_hardware preflight \
  --component all --transport hal-joint

# 现场采集、识别和规划，不发布控制命令
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --plan-only
```

## 真机执行

确认吊架/站立控制、急停、人员隔离、相机外参、TCP 和 OmniPicker 后：

```bash
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --execute --confirm-calibrated
```

提示后输入 `RUN`。首次联调不要使用 `--yes`。

离线包把最后两条 `run_full_grasp_pipeline.sh` 命令替换为 `offline_run.sh`，参数不变。
