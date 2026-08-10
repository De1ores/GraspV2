# graspV2 现场指令

把 `10.0.200.REPLACE_ME` 换成现场机 IP。

## 1. 开发机上传

```bash
cd /home/GraspV2
tar --exclude='.git' --exclude='.venv' --exclude='.planning-venv' \
  --exclude='build' --exclude='install' --exclude='log' --exclude='output' \
  -czf /tmp/graspv2.tar.gz .

scp /tmp/graspv2.tar.gz \
  svt@10.0.200.REPLACE_ME:/home/svt/
scp /path/to/x2_camera_calibration.json \
  svt@10.0.200.REPLACE_ME:/home/svt/x2_camera_calibration.json
ssh svt@10.0.200.REPLACE_ME
```

## 2. 现场机部署

```bash
mkdir -p /home/svt/Raicom2026-old/graspV2
tar -xzf /home/svt/graspv2.tar.gz -C /home/svt/Raicom2026-old/graspV2
cd /home/svt/Raicom2026-old/graspV2
chmod +x run.sh run_vision.sh run_full_grasp_pipeline.sh tools/*.sh
./tools/build_graspv2_with_installed_aimdk.sh
source tools/setup_x2_mc_env.sh
```

现场机没有 Python 环境时，仅首次执行：

```bash
./tools/setup_orin_vision_env.sh
/usr/bin/python3 -m venv .planning-venv
.planning-venv/bin/python -m pip install -r requirements.txt
```

## 3. 检查

```bash
cd /home/svt/Raicom2026-old/graspV2
source tools/setup_x2_mc_env.sh
./run.sh --no-vision --headless
ros2 run graspv2 x2_aimdk_hardware preflight --component all
./run_full_grasp_pipeline.sh --target-class "orange-capped pill bottle" \
  --camera-calibration /home/svt/x2_camera_calibration.json --plan-only
```

## 4. 真机执行

确认急停、人员隔离、相机标定和右侧夹爪正常后：

```bash
cd /home/svt/Raicom2026-old/graspV2
source tools/setup_x2_mc_env.sh
./run_full_grasp_pipeline.sh --target-class "orange-capped pill bottle" \
  --camera-calibration /home/svt/x2_camera_calibration.json \
  --execute --confirm-calibrated
```

看到提示后输入 `RUN`。目标类别也可改为 `cup` 或 `"bag of corn bread"`。

## 离线包最简指令

首次安装：

```bash
cd /home/agi/graspv2-x2-ultra-offline
./install_offline.sh
```

只规划、不驱动机器人：

```bash
cd /home/agi/graspv2-x2-ultra-offline
./offline_run.sh --target-class cup --plan-only
```

确认急停、人员隔离、标定和右夹爪后执行：

```bash
cd /home/agi/graspv2-x2-ultra-offline
./offline_run.sh --target-class cup \
  --camera-calibration /home/agi/x2_camera_calibration.json \
  --execute --confirm-calibrated
```
