# graspV2 现场速查


cd /home/GraspV2

# 本地安装 YOLO、IK、MuJoCo 等重计算环境
./tools/install_local_split.sh

# 部署约 172 KB 的轻量机器人端代码
./tools/deploy_robot_edge.sh --robot-host <机器人IP>

# 真机执行
./run_split_grasp_pipeline.sh \
  --robot-host <机器人IP> \
  --target-class cup \
  --camera-calibration /path/to/x2_camera_calibration.json \
  --execute --confirm-calibrated

"orange-capped pill"
"bag of corn bread"

