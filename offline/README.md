# GraspV2 X2 Ultra 离线包

该目录中的 wheelhouse 和 Jetson 运行库由制包机生成。目标机器人安装时设置
`PIP_NO_INDEX=1`，所有 Python 包均从归档内安装，不访问互联网。
YOLOE 所需的 `yoloe-26s-seg.pt` 和 `mobileclip2_b.ts` 也随归档提供；任一
模型缺失时视觉入口直接报错，不会在机器人上尝试下载。

上传并解压后，一条命令即可完成首次安装和规划：

```bash
./offline_run.sh --target-class cup --plan-only
```

确认相机外参、TCP、吊架和急停后，才允许真机执行：

```bash
./offline_run.sh --target-class cup --execute --confirm-calibrated
```

首次运行会创建 `.planning-venv`、`.vision-venv`、`.runtime`，并针对机器人
已经安装的 ROS Humble/AimDK 编译 `graspv2`。归档不会安装或覆盖系统 ROS、
AimDK、CUDA 驱动，也不会自动跳过真机执行前的 `RUN` 确认。
