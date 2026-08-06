# svt Orin 运行命令

项目目录：`/home/svt/graspV2`。

部署使用两个隔离环境：`.venv` 负责 Jetson CUDA/YOLOE，`.planning-venv`
负责官方 IK 和 MuJoCo；一键脚本会自动选择，不要手工混装两套 NumPy 依赖。

命令必须同时指定识别物体类型和机器人类型：

```bash
cd /home/svt/graspV2
./run_full_grasp_pipeline.sh --target-class bottle --robot youth --plan-only
```

- `--target-class` 是 YOLOE 开放词汇物体类别；带空格时使用引号，例如
  `--target-class "game controller"`。
- `--robot youth` 表示每臂 5 轴青春版；`--robot ultra` 表示每臂 7 轴版本。
- 默认根据识别点自动选手臂：`+Y` 使用左臂、`-Y` 使用右臂；可用
  `--side left|right` 手工覆盖。
- 不加 `--execute` 时不会连接机器人控制接口。

识别、规划并离线生成动作 CSV：

```bash
./run_full_grasp_pipeline.sh --target-class cup --robot ultra
```

确认相机外参、机器人稳定站立、吊架和急停均已就绪后，才可执行实机动作：

```bash
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --robot ultra \
  --execute \
  --confirm-calibrated
```

实机命令仍会要求输入 `RUN`；不要在首次测试时使用 `--yes`。
