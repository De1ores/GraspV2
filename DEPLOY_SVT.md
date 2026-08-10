# svt Orin 运行命令

## 完整离线包（推荐）

在联网的制包机上执行一次：

```bash
./tools/fetch_offline_dependencies.sh
./tools/build_offline_bundle.sh
```

将 `dist/graspv2-x2-ultra-offline.tar.gz` 和同名 `.sha256` 文件上传到
X2 Ultra。真机无需联网：

```bash
sha256sum -c graspv2-x2-ultra-offline.tar.gz.sha256
tar -xzf graspv2-x2-ultra-offline.tar.gz
cd graspv2-x2-ultra-offline
./offline_run.sh --target-class cup --plan-only
```

`offline_run.sh` 首次运行会自动创建规划/视觉环境、解包 Jetson CUDA 运行库，
并针对真机已有的 AimDK 编译 ROS 包。所有 pip 操作均使用 `--no-index`；不会
下载、更新或覆盖系统 ROS、AimDK 与 CUDA 驱动。

项目目录：`/home/svt/graspV2`。

部署使用两个隔离环境：`.venv` 负责 Jetson CUDA/YOLOE，`.planning-venv`
负责官方 IK 和 MuJoCo；一键脚本会自动选择，不要手工混装两套 NumPy 依赖。

命令只需指定识别物体类型；仿真与真机固定使用 Ultra 右臂：

```bash
cd /home/svt/graspV2
./run_full_grasp_pipeline.sh --target-class "orange-capped pill bottle" \
  --plan-only
```

- `--target-class` 是 YOLOE 开放词汇物体类别；带空格时使用引号，例如
  `--target-class "game controller"`。
- Ultra 每臂 7 轴，手臂反馈固定校验为 14 轴。
- 比赛机只有右手安装夹爪，因此抓取规划和真机执行固定使用右臂，不会回退到左臂。
- 不加 `--execute` 时不会连接机器人控制接口。

识别、规划并离线生成动作 CSV：

```bash
./run_full_grasp_pipeline.sh --target-class cup
```

确认相机外参、机器人稳定站立、吊架和急停均已就绪后，才可执行实机动作：

真机执行会强制现场重新采集 RGB-D；`--use-existing-vision` 和
`--capture-backend existing` 只允许离线检查。

```bash
./run_full_grasp_pipeline.sh \
  --target-class cup \
  --execute \
  --confirm-calibrated
```

实机命令仍会要求输入 `RUN`；不要在首次测试时使用 `--yes`。
