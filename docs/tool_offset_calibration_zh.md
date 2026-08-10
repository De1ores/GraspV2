# OmniPicker tool/TCP offset 标定说明

## 1. 配置文件和生效范围

唯一需要修改的文件是：

```text
config/tool_pose_offset.json
```

比赛机只有右手安装 OmniPicker，因此只标定 `right`。`left` 项是官方双臂运动学模型要求的
兼容占位，必须保留全零，抓取规划和真机夹爪命令不会使用它：

```json
{
  "left": {
    "parent_frame": "L_omnipicker_base_link",
    "translation_m": [0.0, 0.0, 0.0],
    "rpy_rad": [0.0, 0.0, 0.0]
  },
  "right": {
    "parent_frame": "R_omnipicker_base_link",
    "translation_m": [0.0, 0.0, 0.0],
    "rpy_rad": [0.0, 0.0, 0.0]
  }
}
```

这个变换定义实际抓取中心 TCP 相对于 OmniPicker 安装基座的固定位姿：

```text
T_world_tcp = T_world_omnipicker_base × T_omnipicker_base_tcp
```

同一份配置同时用于：

- Pinocchio IK 的目标 task frame；
- SDK FK 返回的末端位置和姿态；
- MuJoCo 中的红色 TCP 标记；
- SDK/MuJoCo FK 一致性检查；
- `output/planning_report.json` 中的 `active_tcp_offset` 记录。

修改 JSON 后重新启动 `run.sh`，不需要重新编译。

## 2. translation_m 对应什么

`translation_m = [dx, dy, dz]` 是从 `*_omnipicker_base_link` 原点指向实际抓取中心
TCP 的向量，单位是米，并且是在该 OmniPicker 基座的局部坐标系中表达，不是机器人
世界坐标。

```text
p_world_tcp = p_world_base + R_world_base × [dx, dy, dz]
```

- `dx > 0`：TCP 沿 OmniPicker 基座局部 `+X` 移动；
- `dy > 0`：TCP 沿局部 `+Y` 移动；
- `dz > 0`：TCP 沿局部 `+Z` 移动；
- 负值表示相反方向。

例如，实测右手抓取中心位于基座局部 `+Z` 方向 65 mm、局部 `+X` 方向 8 mm：

```json
"translation_m": [0.008, 0.0, 0.065]
```

规划目标仍然是视觉给出的物体抓取点。增加 TCP 平移后，IK 会自动反算 OmniPicker
基座应该移动到哪里，使配置后的真实 TCP 到达同一个物体点；不要再手工移动视觉目标。

右侧 OmniPicker 基座在官方 URDF 中带固定旋转，因此必须在它自己的局部坐标系中测量，
不能直接把世界坐标位移填入配置。

## 3. rpy_rad 对应什么

`rpy_rad = [roll, pitch, yaw]` 定义 TCP 坐标轴相对 OmniPicker 基座坐标轴的固定旋转：

- `roll`：绕局部 X 轴；
- `pitch`：绕局部 Y 轴；
- `yaw`：绕局部 Z 轴；
- 单位为弧度，旋转矩阵约定为 `Rz(yaw) × Ry(pitch) × Rx(roll)`。

常用角度换算：

| 角度 | 弧度 |
|---:|---:|
| 5° | 0.087266 |
| 10° | 0.174533 |
| 45° | 0.785398 |
| 90° | 1.570796 |
| 180° | 3.141593 |

当前 graspV2 抓取规划使用位置 IK，`rpy_rad` 会正确改变 TCP/FK/MuJoCo 标记的方向，
但不会单独强制机械臂达到某个抓取朝向。真正启用 6D 姿态抓取时，这一固定旋转可直接
作为 TCP 姿态变换继续使用。TCP 原点的位置主要由 `translation_m` 决定。

## 4. 推荐标定流程

1. 将右侧 `translation_m` 和 `rpy_rad` 暂时设为全零并保存原文件；左侧占位项保持全零。
2. 在 CAD 中读取，或使用直尺/标定尖点测量 OmniPicker 基座原点到实际夹持中心的
   局部 XYZ 距离。
3. 只修改 `right`，先以 1～5 mm 的增量验证；不要同时修改相机外参。
4. 运行仿真：

   ```bash
   cd ~/graspV2
   ./run.sh --no-vision
   ```

5. 在 MuJoCo 中观察红色 TCP 标记是否位于实际夹持中心，并检查：

   ```text
   output/planning_report.json
   ```

   其中 `active_tcp_offset` 必须与刚填写的值一致。
6. 最后使用一个已知世界坐标的标定点做右手实机低速验证。

若观察到整个桌子、物体和抓取点一起偏移，应调整
`config/mujoco_camera_calibration.json` 的相机外参/全局点偏移；若只有夹爪实际抓取中心
相对规划点有固定误差，才调整本文件的 tool offset。两者不能混用。

## 5. 安全限制和恢复默认

- 每个平移分量的配置保护上限为 0.50 m；
- 每个旋转分量的保护上限为 `2π`；
- JSON 必须包含三个有限数值；
- `right.parent_frame` 必须保持为 `R_omnipicker_base_link`；左侧兼容占位仍保留
  `L_omnipicker_base_link` 和全零偏移。

恢复默认行为：

```json
"translation_m": [0.0, 0.0, 0.0],
"rpy_rad": [0.0, 0.0, 0.0]
```

## 6. 待抓物圆柱的碰撞说明

视觉识别生成的 bottle 圆柱目前是显示模型，不是碰撞障碍：

```xml
contype="0" conaffinity="0" density="0"
```

桌子、机器人手臂代理和 OmniPicker 仍参与碰撞检查。目标物保持显示-only 是有意设计，
因为现有碰撞门会拒绝所有非白名单接触；若直接给圆柱打开碰撞，末端到达抓取点时会被
判为碰撞，导致抓取规划失败。若以后需要真实物体接触，应一起实现：目标物碰撞几何、
仅允许所选 OmniPicker 接触目标物的白名单、接触阶段切换，以及桌面/非目标物仍然禁止
接触的规则。
