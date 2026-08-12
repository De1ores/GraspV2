# x2_ik_sdk

基于 Pinocchio 的 X2 双臂离线运动学 Python SDK。

## 已实现功能

- 左臂和右臂末端位置 IK；
- 左臂和右臂末端姿态 IK（同时约束 `xyz` 与 `rpy`）；
- 末端位置加单轴方向 IK（约束 `xyz` 和工具轴，轴向旋转保持自由）；
- 末端位置 `xyz` 和姿态 `rpy` 的正运动学；
- `arm_pos` 与 Pinocchio configuration 的相互转换；
- 兼容单臂 7 自由度与 5 自由度（无 `wrist_pitch` / `wrist_roll`）；
- ready pose、关节限位和迭代关节限位保护；
- `current_arm_pos`、`current_head_pos` 和 `q_seed` 初值；
- 内置 X2 omnipicker 纯运动学 URDF。

## 安装

```bash
python -m pip install --no-build-isolation -e .
```

依赖：

```text
numpy
pin
```

## Python API

```python
from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig

solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
arm_pos = solver.ready_arm_pos()

current_xyz = solver.fk_xyz(ArmSide.RIGHT, arm_pos)
current_rpy = solver.fk_rpy(ArmSide.RIGHT, arm_pos)
target_xyz = [current_xyz[0] + 0.01, current_xyz[1], current_xyz[2] + 0.01]
target_rpy = current_rpy

result = solver.solve_pose(
    side=ArmSide.RIGHT,
    target_xyz=target_xyz,
    target_rpy=target_rpy,
    current_arm_pos=arm_pos,
)

print(result.success)
print(result.error_norm)
print(result.arm_pos)
print(result.final_xyz, result.final_rpy)
```

仅约束位置时仍可用 `solve_position`。

只需要保持夹爪轴线垂直、但不希望额外锁死绕轴角度时，使用
`solve_position_axis(side, target_xyz, local_axis, target_axis, current_arm_pos)`。

### 5 自由度手臂

当 URDF 中不存在左右手的 `wrist_pitch` / `wrist_roll` 时，求解器会自动按 5
自由度解析关节。也可在 7 自由度模型上显式只启用前 5 个关节：

```python
solver = X2ArmIKSolver(X2IKConfig.default_omnipicker_5dof())
assert solver.arm_dof == 5
assert len(solver.ready_arm_pos()) == 10
```

也可以运行仓库内示例：

```bash
python examples/offline_demo.py
```

## 模型

默认 URDF：

```text
src/x2_ik_sdk/resources/x2_ultra_plus_omnipicker_omnipicker.urdf
```

`arm_pos` 顺序由求解器解析后的 `arm_pos_order` 决定：左臂关节在前，右臂关节在后。
7 自由度时长度为 14（见 `LEFT_ARM_JOINTS` / `RIGHT_ARM_JOINTS`）；5 自由度时长度为
10（见 `LEFT_ARM_JOINTS_5` / `RIGHT_ARM_JOINTS_5`，不含 `wrist_pitch` 与
`wrist_roll`）。
