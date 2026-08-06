from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig


def test_right_arm_offset_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current = solver.fk_xyz(ArmSide.RIGHT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    result = solver.solve_position(ArmSide.RIGHT, target, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert len(result.arm_pos) == 14
    assert solver.arm_dof == 7


def test_left_arm_offset_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current = solver.fk_xyz(ArmSide.LEFT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    result = solver.solve_position(ArmSide.LEFT, target, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert len(result.arm_pos) == 14


def test_right_arm_pose_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    target_arm = list(seed)
    # Nudge right elbow so the pose target is reachable from ready seed.
    target_arm[10] += 0.05
    target_xyz = solver.fk_xyz(ArmSide.RIGHT, target_arm)
    target_rpy = solver.fk_rpy(ArmSide.RIGHT, target_arm)

    result = solver.solve_pose(ArmSide.RIGHT, target_xyz, target_rpy, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert result.target_rpy is not None
    assert result.final_rpy is not None
    assert abs(result.final_xyz[0] - target_xyz[0]) < 2e-4
    assert abs(result.final_rpy[0] - target_rpy[0]) < 2e-3


def test_left_arm_pose_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    target_arm = list(seed)
    target_arm[3] += 0.05
    target_xyz = solver.fk_xyz(ArmSide.LEFT, target_arm)
    target_rpy = solver.fk_rpy(ArmSide.LEFT, target_arm)

    result = solver.solve_pose(ArmSide.LEFT, target_xyz, target_rpy, seed)
    assert result.success
    assert result.error_norm < 2e-4


def test_5dof_position_and_pose_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker_5dof())
    assert solver.arm_dof == 5
    assert len(solver.arm_pos_order) == 10
    assert "left_wrist_pitch_joint" not in solver.arm_pos_order
    assert "right_wrist_roll_joint" not in solver.arm_pos_order

    seed = solver.ready_arm_pos()
    assert len(seed) == 10

    current = solver.fk_xyz(ArmSide.RIGHT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    pos_result = solver.solve_position(ArmSide.RIGHT, target, seed)
    assert pos_result.success
    assert len(pos_result.arm_pos) == 10
    assert len(pos_result.active_arm) == 5

    target_arm = list(seed)
    target_arm[8] += 0.05  # right elbow in 5-DoF packing: L5 + R[0..4], elbow at index 8
    target_xyz = solver.fk_xyz(ArmSide.RIGHT, target_arm)
    target_rpy = solver.fk_rpy(ArmSide.RIGHT, target_arm)
    pose_result = solver.solve_pose(ArmSide.RIGHT, target_xyz, target_rpy, seed)
    assert pose_result.success
    assert pose_result.error_norm < 2e-4
