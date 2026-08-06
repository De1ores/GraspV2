from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig


def main():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    arm_pos = solver.ready_arm_pos()

    side = ArmSide.RIGHT
    current_xyz = solver.fk_xyz(side, arm_pos)
    current_rpy = solver.fk_rpy(side, arm_pos)
    target_xyz = [current_xyz[0] + 0.01, current_xyz[1], current_xyz[2] + 0.01]
    target_rpy = current_rpy

    result = solver.solve_pose(side, target_xyz, target_rpy, arm_pos)
    print("dof:", solver.arm_dof)
    print("success:", result.success)
    print("error_norm:", result.error_norm)
    print("final_xyz:", result.final_xyz)
    print("final_rpy:", result.final_rpy)
    print("arm_pos:", result.arm_pos)


if __name__ == "__main__":
    main()
