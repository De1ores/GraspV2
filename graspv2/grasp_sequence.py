"""Pure validation contracts for a visually verified grasp sequence."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from .trajectory import JointTrajectory, TrajectoryValidationError


MINIMUM_PREMOTION_OPEN_DURATION_S = 3.0


class VisualVerificationError(RuntimeError):
    """Raised when vision cannot prove that the object followed the gripper."""


@dataclass(frozen=True)
class VisualObservation:
    """One selected object-center observation in the planning world frame."""

    source: Path
    class_name: str
    confidence: float
    object_center_world_m: tuple[float, float, float]
    surface_point_world_m: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class VisualVerificationConfig:
    """Conservative geometric gates used at the two visual checkpoints."""

    close_target_tolerance_m: float = 0.08
    lifted_target_tolerance_m: float = 0.10
    minimum_lift_ratio: float = 0.60
    maximum_lateral_drift_m: float = 0.08

    def validate(self) -> "VisualVerificationConfig":
        for name in (
            "close_target_tolerance_m",
            "lifted_target_tolerance_m",
            "maximum_lateral_drift_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(self.minimum_lift_ratio) or not (
            0.0 < self.minimum_lift_ratio <= 1.0
        ):
            raise ValueError("minimum_lift_ratio must be within (0, 1]")
        return self


@dataclass(frozen=True)
class GraspPlanMetadata:
    """Cross-file metadata needed by the live grasp executor."""

    robot_profile: str
    side: str
    object_center_world_m: tuple[float, float, float]
    gripper_center_world_m: tuple[float, float, float]
    lifted_object_center_world_m: tuple[float, float, float]
    lifted_gripper_center_world_m: tuple[float, float, float]
    lift_direction_world: tuple[float, float, float]
    lift_height_m: float
    lift_duration_s: float
    pregrasp_duration_s: float
    visual_radius_m: float
    preopen_position: float
    grip_position: float
    open_duration_s: float
    close_duration_s: float
    grasp_settle_duration_s: float
    lifted_hold_duration_s: float
    controlled_lower_duration_s: float
    open_hand_retreat_duration_s: float
    release_duration_s: float
    place_settle_duration_s: float
    reclose_duration_s: float


@dataclass(frozen=True)
class VisualCheckResult:
    """Serializable measurements from one successful vision gate."""

    stage: str
    passed: bool
    observed_world_m: tuple[float, float, float]
    expected_world_m: tuple[float, float, float]
    target_error_m: float
    lift_displacement_m: float | None = None
    lateral_drift_m: float | None = None


def _finite_vector(value: object, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise VisualVerificationError(f"{label} must contain three numbers")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise VisualVerificationError(f"{label} must contain three numbers")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise VisualVerificationError(f"{label} contains a non-finite value")
    return result


def _read_object(path: Path, label: str) -> dict[str, object]:
    source = path.expanduser().resolve()
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise VisualVerificationError(f"{label} does not exist: {source}") from error
    except (OSError, json.JSONDecodeError) as error:
        raise VisualVerificationError(f"cannot read {label}: {error}") from error
    if not isinstance(document, dict):
        raise VisualVerificationError(f"{label} root must be an object")
    return document


def load_visual_observation(
    path: Path,
    requested_class: str,
) -> VisualObservation:
    """Load the selected detection and reject class/frame contract mismatches."""

    if not requested_class.strip():
        raise ValueError("requested_class must be non-empty")
    document = _read_object(path, "vision result")
    selected = document.get("selected_detection")
    if not isinstance(selected, dict):
        raise VisualVerificationError("vision result has no selected_detection")
    class_name = selected.get("class_name")
    if not isinstance(class_name, str) or (
        class_name.casefold() != requested_class.casefold()
    ):
        raise VisualVerificationError(
            f"requested {requested_class!r}, but vision selected {class_name!r}"
        )
    confidence = selected.get("confidence")
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise VisualVerificationError("selected detection confidence is invalid")
    confidence = float(confidence)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise VisualVerificationError("selected detection confidence is outside [0, 1]")
    raw_object_center = document.get("object_center_mujoco_m")
    legacy_surface = document.get("grasp_point_mujoco_m")
    if raw_object_center is None:
        if legacy_surface is not None:
            raise VisualVerificationError(
                "schema-v1 vision result contains only a legacy surface point; "
                "rerun vision to obtain object_center_mujoco_m for verification"
            )
        raise VisualVerificationError(
            "vision result has no object_center_mujoco_m"
        )
    object_center = _finite_vector(
        raw_object_center,
        "object_center_mujoco_m",
    )
    raw_surface = document.get("surface_point_mujoco_m", legacy_surface)
    surface_point = (
        _finite_vector(raw_surface, "surface_point_mujoco_m")
        if raw_surface is not None
        else None
    )
    return VisualObservation(
        source=path.expanduser().resolve(),
        class_name=class_name,
        confidence=confidence,
        object_center_world_m=object_center,
        surface_point_world_m=surface_point,
    )


def _planning_object(document: dict[str, object], label: str) -> dict[str, object]:
    planning = document.get("planning")
    if not isinstance(planning, dict):
        raise TrajectoryValidationError(f"{label} has no planning metadata")
    if planning.get("verified_collision_free") is not True:
        raise TrajectoryValidationError(f"{label} is not verified collision-free")
    if planning.get("ik_backend") != "x2_ik_sdk.X2ArmIKSolver":
        raise TrajectoryValidationError(f"{label} was not generated by official IK")
    return planning


def load_grasp_plan_metadata(
    approach_path: Path,
    lift_path: Path,
    return_path: Path,
) -> GraspPlanMetadata:
    """Validate that approach, lift and return JSON files form one grasp plan."""

    approach = _read_object(approach_path, "approach trajectory")
    lift = _read_object(lift_path, "lift trajectory")
    return_to_default = _read_object(return_path, "return trajectory")
    approach_planning = _planning_object(approach, "approach trajectory")
    lift_planning = _planning_object(lift, "lift trajectory")
    return_planning = _planning_object(return_to_default, "return trajectory")
    profile = approach.get("robot_profile")
    side = approach.get("arm_side")
    if not isinstance(profile, str) or not profile:
        raise TrajectoryValidationError("approach robot_profile is invalid")
    if side not in {"left", "right"}:
        raise TrajectoryValidationError("approach arm_side is invalid")
    if lift.get("robot_profile") != profile or lift.get("arm_side") != side:
        raise TrajectoryValidationError(
            "approach and lift trajectories use different robot/arm profiles"
        )
    if (
        return_to_default.get("robot_profile") != profile
        or return_to_default.get("arm_side") != side
    ):
        raise TrajectoryValidationError(
            "approach and return trajectories use different robot/arm profiles"
        )
    if lift_planning.get("trajectory_role") != "lift":
        raise TrajectoryValidationError("lift trajectory has no lift role metadata")
    if return_planning.get("trajectory_role") != "return_to_default":
        raise TrajectoryValidationError(
            "return trajectory has no return_to_default role metadata"
        )
    legacy_gripper_target = _finite_vector(
        approach_planning.get("target_world_m"),
        "approach planning.target_world_m",
    )
    gripper_center = _finite_vector(
        approach_planning.get(
            "gripper_center_world_m", list(legacy_gripper_target)
        ),
        "approach planning.gripper_center_world_m",
    )
    object_center = _finite_vector(
        approach_planning.get(
            "object_center_world_m", list(gripper_center)
        ),
        "approach planning.object_center_world_m",
    )
    lift_start_gripper = _finite_vector(
        lift_planning.get(
            "lift_start_gripper_center_world_m",
            lift_planning.get("lift_start_world_m"),
        ),
        "lift planning.lift_start_gripper_center_world_m",
    )
    if np.linalg.norm(
        np.asarray(gripper_center) - np.asarray(legacy_gripper_target)
    ) > 1e-6:
        raise TrajectoryValidationError(
            "approach gripper center does not match its IK target"
        )
    if np.linalg.norm(
        np.asarray(gripper_center) - np.asarray(lift_start_gripper)
    ) > 1e-6:
        raise TrajectoryValidationError(
            "lift gripper-center start does not match the approach target"
        )
    lifted_gripper_center = _finite_vector(
        lift_planning.get(
            "gripper_center_world_m", lift_planning.get("target_world_m")
        ),
        "lift planning.gripper_center_world_m",
    )
    direction = np.asarray(
        _finite_vector(
            lift_planning.get("lift_direction_world"),
            "lift planning.lift_direction_world",
        ),
        dtype=float,
    )
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1e-9:
        raise TrajectoryValidationError("lift direction must be non-zero")
    direction /= direction_norm
    height = lift_planning.get("lift_height_m")
    duration = lift_planning.get("lift_duration_s")
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in (height, duration)
    ):
        raise TrajectoryValidationError("lift height/duration metadata is invalid")
    height = float(height)
    duration = float(duration)
    if not math.isfinite(height) or height <= 0.0:
        raise TrajectoryValidationError("lift height must be positive")
    if not math.isfinite(duration) or duration <= 0.0:
        raise TrajectoryValidationError("lift duration must be positive")
    lift_start_object = _finite_vector(
        lift_planning.get(
            "lift_start_object_center_world_m", list(object_center)
        ),
        "lift planning.lift_start_object_center_world_m",
    )
    lifted_object_center = _finite_vector(
        lift_planning.get(
            "object_center_world_m",
            list(np.asarray(object_center) + direction * height),
        ),
        "lift planning.object_center_world_m",
    )
    if np.linalg.norm(
        np.asarray(object_center) - np.asarray(lift_start_object)
    ) > 1e-6:
        raise TrajectoryValidationError(
            "lift object-center start does not match the approach observation"
        )
    sequence = approach_planning.get("simulated_grasp_sequence")
    if not isinstance(sequence, dict):
        raise TrajectoryValidationError(
            "approach trajectory has no verified grasp sequence metadata"
        )
    if approach_planning.get("gripper_fully_open_before_arm_motion") is not True:
        raise TrajectoryValidationError(
            "approach trajectory was not collision-checked with the gripper "
            "fully open before arm motion"
        )
    if sequence.get("gripper_fully_open_before_arm_motion") is not True:
        raise TrajectoryValidationError(
            "grasp sequence does not open the gripper before arm motion"
        )

    def sequence_number(name: str, *, positive: bool = True) -> float:
        value = sequence.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TrajectoryValidationError(
                f"grasp sequence {name} must be a number"
            )
        result = float(value)
        if not math.isfinite(result) or (positive and result <= 0.0):
            qualifier = "positive and finite" if positive else "finite"
            raise TrajectoryValidationError(
                f"grasp sequence {name} must be {qualifier}"
            )
        return result

    pregrasp_duration = sequence_number("vertical_descent_start_time_s")
    approach_boundary = approach_planning.get("vertical_descent_start_time_s")
    if isinstance(approach_boundary, bool) or not isinstance(
        approach_boundary, (int, float)
    ):
        raise TrajectoryValidationError(
            "approach vertical descent boundary is invalid"
        )
    if abs(float(approach_boundary) - pregrasp_duration) > 1e-6:
        raise TrajectoryValidationError(
            "grasp sequence descent boundary differs from approach metadata"
        )
    visual_radius = sequence_number("visual_radius_m")
    preopen = sequence_number("preopen_position", positive=False)
    grip = sequence_number("grip_position", positive=False)
    if not (0.0 <= grip <= preopen <= 1.0):
        raise TrajectoryValidationError(
            "grasp sequence positions must satisfy 0 <= grip <= preopen <= 1"
        )
    phase_durations = {
        name: sequence_number(name)
        for name in (
            "open_duration_s",
            "close_duration_s",
            "grasp_settle_duration_s",
            "lifted_hold_duration_s",
            "controlled_lower_duration_s",
            "open_hand_retreat_duration_s",
            "release_duration_s",
            "place_settle_duration_s",
            "reclose_duration_s",
        )
    }
    if phase_durations["open_duration_s"] < MINIMUM_PREMOTION_OPEN_DURATION_S:
        raise TrajectoryValidationError(
            "pre-motion gripper-open duration is shorter than "
            f"{MINIMUM_PREMOTION_OPEN_DURATION_S:.1f} s"
        )
    return_mode = return_planning.get("return_mode")
    if return_mode != "controlled_lower_then_reverse_approach":
        raise TrajectoryValidationError(
            "return trajectory is not a controlled lower/place/retreat path"
        )
    return_lower_duration = return_planning.get("controlled_lower_duration_s")
    return_open_retreat_duration = return_planning.get(
        "open_hand_retreat_duration_s"
    )
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in (return_lower_duration, return_open_retreat_duration)
    ):
        raise TrajectoryValidationError(
            "return trajectory phase boundaries are invalid"
        )
    if abs(
        float(return_lower_duration)
        - phase_durations["controlled_lower_duration_s"]
    ) > 1e-6:
        raise TrajectoryValidationError(
            "return lower duration differs from grasp sequence metadata"
        )
    if abs(
        float(return_open_retreat_duration)
        - phase_durations["open_hand_retreat_duration_s"]
    ) > 1e-6:
        raise TrajectoryValidationError(
            "return open-retreat duration differs from grasp sequence metadata"
        )
    if abs(phase_durations["controlled_lower_duration_s"] - duration) > 1e-6:
        raise TrajectoryValidationError(
            "controlled lower duration must match the verified lift duration"
        )
    actual_lift_displacement = np.asarray(
        _finite_vector(
            lift_planning.get(
                "actual_lift_displacement_world_m",
                list(direction * height),
            ),
            "lift planning.actual_lift_displacement_world_m",
        ),
        dtype=float,
    )
    expected_gripper_center = (
        np.asarray(gripper_center) + actual_lift_displacement
    )
    if np.linalg.norm(
        expected_gripper_center - np.asarray(lifted_gripper_center)
    ) > 1e-5:
        raise TrajectoryValidationError(
            "lift gripper center does not match start + direction * height"
        )
    expected_object_center = np.asarray(object_center) + actual_lift_displacement
    if np.linalg.norm(
        expected_object_center - np.asarray(lifted_object_center)
    ) > 1e-5:
        raise TrajectoryValidationError(
            "lift object center does not match start + direction * height"
        )
    return GraspPlanMetadata(
        robot_profile=profile,
        side=side,
        object_center_world_m=object_center,
        gripper_center_world_m=gripper_center,
        lifted_object_center_world_m=lifted_object_center,
        lifted_gripper_center_world_m=lifted_gripper_center,
        lift_direction_world=tuple(float(value) for value in direction),
        lift_height_m=height,
        lift_duration_s=duration,
        pregrasp_duration_s=pregrasp_duration,
        visual_radius_m=visual_radius,
        preopen_position=preopen,
        grip_position=grip,
        open_duration_s=phase_durations["open_duration_s"],
        close_duration_s=phase_durations["close_duration_s"],
        grasp_settle_duration_s=phase_durations["grasp_settle_duration_s"],
        lifted_hold_duration_s=phase_durations["lifted_hold_duration_s"],
        controlled_lower_duration_s=phase_durations[
            "controlled_lower_duration_s"
        ],
        open_hand_retreat_duration_s=phase_durations[
            "open_hand_retreat_duration_s"
        ],
        release_duration_s=phase_durations["release_duration_s"],
        place_settle_duration_s=phase_durations["place_settle_duration_s"],
        reclose_duration_s=phase_durations["reclose_duration_s"],
    )


def validate_trajectory_continuity(
    approach: JointTrajectory,
    lift: JointTrajectory,
    *,
    tolerance_rad: float = 1e-6,
) -> None:
    """Require the lift to start exactly where the approach finishes."""

    if approach.joint_names != lift.joint_names:
        raise TrajectoryValidationError(
            "approach and lift active joint names/order differ"
        )
    error = max(
        abs(end - start)
        for end, start in zip(approach.positions[-1], lift.positions[0])
    )
    if error > tolerance_rad:
        raise TrajectoryValidationError(
            f"approach/lift discontinuity is {error:.6f} rad; "
            f"limit is {tolerance_rad:.6f} rad"
        )


def verify_closed_observation(
    observation: VisualObservation,
    expected_object_center_world_m: Iterable[float],
    config: VisualVerificationConfig,
) -> VisualCheckResult:
    """Confirm that the requested object's center remains at the grasp region."""

    settings = config.validate()
    observed = np.asarray(observation.object_center_world_m, dtype=float)
    expected = np.asarray(tuple(expected_object_center_world_m), dtype=float)
    if expected.shape != (3,) or not np.all(np.isfinite(expected)):
        raise ValueError(
            "expected_object_center_world_m must contain three finite values"
        )
    error = float(np.linalg.norm(observed - expected))
    if error > settings.close_target_tolerance_m:
        raise VisualVerificationError(
            "closed-grasp vision gate failed: object is "
            f"{error:.3f} m from the planned grasp region; limit is "
            f"{settings.close_target_tolerance_m:.3f} m"
        )
    return VisualCheckResult(
        stage="closed_grasp",
        passed=True,
        observed_world_m=observation.object_center_world_m,
        expected_world_m=tuple(float(value) for value in expected),
        target_error_m=error,
    )


def verify_initial_observation(
    observation: VisualObservation,
    expected_object_center_world_m: Iterable[float],
    config: VisualVerificationConfig,
) -> VisualCheckResult:
    """Reject a stale or unrelated initial vision file before live execution."""

    settings = config.validate()
    observed = np.asarray(observation.object_center_world_m, dtype=float)
    expected = np.asarray(tuple(expected_object_center_world_m), dtype=float)
    if expected.shape != (3,) or not np.all(np.isfinite(expected)):
        raise ValueError(
            "expected_object_center_world_m must contain three finite values"
        )
    error = float(np.linalg.norm(observed - expected))
    if error > settings.close_target_tolerance_m:
        raise VisualVerificationError(
            "initial vision does not match the planned grasp target: "
            f"error={error:.3f} m, limit={settings.close_target_tolerance_m:.3f} m"
        )
    return VisualCheckResult(
        stage="initial_plan_alignment",
        passed=True,
        observed_world_m=observation.object_center_world_m,
        expected_world_m=tuple(float(value) for value in expected),
        target_error_m=error,
    )


def verify_lifted_observation(
    closed_observation: VisualObservation,
    lifted_observation: VisualObservation,
    metadata: GraspPlanMetadata,
    config: VisualVerificationConfig,
) -> VisualCheckResult:
    """Prove that the object moved with the gripper through the two-second lift."""

    settings = config.validate()
    closed = np.asarray(closed_observation.object_center_world_m, dtype=float)
    lifted = np.asarray(lifted_observation.object_center_world_m, dtype=float)
    expected = np.asarray(metadata.lifted_object_center_world_m, dtype=float)
    direction = np.asarray(metadata.lift_direction_world, dtype=float)
    delta = lifted - closed
    lift_displacement = float(np.dot(delta, direction))
    lateral = float(np.linalg.norm(delta - lift_displacement * direction))
    target_error = float(np.linalg.norm(lifted - expected))
    minimum_displacement = metadata.lift_height_m * settings.minimum_lift_ratio
    failures = []
    if lift_displacement < minimum_displacement:
        failures.append(
            f"lift displacement {lift_displacement:.3f} m is below "
            f"{minimum_displacement:.3f} m"
        )
    if lateral > settings.maximum_lateral_drift_m:
        failures.append(
            f"lateral drift {lateral:.3f} m exceeds "
            f"{settings.maximum_lateral_drift_m:.3f} m"
        )
    if target_error > settings.lifted_target_tolerance_m:
        failures.append(
            f"lifted target error {target_error:.3f} m exceeds "
            f"{settings.lifted_target_tolerance_m:.3f} m"
        )
    if failures:
        raise VisualVerificationError(
            "post-lift vision gate failed (object dropped or did not follow gripper): "
            + "; ".join(failures)
        )
    return VisualCheckResult(
        stage="post_lift",
        passed=True,
        observed_world_m=lifted_observation.object_center_world_m,
        expected_world_m=metadata.lifted_object_center_world_m,
        target_error_m=target_error,
        lift_displacement_m=lift_displacement,
        lateral_drift_m=lateral,
    )


def write_grasp_status(
    path: Path,
    *,
    state: str,
    metadata: GraspPlanMetadata,
    checks: Iterable[VisualCheckResult] = (),
    error: str | None = None,
) -> Path:
    """Atomically persist the last state for operators and automated judging."""

    destination = path.expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "schema_version": 1,
        "state": state,
        "success": state == "complete",
        "plan": asdict(metadata),
        "visual_checks": [asdict(check) for check in checks],
        "error": error,
    }
    temporary = destination.with_name(destination.name + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    return destination
