"""Pure validation contracts for a visually verified grasp sequence."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from .trajectory import JointTrajectory, TrajectoryValidationError


class VisualVerificationError(RuntimeError):
    """Raised when vision cannot prove that the object followed the gripper."""


@dataclass(frozen=True)
class VisualObservation:
    """One selected object observation expressed in the planning world frame."""

    source: Path
    class_name: str
    confidence: float
    point_world_m: tuple[float, float, float]


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
    grasp_target_world_m: tuple[float, float, float]
    lifted_target_world_m: tuple[float, float, float]
    lift_direction_world: tuple[float, float, float]
    lift_height_m: float
    lift_duration_s: float


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
    point = _finite_vector(
        document.get("grasp_point_mujoco_m"),
        "grasp_point_mujoco_m",
    )
    return VisualObservation(
        source=path.expanduser().resolve(),
        class_name=class_name,
        confidence=confidence,
        point_world_m=point,
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
) -> GraspPlanMetadata:
    """Validate that approach and lift JSON files form one grasp plan."""

    approach = _read_object(approach_path, "approach trajectory")
    lift = _read_object(lift_path, "lift trajectory")
    approach_planning = _planning_object(approach, "approach trajectory")
    lift_planning = _planning_object(lift, "lift trajectory")
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
    if lift_planning.get("trajectory_role") != "lift":
        raise TrajectoryValidationError("lift trajectory has no lift role metadata")
    grasp_target = _finite_vector(
        approach_planning.get("target_world_m"),
        "approach planning.target_world_m",
    )
    lift_start = _finite_vector(
        lift_planning.get("lift_start_world_m"),
        "lift planning.lift_start_world_m",
    )
    if np.linalg.norm(np.asarray(grasp_target) - np.asarray(lift_start)) > 1e-6:
        raise TrajectoryValidationError(
            "lift world-frame start does not match the approach target"
        )
    lifted_target = _finite_vector(
        lift_planning.get("target_world_m"),
        "lift planning.target_world_m",
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
    expected_target = np.asarray(grasp_target) + direction * height
    if np.linalg.norm(expected_target - np.asarray(lifted_target)) > 1e-5:
        raise TrajectoryValidationError(
            "lift target does not match start + direction * height"
        )
    return GraspPlanMetadata(
        robot_profile=profile,
        side=side,
        grasp_target_world_m=grasp_target,
        lifted_target_world_m=lifted_target,
        lift_direction_world=tuple(float(value) for value in direction),
        lift_height_m=height,
        lift_duration_s=duration,
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
    expected_grasp_world_m: Iterable[float],
    config: VisualVerificationConfig,
) -> VisualCheckResult:
    """Confirm that the requested object remains inside the closed grasp region."""

    settings = config.validate()
    observed = np.asarray(observation.point_world_m, dtype=float)
    expected = np.asarray(tuple(expected_grasp_world_m), dtype=float)
    if expected.shape != (3,) or not np.all(np.isfinite(expected)):
        raise ValueError("expected_grasp_world_m must contain three finite values")
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
        observed_world_m=observation.point_world_m,
        expected_world_m=tuple(float(value) for value in expected),
        target_error_m=error,
    )


def verify_initial_observation(
    observation: VisualObservation,
    expected_grasp_world_m: Iterable[float],
    config: VisualVerificationConfig,
) -> VisualCheckResult:
    """Reject a stale or unrelated initial vision file before live execution."""

    settings = config.validate()
    observed = np.asarray(observation.point_world_m, dtype=float)
    expected = np.asarray(tuple(expected_grasp_world_m), dtype=float)
    if expected.shape != (3,) or not np.all(np.isfinite(expected)):
        raise ValueError("expected_grasp_world_m must contain three finite values")
    error = float(np.linalg.norm(observed - expected))
    if error > settings.close_target_tolerance_m:
        raise VisualVerificationError(
            "initial vision does not match the planned grasp target: "
            f"error={error:.3f} m, limit={settings.close_target_tolerance_m:.3f} m"
        )
    return VisualCheckResult(
        stage="initial_plan_alignment",
        passed=True,
        observed_world_m=observation.point_world_m,
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
    closed = np.asarray(closed_observation.point_world_m, dtype=float)
    lifted = np.asarray(lifted_observation.point_world_m, dtype=float)
    expected = np.asarray(metadata.lifted_target_world_m, dtype=float)
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
        observed_world_m=lifted_observation.point_world_m,
        expected_world_m=metadata.lifted_target_world_m,
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
