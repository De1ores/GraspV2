"""MuJoCo model derived from the official IK and physical robot URDFs.

The official SDK URDF remains the kinematic source of truth.  When the physical
X2 URDF is available, its full-body visual meshes are attached to the matching
SDK link frames. Conservative primitive proxies cover the arm, while the
right-hand OmniPicker meshes provide both the visible gripper and its collision
geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import struct
from typing import Iterable
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .official_ik import OfficialIK
from .robot_profiles import INSTALLED_GRIPPER_SIDE, RobotProfile
from .tool_pose import ToolPoseConfig
from .vision_geometry import DEFAULT_SIDE_GRASP_HEIGHT_OFFSET_M


TABLE_GEOM_NAME = "planning_table"
TARGET_OBJECT_BODY_NAME = "recognized_target_body"
TARGET_OBJECT_GEOM_NAME = "recognized_target_object"
TARGET_GRIPPER_CENTER_SITE_NAME = "recognized_target_gripper_center"
LEFT_SITE_NAME = "left_omnipicker_tcp"
RIGHT_SITE_NAME = "right_omnipicker_tcp"
RIGHT_CLAW_JOINT_NAME = "right_claw_joint"
RIGHT_WIDE_JOINT_NAME = "R_hand_wide1_joint"
OMNIPICKER_MAX_GRASP_DIAMETER_M = 0.12
COLLISION_CLEARANCE_NUMERICAL_TOLERANCE_M = 1e-4
SIDE_GRASP_HEIGHT_OFFSET_M = DEFAULT_SIDE_GRASP_HEIGHT_OFFSET_M
ROBOT_URDF_ENV = "GRASPV2_ROBOT_URDF"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OMNIPICKER_DESCRIPTION = (
    PROJECT_ROOT / "robot_description" / "urdf" / "robot_urdf.xacro"
)
OMNIPICKER_REPLACED_VISUAL_LINKS = {
    "left": ("left_wrist_roll_link",),
    "right": ("right_wrist_roll_link",),
}[INSTALLED_GRIPPER_SIDE]


def _numbers(text: str | None, length: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if text is None:
        return default
    values = tuple(float(value) for value in text.split())
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ValueError(f"expected {length} finite numbers, received {text!r}")
    return values


def _format(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _rpy_quaternion(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _rpy_matrix(rpy: Iterable[float]) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


@dataclass(frozen=True)
class TargetObjectVisual:
    class_name: str
    object_center_m: tuple[float, float, float]
    gripper_center_m: tuple[float, float, float]
    object_height_m: float
    quaternion_wxyz: tuple[float, float, float, float]
    geom_type: str
    size_m: tuple[float, ...]
    rgba: tuple[float, float, float, float]
    visual_radius_m: float


@dataclass(frozen=True)
class TableObstacle:
    center_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    half_extents_m: tuple[float, float, float]
    plane_normal: tuple[float, float, float]
    target_object: TargetObjectVisual | None = None


@dataclass(frozen=True)
class CollisionReport:
    valid: bool
    minimum_table_distance_m: float
    nearest_table_body: str | None
    self_contacts: tuple[str, ...]


@dataclass(frozen=True)
class FKAlignment:
    profile: str
    samples: int
    maximum_position_error_m: float
    maximum_orientation_error_deg: float


def _finite_vector(raw: object, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(raw, list) or len(raw) != length:
        raise ValueError(f"{label} must contain {length} values")
    values = tuple(float(value) for value in raw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain finite values")
    return values


def _target_object_shape(
    class_name: str,
) -> tuple[str, tuple[float, ...], float, tuple[float, float, float, float]]:
    """Return a stable display primitive and its height above the table."""
    normalized = class_name.casefold().strip()
    if "bottle" in normalized:
        return "cylinder", (0.035, 0.10), 0.10, (0.10, 0.42, 0.95, 1.0)
    if normalized == "can" or " can" in normalized:
        return "cylinder", (0.035, 0.06), 0.06, (0.88, 0.24, 0.12, 1.0)
    if normalized in ("cup", "mug") or " cup" in normalized or " mug" in normalized:
        return "cylinder", (0.045, 0.055), 0.055, (0.95, 0.62, 0.10, 1.0)
    if "bowl" in normalized:
        return "cylinder", (0.075, 0.03), 0.03, (0.30, 0.72, 0.34, 1.0)
    if "bag" in normalized or "corn bread" in normalized:
        return "box", (0.065, 0.035, 0.085), 0.085, (0.92, 0.72, 0.18, 1.0)
    if "keyboard" in normalized:
        return "box", (0.14, 0.055, 0.012), 0.012, (0.18, 0.20, 0.24, 1.0)
    if "remote" in normalized:
        return "box", (0.065, 0.025, 0.012), 0.012, (0.16, 0.18, 0.22, 1.0)
    if "game controller" in normalized:
        return "box", (0.075, 0.045, 0.02), 0.02, (0.22, 0.24, 0.30, 1.0)
    if "screwdriver" in normalized or normalized == "pen" or " pen" in normalized:
        return "box", (0.07, 0.012, 0.012), 0.012, (0.92, 0.25, 0.16, 1.0)
    if "box" in normalized:
        return "box", (0.055, 0.045, 0.04), 0.04, (0.62, 0.34, 0.16, 1.0)
    return "sphere", (0.045,), 0.045, (0.58, 0.30, 0.88, 1.0)


def _target_object_visual(
    document: dict[str, object],
    legacy_surface_point: tuple[float, float, float] | None,
    table_center: tuple[float, float, float],
    table_quaternion: tuple[float, float, float, float],
    table_half_extents: tuple[float, float, float],
    table_normal: tuple[float, float, float],
) -> TargetObjectVisual | None:
    selected = document.get("selected_detection")
    if not isinstance(selected, dict):
        return None
    raw_class_name = selected.get("class_name")
    if not isinstance(raw_class_name, str) or not raw_class_name.strip():
        return None
    class_name = raw_class_name.strip()
    geom_type, size, half_height, rgba = _target_object_shape(class_name)
    raw_object_height = selected.get("estimated_object_height_m")
    if raw_object_height is None:
        object_height = 2.0 * half_height
    elif isinstance(raw_object_height, bool) or not isinstance(
        raw_object_height, (int, float)
    ):
        raise ValueError("selected_detection.estimated_object_height_m must be a number")
    else:
        object_height = float(raw_object_height)
    if not math.isfinite(object_height) or object_height <= 0.0:
        raise ValueError("selected_detection.estimated_object_height_m must be positive")
    raw_visual_radius = selected.get("visual_radius_m")
    if raw_visual_radius is None:
        visual_radius = float(size[0])
    elif isinstance(raw_visual_radius, bool) or not isinstance(
        raw_visual_radius, (int, float)
    ):
        raise ValueError("selected_detection.visual_radius_m must be a number")
    else:
        visual_radius = float(raw_visual_radius)
    if not math.isfinite(visual_radius) or visual_radius <= 0.0:
        raise ValueError("selected_detection.visual_radius_m must be positive")
    raw_object_center = document.get("object_center_mujoco_m")
    raw_gripper_center = document.get("gripper_center_mujoco_m")
    if raw_object_center is not None or raw_gripper_center is not None:
        if raw_object_center is None or raw_gripper_center is None:
            raise ValueError(
                "vision result must provide both object_center_mujoco_m and "
                "gripper_center_mujoco_m"
            )
        object_center = _finite_vector(
            raw_object_center, 3, "object center"
        )
        gripper_center = _finite_vector(
            raw_gripper_center, 3, "gripper center"
        )
    else:
        # Explicit schema-v1 compatibility. Old results mislabeled one visible
        # surface sample as grasp_point_mujoco_m, so reconstruct the same class-
        # prior center used by the former planner instead of treating it as a
        # measured object or gripper center.
        if legacy_surface_point is None:
            raise ValueError(
                "vision result has no object/gripper centers or legacy surface point"
            )
        normal = np.asarray(table_normal, dtype=float)
        table_top = (
            np.asarray(table_center, dtype=float)
            + normal * table_half_extents[2]
        )
        target_array = np.asarray(legacy_surface_point, dtype=float)
        projected = target_array - normal * float(
            np.dot(target_array - table_top, normal)
        )
        legacy_center = projected + normal * half_height
        object_center = tuple(float(value) for value in legacy_center)
        gripper_center = tuple(
            float(value)
            for value in legacy_center + normal * SIDE_GRASP_HEIGHT_OFFSET_M
        )
    return TargetObjectVisual(
        class_name=class_name,
        object_center_m=object_center,
        gripper_center_m=gripper_center,
        object_height_m=object_height,
        quaternion_wxyz=table_quaternion,
        geom_type=geom_type,
        size_m=size,
        rgba=rgba,
        visual_radius_m=visual_radius,
    )


def load_table_obstacle(path: Path) -> tuple[tuple[float, float, float], TableObstacle]:
    """Load the gripper center and fitted table box emitted by vision."""
    source = path.expanduser().resolve()
    document = json.loads(source.read_text(encoding="utf-8"))
    raw_gripper_center = document.get("gripper_center_mujoco_m")
    raw_legacy_surface = document.get("grasp_point_mujoco_m")
    legacy_surface = (
        _finite_vector(raw_legacy_surface, 3, "legacy surface point")
        if raw_legacy_surface is not None
        else None
    )
    if raw_gripper_center is not None:
        gripper_center = _finite_vector(
            raw_gripper_center, 3, "gripper center"
        )
    elif legacy_surface is not None:
        # Replaced with the reconstructed center below when selected_detection
        # is available. Retain this value for old position-only scene files.
        gripper_center = legacy_surface
    else:
        raise ValueError(
            "vision result is missing gripper_center_mujoco_m"
        )
    table = document.get("table_plane_mujoco")
    if not isinstance(table, dict):
        raise ValueError("table_plane_mujoco is missing")
    box = table.get("mujoco_collision_box")
    if not isinstance(box, dict):
        raise ValueError("table mujoco_collision_box is missing")
    center = _finite_vector(
        box.get("pos_mujoco_m", box.get("center_mujoco_m")), 3, "table center"
    )
    quat = _finite_vector(
        box.get("quat_wxyz", box.get("quaternion_wxyz")), 4, "table quaternion"
    )
    quat_norm = math.sqrt(sum(value * value for value in quat))
    if quat_norm < 1e-12:
        raise ValueError("table quaternion must be non-zero")
    quat = tuple(value / quat_norm for value in quat)
    half_extents = _finite_vector(
        box.get("size_half_extents_m"), 3, "table half extents"
    )
    if any(value <= 0.0 for value in half_extents):
        raise ValueError("table half extents must be positive")
    equation = table.get("plane_equation")
    if not isinstance(equation, dict):
        raise ValueError("table plane_equation is missing")
    normal = _finite_vector(equation.get("normal_mujoco"), 3, "table normal")
    normal_norm = math.sqrt(sum(value * value for value in normal))
    if normal_norm < 1e-12:
        raise ValueError("table normal must be non-zero")
    normal = tuple(value / normal_norm for value in normal)
    if normal[2] < 0.0:
        normal = tuple(-value for value in normal)
    target_object = _target_object_visual(
        document,
        legacy_surface,
        center,
        quat,
        half_extents,
        normal,
    )
    if target_object is not None:
        gripper_center = target_object.gripper_center_m
    return gripper_center, TableObstacle(
        center, quat, half_extents, normal, target_object
    )


@dataclass(frozen=True)
class _UrdfJoint:
    name: str
    kind: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    axis: tuple[float, float, float]
    lower: float
    upper: float


@dataclass(frozen=True)
class _UrdfVisual:
    xyz: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    mesh_path: Path
    mesh_scale: tuple[float, float, float]
    rgba: tuple[float, float, float, float]


def _robot_urdf_candidates() -> tuple[Path, ...]:
    home = Path.home()
    version = "X2_URDF-v1.3.0"
    return (
        home / "x2_arm_sim" / "urdf" / "x2_ultra.urdf",
        home / "下载" / f"{version}(1)" / version / "x2_ultra.urdf",
        home / "下载" / version / "x2_ultra.urdf",
        home / "Downloads" / f"{version}(1)" / version / "x2_ultra.urdf",
        home / "Downloads" / version / "x2_ultra.urdf",
        home
        / "Raicom2026"
        / "mc"
        / "bin"
        / "mc_param"
        / "src"
        / "robot"
        / "lx2501_3_t2d5"
        / "urdf"
        / "x2_whole_body.urdf",
    )


def resolve_robot_visual_urdf(path: Path | None = None) -> Path | None:
    """Resolve an explicit, configured, or locally installed physical X2 URDF."""
    configured = os.environ.get(ROBOT_URDF_ENV)
    explicit = path is not None or bool(configured)
    candidates = (
        (Path(path) if path is not None else Path(configured).expanduser(),)
        if explicit
        else _robot_urdf_candidates()
    )
    for candidate in candidates:
        source = candidate.expanduser().resolve()
        if source.is_file():
            # Validate both XML and every referenced visual mesh before choosing
            # an automatically discovered installation. Some field images ship
            # x2_whole_body.urdf without its sibling meshes; that installation
            # must fall back to collision proxies instead of breaking planning.
            try:
                root = ET.parse(source).getroot()
                if root.tag != "robot":
                    raise ValueError(
                        f"robot visual URDF root must be <robot>: {source}"
                    )
                if not root.findall(".//visual/geometry/mesh"):
                    raise ValueError(
                        f"robot visual URDF has no visual meshes: {source}"
                    )
                _read_urdf_visuals(source)
            except (OSError, ET.ParseError, ValueError) as error:
                if explicit:
                    raise ValueError(
                        f"invalid robot visual URDF {source}: {error}"
                    ) from error
                continue
            return source
    if explicit:
        raise ValueError(f"robot visual URDF does not exist: {candidates[0]}")
    return None


def resolve_omnipicker_description(path: Path | None = None) -> Path | None:
    """Resolve the project-local OmniPicker xacro supplied with its STL meshes."""
    explicit = path is not None
    source = (
        path.expanduser().resolve()
        if path is not None
        else DEFAULT_OMNIPICKER_DESCRIPTION
    )
    if source.is_file():
        try:
            root = ET.parse(source).getroot()
        except (OSError, ET.ParseError) as error:
            raise ValueError(f"cannot parse OmniPicker description {source}: {error}") from error
        if not root.findall(".//visual/geometry/mesh"):
            raise ValueError(f"OmniPicker description has no visual meshes: {source}")
        return source
    if explicit:
        raise ValueError(f"OmniPicker description does not exist: {source}")
    return None


def _material_rgba(element: ET.Element, materials: dict[str, tuple[float, ...]]) -> tuple[float, ...]:
    material = element.find("material")
    if material is None:
        return (0.72, 0.74, 0.78, 1.0)
    color = material.find("color")
    if color is not None:
        return _numbers(color.attrib.get("rgba"), 4, (0.72, 0.74, 0.78, 1.0))
    name = material.attrib.get("name")
    return materials.get(name or "", (0.72, 0.74, 0.78, 1.0))


def _resolve_mesh_path(urdf_path: Path, filename: str) -> Path:
    if filename.startswith("file://"):
        source = Path(filename[7:])
    elif filename.startswith("package://"):
        package_uri = filename[len("package://"):]
        package_name, separator, relative = package_uri.partition("/")
        if not separator:
            raise ValueError(f"invalid package mesh URI: {filename}")
        package_roots = [
            ancestor
            for ancestor in urdf_path.parents
            if ancestor.name == package_name
        ]
        if not package_roots:
            raise ValueError(f"cannot resolve package mesh URI: {filename}")
        source = package_roots[0] / relative
    elif filename.startswith("${mesh_path}/"):
        package_root = urdf_path.parent.parent
        source = package_root / "meshes" / filename[len("${mesh_path}/"):]
    else:
        source = Path(filename)
    resolved = source if source.is_absolute() else urdf_path.parent / source
    resolved = resolved.resolve()
    if not resolved.is_file():
        raise ValueError(f"robot visual mesh does not exist: {resolved}")
    if resolved.suffix.lower() == ".stl":
        with resolved.open("rb") as stream:
            header = stream.read(84)
        if len(header) == 84:
            face_count = struct.unpack("<I", header[80:84])[0]
            if resolved.stat().st_size == 84 + 50 * face_count and face_count > 200_000:
                cached = resolved.parent / "mujoco" / resolved.name
                if not cached.is_file():
                    raise ValueError(
                        f"STL exceeds MuJoCo's 200000-face limit ({face_count}): "
                        f"{resolved}; run tools/prepare_omnipicker_meshes.py"
                    )
                resolved = cached.resolve()
    return resolved


def _read_urdf_visuals(
    path: Path,
    *,
    minimum_meshed_links: int = 20,
) -> dict[str, tuple[_UrdfVisual, ...]]:
    root = ET.parse(path).getroot()
    materials: dict[str, tuple[float, ...]] = {}
    for material in root.findall(".//material"):
        color = material.find("color")
        name = material.attrib.get("name")
        if name and color is not None:
            materials[name] = _numbers(
                color.attrib.get("rgba"), 4, (0.72, 0.74, 0.78, 1.0)
            )
    result: dict[str, tuple[_UrdfVisual, ...]] = {}
    for link in root.findall(".//link"):
        link_name = link.attrib.get("name")
        if not link_name:
            continue
        visuals: list[_UrdfVisual] = []
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            mesh = None if geometry is None else geometry.find("mesh")
            if mesh is None or not mesh.attrib.get("filename"):
                continue
            origin = visual.find("origin")
            xyz = _numbers(
                None if origin is None else origin.attrib.get("xyz"),
                3,
                (0.0, 0.0, 0.0),
            )
            rpy = _numbers(
                None if origin is None else origin.attrib.get("rpy"),
                3,
                (0.0, 0.0, 0.0),
            )
            visuals.append(
                _UrdfVisual(
                    xyz=xyz,
                    quaternion=_rpy_quaternion(rpy),
                    mesh_path=_resolve_mesh_path(path, mesh.attrib["filename"]),
                    mesh_scale=_numbers(
                        mesh.attrib.get("scale"), 3, (1.0, 1.0, 1.0)
                    ),
                    rgba=_material_rgba(visual, materials),
                )
            )
        if visuals:
            result[link_name] = tuple(visuals)
    if len(result) < minimum_meshed_links:
        raise ValueError(
            f"robot visual URDF contains only {len(result)} meshed links: {path}"
        )
    return result


def _read_omnipicker_visuals(path: Path) -> dict[str, tuple[_UrdfVisual, ...]]:
    source = _read_urdf_visuals(path, minimum_meshed_links=9)
    result: dict[str, tuple[_UrdfVisual, ...]] = {}
    prefix = "L" if INSTALLED_GRIPPER_SIDE == "left" else "R"
    for link_name, visuals in source.items():
        target = (
            f"{prefix}_omnipicker_base_link"
            if link_name == "base_link"
            else f"{prefix}_{link_name}"
        )
        result[target] = visuals
    return result


def _read_urdf_joints(path: Path) -> dict[str, _UrdfJoint]:
    root = ET.parse(path).getroot()
    by_child: dict[str, _UrdfJoint] = {}
    for element in root.findall("joint"):
        name = str(element.attrib["name"])
        kind = str(element.attrib.get("type", "fixed"))
        parent_element = element.find("parent")
        child_element = element.find("child")
        if parent_element is None or child_element is None:
            raise ValueError(f"URDF joint {name!r} lacks parent/child")
        origin = element.find("origin")
        xyz = _numbers(
            None if origin is None else origin.attrib.get("xyz"),
            3,
            (0.0, 0.0, 0.0),
        )
        rpy = _numbers(
            None if origin is None else origin.attrib.get("rpy"),
            3,
            (0.0, 0.0, 0.0),
        )
        axis_element = element.find("axis")
        axis = _numbers(
            None if axis_element is None else axis_element.attrib.get("xyz"),
            3,
            (1.0, 0.0, 0.0),
        )
        limit = element.find("limit")
        lower = float(limit.attrib.get("lower", -math.pi)) if limit is not None else 0.0
        upper = float(limit.attrib.get("upper", math.pi)) if limit is not None else 0.0
        joint = _UrdfJoint(
            name=name,
            kind=kind,
            parent=str(parent_element.attrib["link"]),
            child=str(child_element.attrib["link"]),
            xyz=xyz,
            quaternion=_rpy_quaternion(rpy),
            axis=axis,
            lower=lower,
            upper=upper,
        )
        if joint.child in by_child:
            raise ValueError(f"URDF link {joint.child!r} has multiple parents")
        by_child[joint.child] = joint
    return by_child


def _path_to_root(by_child: dict[str, _UrdfJoint], link: str) -> list[_UrdfJoint]:
    path: list[_UrdfJoint] = []
    current = link
    while current != "base_link":
        try:
            joint = by_child[current]
        except KeyError as error:
            raise ValueError(f"URDF cannot connect {link!r} to base_link") from error
        path.append(joint)
        current = joint.parent
    path.reverse()
    return path


def _proxy_radius(link_name: str) -> float:
    if "wrist" in link_name:
        return 0.028
    if "elbow" in link_name:
        return 0.038
    if "shoulder" in link_name:
        return 0.043
    return 0.035


def _add_link_proxy(
    body: ET.Element,
    link_name: str,
    child_joints: list[_UrdfJoint],
    *,
    visible: bool,
) -> None:
    if not any(token in link_name for token in ("shoulder", "elbow", "wrist")):
        return
    arm_children = [
        joint
        for joint in child_joints
        if "shoulder" in joint.name
        or "elbow" in joint.name
        or "wrist" in joint.name
        or "omnipicker" in joint.name.lower()
    ]
    radius = _proxy_radius(link_name)
    usable = [joint for joint in arm_children if np.linalg.norm(joint.xyz) > 0.045]
    if usable:
        for index, joint in enumerate(usable):
            vector = np.asarray(joint.xyz, dtype=float)
            start = vector * 0.10
            end = vector * 0.90
            ET.SubElement(
                body,
                "geom",
                name=f"proxy_{link_name}_{index}",
                type="capsule",
                fromto=_format(tuple(start) + tuple(end)),
                size=f"{radius:.6g}",
                rgba="0.35 0.55 0.85 0.55" if visible else "0.35 0.55 0.85 0",
            )
    elif any(token in link_name for token in ("shoulder", "elbow", "wrist")):
        ET.SubElement(
            body,
            "geom",
            name=f"proxy_{link_name}",
            type="sphere",
            size=f"{radius:.6g}",
            rgba="0.35 0.55 0.85 0.55" if visible else "0.35 0.55 0.85 0",
        )


def _add_full_body_proxy_visual(
    body: ET.Element,
    link_name: str,
    child_joints: list[_UrdfJoint],
) -> None:
    """Draw non-colliding primitives when the SDK URDF has no visual meshes."""

    common = {
        "contype": "0",
        "conaffinity": "0",
        "density": "0",
        "group": "2",
    }
    blue = "0.16 0.42 0.72 0.92"
    dark = "0.12 0.18 0.27 0.96"
    joint = "0.26 0.62 0.88 0.96"

    if link_name == "pelvis":
        ET.SubElement(
            body,
            "geom",
            name="proxy_visual_pelvis",
            type="box",
            pos="0 0 0.025",
            size="0.12 0.105 0.07",
            rgba=dark,
            **common,
        )
        return
    if link_name in {"waist_yaw_link", "waist_pitch_link"}:
        ET.SubElement(
            body,
            "geom",
            name=f"proxy_visual_{link_name}",
            type="cylinder",
            pos="0 0 0.035",
            size="0.075 0.045",
            rgba=joint,
            **common,
        )
        return
    if link_name == "head_yaw_link":
        ET.SubElement(
            body,
            "geom",
            name="proxy_visual_neck",
            type="cylinder",
            pos="0 0 0.035",
            size="0.045 0.05",
            rgba=joint,
            **common,
        )
        return
    if link_name == "head_pitch_link":
        ET.SubElement(
            body,
            "geom",
            name="proxy_visual_head",
            type="ellipsoid",
            pos="0.025 0 0.075",
            size="0.09 0.075 0.105",
            rgba=dark,
            **common,
        )
        return
    if link_name == "lidar_chest_front":
        ET.SubElement(
            body,
            "geom",
            name="proxy_visual_chest_lidar",
            type="cylinder",
            size="0.035 0.025",
            rgba="0.05 0.07 0.09 1",
            **common,
        )
        return
    if "ankle_roll_link" in link_name:
        ET.SubElement(
            body,
            "geom",
            name=f"proxy_visual_{link_name}",
            type="box",
            pos="0.045 0 -0.05",
            size="0.105 0.048 0.025",
            rgba=dark,
            **common,
        )
        return
    if "ankle_pitch_link" in link_name or "hip_pitch_link" in link_name:
        ET.SubElement(
            body,
            "geom",
            name=f"proxy_visual_{link_name}",
            type="sphere",
            size="0.055",
            rgba=joint,
            **common,
        )
        return
    leg_segment_tokens = ("hip_roll_link", "hip_yaw_link", "knee_link")
    if any(token in link_name for token in leg_segment_tokens):
        next_joint = next(
            (
                candidate
                for candidate in child_joints
                if np.linalg.norm(candidate.xyz) > 0.08
            ),
            None,
        )
        if next_joint is not None:
            vector = np.asarray(next_joint.xyz, dtype=float)
            ET.SubElement(
                body,
                "geom",
                name=f"proxy_visual_{link_name}",
                type="capsule",
                fromto=_format(tuple(vector * 0.08) + tuple(vector * 0.92)),
                size="0.052" if "hip" in link_name else "0.045",
                rgba=blue,
                **common,
            )


def _build_mjcf(
    profile: RobotProfile,
    urdf_path: Path,
    visual_urdf_path: Path | None,
    omnipicker_description_path: Path | None,
    obstacle: TableObstacle | None,
    probe_distance_m: float,
    tool_pose: ToolPoseConfig,
) -> str:
    by_child = _read_urdf_joints(urdf_path)
    robot_visuals = (
        _read_urdf_visuals(visual_urdf_path)
        if visual_urdf_path is not None
        else {}
    )
    omnipicker_visuals = (
        _read_omnipicker_visuals(omnipicker_description_path)
        if omnipicker_description_path is not None
        else {}
    )
    if omnipicker_visuals:
        # X2 Ultra's wrist-roll meshes include sculpted human-hand geometry.
        # OmniPicker replaces those visuals while retaining the wrist bodies,
        # joints and invisible arm collision proxies used by IK/planning.
        robot_visuals = {
            link_name: link_visuals
            for link_name, link_visuals in robot_visuals.items()
            if link_name not in OMNIPICKER_REPLACED_VISUAL_LINKS
        }
    known_links = set(by_child)
    missing_omnipicker_links = sorted(set(omnipicker_visuals) - known_links)
    if missing_omnipicker_links:
        raise ValueError(
            "OmniPicker visual links are missing from the official IK URDF: "
            + ", ".join(missing_omnipicker_links)
        )
    visuals = {**robot_visuals, **omnipicker_visuals}
    if visuals:
        # Use the complete SDK tree so the Viewer shows the physical full body.
        # Visuals are mapped by link name but never supply joint transforms.
        selected = {joint.name: joint for joint in by_child.values()}
    else:
        paths = (
            _path_to_root(by_child, profile.left_ee_frame),
            _path_to_root(by_child, profile.right_ee_frame),
        )
        selected = {joint.name: joint for path in paths for joint in path}
    children: dict[str, list[_UrdfJoint]] = {}
    for joint in selected.values():
        children.setdefault(joint.parent, []).append(joint)
    active = set(profile.arm_pos_order)
    active.update((RIGHT_CLAW_JOINT_NAME, RIGHT_WIDE_JOINT_NAME))

    root = ET.Element("mujoco", model=f"graspv2_{profile.name}")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", gravity="0 0 0", timestep="0.002")
    ET.SubElement(root, "statistic", center="0.25 0 0.75", extent="1.45")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "headlight",
        diffuse="0.58 0.58 0.58",
        ambient="0.28 0.28 0.28",
        specular="0.75 0.75 0.75",
    )
    ET.SubElement(visual, "rgba", haze="0.15 0.25 0.35 1")
    ET.SubElement(visual, "global", azimuth="200", elevation="-20")
    default = ET.SubElement(root, "default")
    ET.SubElement(
        default,
        "geom",
        contype="1",
        conaffinity="1",
        density="500",
        friction="0.8 0.02 0.01",
    )
    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        type="skybox",
        builtin="gradient",
        rgb1="0.30 0.50 0.70",
        rgb2="0.02 0.03 0.05",
        width="512",
        height="3072",
    )
    ET.SubElement(
        asset,
        "texture",
        type="2d",
        name="groundplane_texture",
        builtin="checker",
        mark="edge",
        rgb1="0.26 0.43 0.52",
        rgb2="0.18 0.31 0.38",
        markrgb="0.8 0.9 1",
        width="300",
        height="300",
    )
    ET.SubElement(
        asset,
        "material",
        name="groundplane_material",
        texture="groundplane_texture",
        texuniform="true",
        texrepeat="5 5",
        reflectance="0.16",
    )
    ET.SubElement(
        asset,
        "material",
        name="planning_table_material",
        rgba="0.58 0.31 0.12 1",
        specular="0.25",
        shininess="0.18",
        reflectance="0.08",
    )
    visual_assets: dict[tuple[str, int], str] = {}
    for link_name, link_visuals in sorted(visuals.items()):
        for index, visual in enumerate(link_visuals):
            asset_name = f"actual_{link_name}_{index}"
            visual_assets[(link_name, index)] = asset_name
            ET.SubElement(
                asset,
                "mesh",
                name=asset_name,
                file=str(visual.mesh_path),
                scale=_format(visual.mesh_scale),
            )
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world,
        "light",
        name="scene_key_light",
        directional="true",
        pos="-1.2 -1.0 3.0",
        dir="0.35 0.25 -1",
        diffuse="0.78 0.76 0.72",
        specular="0.35 0.35 0.32",
        castshadow="true",
    )
    ET.SubElement(
        world,
        "light",
        name="scene_fill_light",
        directional="true",
        pos="1.5 1.2 2.2",
        dir="-0.45 -0.25 -1",
        diffuse="0.30 0.36 0.45",
        specular="0.08 0.10 0.14",
        castshadow="false",
    )
    ET.SubElement(
        world,
        "geom",
        name="scene_floor",
        type="plane",
        pos="0 0 -0.005",
        size="0 0 0.05",
        material="groundplane_material",
        contype="0",
        conaffinity="0",
        density="0",
        group="0",
    )
    base = ET.SubElement(
        world,
        "body",
        name="base_link",
        pos=_format(profile.base_world_xyz),
    )

    def add_children(parent_link: str, parent_body: ET.Element) -> None:
        for joint in sorted(children.get(parent_link, []), key=lambda value: value.name):
            body = ET.SubElement(
                parent_body,
                "body",
                name=joint.child,
                pos=_format(joint.xyz),
                quat=_format(joint.quaternion),
            )
            if joint.name in active:
                ET.SubElement(
                    body,
                    "joint",
                    name=joint.name,
                    type="hinge",
                    axis=_format(joint.axis),
                    range=_format((joint.lower, joint.upper)),
                    damping="0.2",
                )
                if joint.name in (RIGHT_CLAW_JOINT_NAME, RIGHT_WIDE_JOINT_NAME):
                    ET.SubElement(
                        body,
                        "inertial",
                        pos="0 0 0",
                        mass="0.01",
                        diaginertia="1e-5 1e-5 1e-5",
                    )
            is_omnipicker = joint.child in omnipicker_visuals
            for index, visual in enumerate(visuals.get(joint.child, ())):
                ET.SubElement(
                    body,
                    "geom",
                    name=f"actual_visual_{joint.child}_{index}",
                    type="mesh",
                    mesh=visual_assets[(joint.child, index)],
                    pos=_format(visual.xyz),
                    quat=_format(visual.quaternion),
                    rgba=_format(visual.rgba),
                    # OmniPicker parts share one contact class, so they do not
                    # collide with one another but still collide with the table
                    # and the arm proxies.  This replaces the old fake-hand box.
                    contype="2" if is_omnipicker else "0",
                    conaffinity="1" if is_omnipicker else "0",
                    density="0",
                    group="2",
                )
            if joint.child == "torso_link":
                ET.SubElement(
                    body,
                    "geom",
                    name="proxy_torso",
                    type="box",
                    pos="0.02 0 0.10",
                    size="0.105 0.10 0.20",
                    rgba=(
                        "0.22 0.38 0.62 0.82"
                        if not robot_visuals
                        else "0.5 0.5 0.55 0"
                    ),
                )
            if not robot_visuals:
                _add_full_body_proxy_visual(
                    body,
                    joint.child,
                    children.get(joint.child, []),
                )
            _add_link_proxy(
                body,
                joint.child,
                children.get(joint.child, []),
                visible=not bool(robot_visuals),
            )
            if joint.child in (profile.left_ee_frame, profile.right_ee_frame):
                side = "left" if joint.child == profile.left_ee_frame else "right"
                tcp = tool_pose.for_side(side)
                ET.SubElement(
                    body,
                    "site",
                    name=LEFT_SITE_NAME if side == "left" else RIGHT_SITE_NAME,
                    type="sphere",
                    pos=_format(tcp.translation_m),
                    quat=_format(_rpy_quaternion(tcp.rpy_rad)),
                    size="0.008",
                    rgba="1 0.1 0.1 1",
                )
            add_children(joint.child, body)

    add_children("base_link", base)
    if obstacle is not None:
        ET.SubElement(
            world,
            "geom",
            name=TABLE_GEOM_NAME,
            type="box",
            pos=_format(obstacle.center_m),
            quat=_format(obstacle.quaternion_wxyz),
            size=_format(obstacle.half_extents_m),
            margin=f"{probe_distance_m:.12g}",
            gap="0",
            material="planning_table_material",
        )
        table_visuals = ET.SubElement(
            world,
            "body",
            name="planning_table_visuals",
            pos=_format(obstacle.center_m),
            quat=_format(obstacle.quaternion_wxyz),
        )
        half_x, half_y, half_z = obstacle.half_extents_m
        leg_half_height = max(0.08, (obstacle.center_m[2] - half_z) / 2.0)
        leg_x = max(0.0, half_x - min(0.07, half_x * 0.28))
        leg_y = max(0.0, half_y - min(0.07, half_y * 0.20))
        leg_z = -(half_z + leg_half_height)
        leg_half_width = min(0.035, max(0.018, min(half_x, half_y) * 0.12))
        for index, (x, y) in enumerate(
            ((leg_x, leg_y), (leg_x, -leg_y), (-leg_x, leg_y), (-leg_x, -leg_y))
        ):
            ET.SubElement(
                table_visuals,
                "geom",
                name=f"planning_table_leg_{index}",
                type="box",
                pos=_format((x, y, leg_z)),
                size=_format((leg_half_width, leg_half_width, leg_half_height)),
                material="planning_table_material",
                contype="0",
                conaffinity="0",
                density="0",
                group="2",
            )
        if obstacle.target_object is not None:
            target_object = obstacle.target_object
            target_rotation = np.empty(9, dtype=float)
            mujoco.mju_quat2Mat(
                target_rotation,
                np.asarray(target_object.quaternion_wxyz, dtype=float),
            )
            target_rotation = target_rotation.reshape(3, 3)
            local_gripper_center = target_rotation.T @ (
                np.asarray(target_object.gripper_center_m, dtype=float)
                - np.asarray(target_object.object_center_m, dtype=float)
            )
            target_body = ET.SubElement(
                world,
                "body",
                name=TARGET_OBJECT_BODY_NAME,
                mocap="true",
                pos=_format(target_object.object_center_m),
                quat=_format(target_object.quaternion_wxyz),
            )
            ET.SubElement(
                target_body,
                "geom",
                name=TARGET_OBJECT_GEOM_NAME,
                type=target_object.geom_type,
                pos="0 0 0",
                size=_format(target_object.size_m),
                rgba=_format(target_object.rgba),
                contype="0",
                conaffinity="0",
                density="0",
                group="2",
            )
            ET.SubElement(
                target_body,
                "site",
                name=TARGET_GRIPPER_CENTER_SITE_NAME,
                type="sphere",
                pos=_format(local_gripper_center),
                size="0.009",
                rgba="0.15 1 0.2 1",
                group="2",
            )
    return ET.tostring(root, encoding="unicode")


class RobotSimulation:
    """Kinematic MuJoCo scene used for FK parity, visuals and collision gates."""

    def __init__(
        self,
        profile: RobotProfile,
        ik: OfficialIK,
        obstacle: TableObstacle | None = None,
        *,
        visual_urdf_path: Path | None = None,
        omnipicker_description_path: Path | None = None,
        probe_distance_m: float = 0.12,
    ):
        if profile != ik.profile:
            raise ValueError("simulation and IK profiles differ")
        self.profile = profile
        self.ik = ik
        self.obstacle = obstacle
        self.visual_urdf_path = (
            visual_urdf_path.expanduser().resolve()
            if visual_urdf_path is not None
            else None
        )
        self.omnipicker_description_path = resolve_omnipicker_description(
            omnipicker_description_path
        )
        self.probe_distance_m = float(probe_distance_m)
        self.xml = _build_mjcf(
            profile,
            ik.urdf_path,
            self.visual_urdf_path,
            self.omnipicker_description_path,
            obstacle,
            probe_distance_m,
            ik.tool_pose,
        )
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.qpos_indices = {}
        for name in profile.arm_pos_order:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise RuntimeError(f"simulation joint is missing: {name}")
            self.qpos_indices[name] = int(self.model.jnt_qposadr[joint_id])
        self.gripper_qpos_indices: dict[str, int] = {}
        for name in (RIGHT_CLAW_JOINT_NAME, RIGHT_WIDE_JOINT_NAME):
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise RuntimeError(f"simulation gripper joint is missing: {name}")
            self.gripper_qpos_indices[name] = int(
                self.model.jnt_qposadr[joint_id]
            )
        self.site_ids = {
            "left": mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, LEFT_SITE_NAME
            ),
            "right": mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, RIGHT_SITE_NAME
            ),
        }
        self.table_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, TABLE_GEOM_NAME
        )
        self.target_object_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, TARGET_OBJECT_GEOM_NAME
        )
        self.target_object_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, TARGET_OBJECT_BODY_NAME
        )
        self.target_object_mocap_id = (
            int(self.model.body_mocapid[self.target_object_body_id])
            if self.target_object_body_id >= 0
            else -1
        )
        target_object = obstacle.target_object if obstacle is not None else None
        self.target_object_initial_center_m = (
            target_object.object_center_m if target_object is not None else None
        )
        self.target_object_initial_quaternion_wxyz = (
            target_object.quaternion_wxyz if target_object is not None else None
        )
        self.set_arm_pos(ik.ready_arm_pos())
        self.set_gripper_position(0.0)

    def set_arm_pos(self, arm_pos: Iterable[float]) -> None:
        values = tuple(float(value) for value in arm_pos)
        if len(values) != len(self.profile.arm_pos_order):
            raise ValueError(
                f"arm_pos must contain {len(self.profile.arm_pos_order)} values"
            )
        for name, value in zip(self.profile.arm_pos_order, values):
            self.data.qpos[self.qpos_indices[name]] = value
        mujoco.mj_forward(self.model, self.data)

    def set_gripper_position(self, position: float) -> None:
        """Set normalized right OmniPicker opening (0=closed, 1=fully open)."""

        value = float(position)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError("gripper position must be within [0, 1]")
        self.data.qpos[
            self.gripper_qpos_indices[RIGHT_CLAW_JOINT_NAME]
        ] = -value
        self.data.qpos[
            self.gripper_qpos_indices[RIGHT_WIDE_JOINT_NAME]
        ] = value
        mujoco.mj_forward(self.model, self.data)

    def target_object_pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        if self.target_object_mocap_id < 0:
            raise RuntimeError("simulation has no recognized target object")
        position = tuple(
            float(value) for value in self.data.mocap_pos[self.target_object_mocap_id]
        )
        quaternion = tuple(
            float(value) for value in self.data.mocap_quat[self.target_object_mocap_id]
        )
        return position, quaternion

    def set_target_object_pose(
        self,
        center_m: Iterable[float],
        quaternion_wxyz: Iterable[float] | None = None,
    ) -> None:
        """Move the recognized target mocap body without enabling fake contacts."""

        if self.target_object_mocap_id < 0:
            raise RuntimeError("simulation has no recognized target object")
        center = np.asarray(tuple(center_m), dtype=float)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError("target object center must contain three finite values")
        self.data.mocap_pos[self.target_object_mocap_id] = center
        if quaternion_wxyz is not None:
            quaternion = np.asarray(tuple(quaternion_wxyz), dtype=float)
            norm = float(np.linalg.norm(quaternion))
            if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
                raise ValueError("target quaternion must contain four finite values")
            if norm <= 1e-12:
                raise ValueError("target quaternion must be non-zero")
            self.data.mocap_quat[self.target_object_mocap_id] = quaternion / norm
        mujoco.mj_forward(self.model, self.data)

    def set_side_joints(
        self,
        side: str,
        joints: Iterable[float],
        base_arm_pos: Iterable[float],
    ) -> list[float]:
        arm_pos = list(float(value) for value in base_arm_pos)
        values = tuple(float(value) for value in joints)
        expected = self.profile.joints_for_side(side)
        if len(values) != len(expected):
            raise ValueError(f"{side} joints must contain {len(expected)} values")
        offset = 0 if side == "left" else self.profile.arm_dof
        arm_pos[offset:offset + self.profile.arm_dof] = values
        self.set_arm_pos(arm_pos)
        return arm_pos

    def site_world_xyz(self, side: str) -> tuple[float, float, float]:
        site_id = self.site_ids[side]
        return tuple(float(value) for value in self.data.site_xpos[site_id])

    def site_world_rotation(self, side: str) -> np.ndarray:
        site_id = self.site_ids[side]
        return self.data.site_xmat[site_id].reshape(3, 3).copy()

    def collision_report(self, clearance_m: float = 0.0) -> CollisionReport:
        if clearance_m < 0.0 or clearance_m > self.probe_distance_m:
            raise ValueError(
                f"clearance must be within [0, {self.probe_distance_m}]"
            )
        table_contacts: list[tuple[float, str]] = []
        self_contacts: list[str] = []
        for contact in self.data.contact[: self.data.ncon]:
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if self.table_geom_id >= 0 and self.table_geom_id in (geom1, geom2):
                other = geom2 if geom1 == self.table_geom_id else geom1
                body_id = int(self.model.geom_bodyid[other])
                body = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, body_id
                ) or f"body#{body_id}"
                table_contacts.append((float(contact.dist), body))
            elif float(contact.dist) < -1e-6:
                body1 = int(self.model.geom_bodyid[geom1])
                body2 = int(self.model.geom_bodyid[geom2])
                ancestors: dict[int, int] = {}
                current = body1
                distance = 0
                while current > 0:
                    ancestors[current] = distance
                    current = int(self.model.body_parentid[current])
                    distance += 1
                current = body2
                distance = 0
                graph_distance = None
                while current > 0:
                    if current in ancestors:
                        graph_distance = distance + ancestors[current]
                        break
                    current = int(self.model.body_parentid[current])
                    distance += 1
                # Primitive proxies overlap around compact multi-axis joint
                # packs.  They are one rigid assembly for collision purposes.
                if graph_distance is not None and graph_distance <= 3:
                    continue
                first = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, geom1
                ) or f"geom#{geom1}"
                second = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, geom2
                ) or f"geom#{geom2}"
                self_contacts.append(f"{first}<->{second}")
        if table_contacts:
            table_distance, nearest_body = min(table_contacts, key=lambda item: item[0])
        else:
            table_distance, nearest_body = self.probe_distance_m, None
        return CollisionReport(
            valid=(
                table_distance + COLLISION_CLEARANCE_NUMERICAL_TOLERANCE_M
                >= clearance_m
                and not self_contacts
            ),
            minimum_table_distance_m=table_distance,
            nearest_table_body=nearest_body,
            self_contacts=tuple(sorted(set(self_contacts))),
        )


def validate_fk_alignment(
    profile: RobotProfile,
    ik: OfficialIK,
    *,
    random_samples: int = 8,
    seed: int = 7,
) -> FKAlignment:
    """Verify that the generated MuJoCo chain matches the official SDK."""
    simulation = RobotSimulation(profile, ik)
    rng = np.random.default_rng(seed)
    lower_left, upper_left = ik.joint_limits_for_side("left")
    lower_right, upper_right = ik.joint_limits_for_side("right")
    samples = [np.asarray(ik.ready_arm_pos(), dtype=float)]
    for _ in range(random_samples):
        samples.append(
            np.concatenate(
                [
                    rng.uniform(lower_left, upper_left),
                    rng.uniform(lower_right, upper_right),
                ]
            )
        )
    max_position = 0.0
    max_orientation = 0.0
    for arm_pos in samples:
        simulation.set_arm_pos(arm_pos)
        for side in ("left", "right"):
            expected_xyz = np.asarray(ik.fk_world(side, arm_pos), dtype=float)
            actual_xyz = np.asarray(simulation.site_world_xyz(side), dtype=float)
            max_position = max(
                max_position, float(np.linalg.norm(expected_xyz - actual_xyz))
            )
            expected_rotation = _rpy_matrix(ik.fk_rpy(side, arm_pos))
            actual_rotation = simulation.site_world_rotation(side)
            cosine = (np.trace(expected_rotation.T @ actual_rotation) - 1.0) / 2.0
            angle = math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))
            max_orientation = max(max_orientation, angle)
    return FKAlignment(
        profile=profile.name,
        samples=len(samples) * 2,
        maximum_position_error_m=max_position,
        maximum_orientation_error_deg=max_orientation,
    )
