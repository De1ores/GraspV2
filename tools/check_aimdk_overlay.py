#!/usr/bin/env python3
"""Validate the AimDK interfaces used by GraspV2 without contacting ROS."""

from __future__ import annotations

import sys


def _fields(message_type: type) -> set[str]:
    return set(message_type.get_fields_and_field_types())


def _require_exact(message_type: type, expected: set[str]) -> None:
    actual = _fields(message_type)
    if actual != expected:
        raise RuntimeError(
            f"{message_type.__name__} fields are {sorted(actual)}, "
            f"expected {sorted(expected)}"
        )


def validate() -> None:
    try:
        from aimdk_msgs.msg import (
            HandCommand,
            HandCommandArray,
            HandStateArray,
            JointCommand,
            JointCommandArray,
            JointState,
            JointStateArray,
            McActionCommand,
            McInputAction,
            McInputSource,
            UpperBodyCommandArray,
        )
        from aimdk_msgs.srv import (
            GetCurrentInputSource,
            GetMcAction,
            SetMcAction,
            SetMcInputSource,
            SetMcPresetMotion,
        )
    except (ImportError, ModuleNotFoundError) as error:
        raise RuntimeError(
            f"required AimDK interface import failed: {error}"
        ) from error

    exact_schemas = {
        JointCommand: {
            "name",
            "position",
            "velocity",
            "effort",
            "stiffness",
            "damping",
        },
        JointCommandArray: {"header", "joints"},
        JointStateArray: {"header", "state", "joints"},
        HandCommand: {
            "name",
            "position",
            "velocity",
            "acceleration",
            "deceleration",
            "effort",
        },
        HandCommandArray: {
            "header",
            "left_hand_type",
            "left_hands",
            "right_hand_type",
            "right_hands",
        },
        HandStateArray: {
            "header",
            "left_hand_type",
            "left_hands",
            "left_touch_sensors",
            "right_hand_type",
            "right_hands",
            "right_touch_sensors",
        },
        UpperBodyCommandArray: {
            "header",
            "source",
            "hand_sub_mode",
            "head_pos",
            "arm_pos",
            "hand_pos",
        },
        McActionCommand: {"action", "action_desc"},
        McInputAction: {"value"},
        McInputSource: {"name", "priority", "timeout"},
        GetMcAction.Request: {"request"},
        GetMcAction.Response: {"header", "info"},
        SetMcAction.Request: {"header", "source", "command"},
        SetMcAction.Response: {"response"},
        GetCurrentInputSource.Request: {"request"},
        GetCurrentInputSource.Response: {"response", "input_source"},
        SetMcInputSource.Request: {"request", "action", "input_source"},
        SetMcInputSource.Response: {"response"},
    }
    for message_type, expected in exact_schemas.items():
        _require_exact(message_type, expected)

    joint_state_fields = _fields(JointState)
    joint_state_base = {"name", "position", "velocity", "effort"}
    allowed_joint_states = (
        joint_state_base | {"coil_temp", "motor_temp", "motor_vol"},
        joint_state_base | {"error_code"},
    )
    if joint_state_fields not in allowed_joint_states:
        raise RuntimeError(
            f"JointState fields are {sorted(joint_state_fields)}; no supported "
            "temperature/error-code layout matched"
        )

    preset_fields = _fields(SetMcPresetMotion.Request)
    preset_base = {"header", "area", "motion", "interrupt", "ani_path"}
    if preset_fields not in (preset_base, preset_base | {"play_timestamp"}):
        raise RuntimeError(
            f"SetMcPresetMotion_Request fields are {sorted(preset_fields)}; "
            "expected ani_path layout with optional play_timestamp"
        )

    expected_actions = {
        "INPUTACTION_ADD": 1001,
        "INPUTACTION_MODIFY": 1002,
        "INPUTACTION_DELETE": 1003,
        "INPUTACTION_ENABLE": 2001,
        "INPUTACTION_DISABLE": 2002,
    }
    action_values = {
        name: getattr(McInputAction, name, None) for name in expected_actions
    }
    if action_values != expected_actions:
        raise RuntimeError(
            f"McInputAction constants are {action_values}, expected {expected_actions}"
        )


def main() -> int:
    try:
        validate()
    except RuntimeError as error:
        print(f"Incompatible AimDK overlay: {error}", file=sys.stderr)
        return 1
    print("Compatible AimDK control schema")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
