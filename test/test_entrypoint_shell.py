from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).parents[1]


def _run_help(script: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment["GRASPV2_AIMDK_SETUP"] = "/nonexistent/aimdk/setup.bash"
    environment.pop("GRASPV2_X2_ENV_READY", None)
    return subprocess.run(
        [str(ROOT / script), "--help"],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )


def test_full_pipeline_help_does_not_probe_ros_or_aimdk() -> None:
    completed = _run_help("run_full_grasp_pipeline.sh")

    assert completed.returncode == 0
    assert completed.stdout.startswith("Usage:")
    assert "auto (default)" in completed.stdout
    assert "AimDK" not in completed.stderr
    assert "X2 MC environment ready" not in completed.stdout


def test_offline_help_does_not_install_or_require_aarch64() -> None:
    completed = _run_help("offline_run.sh")

    assert completed.returncode == 0
    assert completed.stdout.startswith("Usage:")
    assert "targets aarch64" not in completed.stderr


def test_existing_vision_plan_only_does_not_probe_ros_or_aimdk(
    tmp_path: Path,
) -> None:
    shutil.copy2(ROOT / "run_full_grasp_pipeline.sh", tmp_path)
    (tmp_path / "output").mkdir()
    (tmp_path / "output" / "result.json").write_text(
        '{"selected_detection":{"class_name":"cup"}}\n', encoding="utf-8"
    )
    planning_bin = tmp_path / ".venv" / "bin" / "python"
    planning_bin.parent.mkdir(parents=True)
    planning_bin.symlink_to(sys.executable)
    fake_planner = tmp_path / "run.sh"
    fake_planner.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    fake_planner.chmod(0o755)
    environment = os.environ.copy()
    environment["GRASPV2_AIMDK_SETUP"] = "/nonexistent/aimdk/setup.bash"
    completed = subprocess.run(
        [
            str(tmp_path / "run_full_grasp_pipeline.sh"),
            "--target-class",
            "cup",
            "--use-existing-vision",
            "--plan-only",
        ],
        cwd=tmp_path,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert completed.returncode == 0, completed.stderr
    assert "Plan-only complete" in completed.stdout
    assert "AimDK" not in completed.stderr


def test_svt_current_aimdk_overlay_precedes_legacy_overlay() -> None:
    selector = (ROOT / "tools" / "select_aimdk_setup.sh").read_text(
        encoding="utf-8"
    )
    current = "/home/svt/Raicom2026/aimdk_msgs/install/setup.bash"
    legacy = "/home/svt/Raicom2026-old/aimdk_msgs/install/setup.bash"

    assert current in selector
    assert selector.index(current) < selector.index(legacy)


def test_animation_backend_has_noninteractive_robot_address_fallback() -> None:
    backend = (ROOT / "tools" / "animation_backend.sh").read_text(
        encoding="utf-8"
    )

    assert "${GRASPV2_ROBOT_ADDRESS:-10.0.200.40}" in backend
    assert "${GRASPV2_ROBOT_FALLBACK_ADDRESS:-10.0.1.40}" in backend
    assert "${GRASPV2_ROBOT_PASSWORD:-1}" in backend
    assert "SSH_ASKPASS_REQUIRE=force" in backend
    assert "setsid -w ssh" in backend
    assert "Competition robot selected: local agi/aarch64 MC" in backend
    assert "--require-gripper-sdk" in backend
    assert 'install -m 0644 "$animation" "$temporary_animation"' in backend
    assert backend.index("primary_robot_address") < backend.index(
        "fallback_robot_address"
    )


def test_robot_askpass_uses_password_environment_without_prompt() -> None:
    environment = os.environ.copy()
    environment["GRASPV2_ROBOT_PASSWORD"] = "test-password"

    completed = subprocess.run(
        [str(ROOT / "tools" / "ssh_askpass.sh")],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert completed.returncode == 0
    assert completed.stdout == "test-password\n"
    assert completed.stderr == ""


def test_runtime_profile_can_be_pinned_without_host_probing() -> None:
    detector = ROOT / "tools" / "detect_runtime_profile.sh"
    for expected in ("competition", "test"):
        environment = os.environ.copy()
        environment["GRASPV2_RUNTIME_PROFILE"] = expected
        completed = subprocess.run(
            [str(detector)],
            cwd=ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
        assert completed.returncode == 0
        assert completed.stdout == expected + "\n"
