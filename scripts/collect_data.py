#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path
from typing import Callable

import numpy as np

from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dvrk_gc.collection import build_collection_plan, build_joint_trajectory
from dvrk_gc.config import load_json, save_json
from dvrk_gc.arm_utils import create_arm_client


def _package_data_collection_template() -> Path:
    return Path(get_package_share_directory("dvrk_mtm_gc")) / "config" / "data_collection.json"


def _workspace_serial_number(workspace: dict) -> str:
    value = workspace.get("serialNumber")
    if value is None:
        raise KeyError("workspace.json missing serial number (expected: serialNumber)")
    serial = str(value).strip()
    if not serial:
        raise ValueError("workspace.json serialNumber is empty")
    return serial


def _apply_workspace_ranges(config: dict, workspace: dict) -> dict:
    arm_name = str(workspace["arm"])
    serial_number = _workspace_serial_number(workspace)

    config["arm"] = arm_name
    config["serialNumber"] = serial_number
    config["workspace"] = workspace
    config.pop("ARM_NAME", None)
    config.pop("SN", None)

    return config


def _sample_arm(arm) -> tuple[np.ndarray, np.ndarray]:
    measured_pos, _ = arm.measured_jp()
    measured_effort, _ = arm.measured_jf()
    effort = measured_effort.reshape(7)
    position = measured_pos.reshape(7)
    return position, effort


def _format_duration(seconds: float) -> str:
    total_seconds = max(int(round(seconds)), 0)
    minutes, secs = divmod(total_seconds, 60)
    hours, minutes = divmod(minutes, 60)
    if hours > 0:
        return f"{hours:d}h{minutes:02d}m{secs:02d}s"
    if minutes > 0:
        return f"{minutes:d}m{secs:02d}s"
    return f"{secs:d}s"


def _count_total_target_points(plan) -> int:
    total_target_points = 0
    for job in plan.jobs:
        dir_specs: list[list[float]] = []
        if job.is_pos_dir:
            dir_specs.append(job.train_angle_list_rad)
        if job.is_neg_dir:
            dir_specs.append(list(reversed(job.train_angle_list_rad)))

        total_repeats = max(job.repeat_times, 1)
        for train_vals in dir_specs:
            for _ in range(total_repeats):
                for theta in job.theta_angle_list_rad:
                    trajectory = build_joint_trajectory(
                        init_joint_range_rad=job.init_joint_range_rad,
                        train_joint_no=job.train_joint_no,
                        theta_joint_no=job.theta_joint_no,
                        theta=theta,
                        train_vals=train_vals,
                    )
                    total_target_points += int(trajectory.shape[1])
    return total_target_points


def _collect_one_theta(
    arm,
    trajectory: np.ndarray,
    sample_num: int,
    steady_time: float,
    on_target_complete: Callable[[int, int], None] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    sample_size = trajectory.shape[1]
    desired_effort = np.zeros((7, sample_size, sample_num), dtype=float)
    current_position = np.zeros((7, sample_size, sample_num), dtype=float)

    for i in range(sample_size):
        if i == 0:
            print(f"[progress] Starting trajectory with {sample_size} target points", flush=True)
        # print(f"  Target point {i+1}: {trajectory[:, i]}", flush=True)
        handle = arm.move_jp(trajectory[:, i])
        if not handle.wait():
            print(f"[warning] Motion to point {i+1} timed out or failed", flush=True)
        time.sleep(steady_time)
        for j in range(sample_num):
            time.sleep(0.01)
            pos, eff = _sample_arm(arm)
            current_position[:, i, j] = pos
            desired_effort[:, i, j] = eff
        if on_target_complete is not None:
            on_target_complete(i + 1, sample_size)
    return current_position, desired_effort


def _save_block(path: Path, theta_deg: int, trajectory: np.ndarray, current_position: np.ndarray, desired_effort: np.ndarray) -> None:
    path.mkdir(parents=True, exist_ok=True)
    existing = len(list(path.glob(f"theta{theta_deg}-*.npz")))
    file_path = path / f"theta{theta_deg}-{existing:03d}.npz"
    np.savez(
        file_path,
        joint_trajectory=trajectory,
        current_position=current_position,
        desired_effort=desired_effort,
        Theta=theta_deg,
        current_date_time=file_path.stem,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 2: collect torque and position data")
    parser.add_argument("-d", "--dir", required=True, help="Data directory for workspace, config, and collected files")
    args = parser.parse_args()

    root = Path(args.dir).resolve()
    root.mkdir(parents=True, exist_ok=True)

    workspace_path = root / "workspace.json"
    if not workspace_path.exists():
        raise FileNotFoundError(f"Missing workspace file: {workspace_path}")

    config_path = root / "data_collection.json"
    if not config_path.exists():
        template_path = _package_data_collection_template()
        shutil.copyfile(template_path, config_path)
        print(f"[progress] Copied default config to {config_path}", flush=True)

    workspace_cfg = load_json(workspace_path)
    cfg = load_json(config_path)
    cfg = _apply_workspace_ranges(cfg, workspace_cfg)
    save_json(config_path, cfg)

    arm_name = str(workspace_cfg["arm"])

    plan = build_collection_plan(cfg, root)

    total_blocks = 0
    for job in plan.jobs:
        directions = int(job.is_pos_dir) + int(job.is_neg_dir)
        total_blocks += len(job.theta_angle_list_rad) * max(job.repeat_times, 1) * directions
    total_target_points = _count_total_target_points(plan)

    print(f"Collection root: {root}", flush=True)
    print(f"Planned collection blocks: {total_blocks}", flush=True)
    print(f"Planned target points: {total_target_points}", flush=True)

    print("[progress] Initializing robot connection", flush=True)
    arm, ral = create_arm_client("gc_collect_data", arm_name)

    print(f"[progress] Checking RAL-level ROS topic connections for '{arm_name}'", flush=True)
    # Give the RAL node a moment to spin/initialize and discover topics
    ral.spin()
    ral.check_connections()
    print("[progress] RAL connections established", flush=True)

    print("[progress] Enabling arm", flush=True)
    if not arm.enable(10.0):
        raise RuntimeError("Failed to enable arm within timeout")
    print("[progress] Arm enabled", flush=True)

    print("[progress] Homing arm", flush=True)
    if not arm.home(30.0):
        raise RuntimeError("Failed to home arm within timeout")
    print("[progress] Arm homed", flush=True)

    print("[progress] Sending arm to home pose [0,0,0,0,0,0,0] rad", flush=True)
    arm.move_jp(np.zeros(7)).wait()
    print("[progress] Home pose reached; data collection starts", flush=True)

    done = 0
    completed_target_points = 0
    collection_start_t = time.time()
    for job in plan.jobs:
        dir_specs: list[tuple[str, list[float], Path]] = []
        if job.is_pos_dir:
            dir_specs.append(("pos", job.train_angle_list_rad, job.pos_data_path))
        if job.is_neg_dir:
            dir_specs.append(("neg", list(reversed(job.train_angle_list_rad)), job.neg_data_path))

        for direction, train_vals, out_dir in dir_specs:
            total_repeats = max(job.repeat_times, 1)
            for rep in range(total_repeats):
                for theta in job.theta_angle_list_rad:
                    trajectory = build_joint_trajectory(
                        init_joint_range_rad=job.init_joint_range_rad,
                        train_joint_no=job.train_joint_no,
                        theta_joint_no=job.theta_joint_no,
                        theta=theta,
                        train_vals=train_vals,
                    )
                    if trajectory.shape[1] == 0:
                        continue

                    theta_deg = int(round(np.degrees(theta)))
                    if completed_target_points > 0:
                        elapsed = time.time() - collection_start_t
                        remaining_points = total_target_points - completed_target_points
                        remaining_seconds = elapsed * remaining_points / completed_target_points
                        eta_timestamp = time.time() + remaining_seconds
                        eta_clock = time.strftime("%H:%M:%S", time.localtime(eta_timestamp))
                        eta_text = f"{eta_clock} ({_format_duration(remaining_seconds)}) remaining"
                    else:
                        eta_text = "estimating"

                    print(
                        f"Joint {job.train_joint_no}, dir={direction}, theta={theta_deg} deg, "
                        f"rep={rep+1}/{total_repeats}, samples={trajectory.shape[1]}, "
                        f"eta={eta_text} ({done + 1}/{total_blocks})"
                    , flush=True)

                    def _on_target_complete(block_point_index: int, block_point_count: int) -> None:
                        nonlocal completed_target_points
                        completed_target_points += 1

                    current_position, desired_effort = _collect_one_theta(
                        arm=arm,
                        trajectory=trajectory,
                        sample_num=job.sample_num,
                        steady_time=job.steady_time,
                        on_target_complete=_on_target_complete,
                    )
                    _save_block(out_dir, theta_deg, trajectory, current_position, desired_effort)
                    done += 1

    print("[progress] Returning arm to home pose", flush=True)
    arm.move_jp(np.zeros(7)).wait()
    print("[progress] Final home pose reached", flush=True)
    if ral is not None:
        print("[progress] Shutting down CRTK RAL", flush=True)
        ral.shutdown()
    print(f"Collection complete. Config file: {config_path}", flush=True)
    print(f"Next step: ros2 run dvrk_mtm_gc identify_parameters -d {Path(config_path).parent}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
