#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

import numpy as np

from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dvrk_gc.collection import build_collection_plan, build_joint_trajectory
from dvrk_gc.config import load_json, save_json


def _require_dvrk():
    try:
        import dvrk  # type: ignore
        import crtk  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("dvrk_python import failed. Ensure dvrk_python is installed and sourced.") from exc
    return dvrk, crtk


def _create_arm_client(arm_name: str):
    print("[progress] Loading dvrk_python modules", flush=True)
    dvrk, crtk = _require_dvrk()

    print("[progress] Creating CRTK RAL node", flush=True)
    ral = crtk.ral("gc_collect_data")
    print(f"[progress] Creating MTM client for arm '{arm_name}'", flush=True)
    arm = dvrk.mtm(ral, arm_name)
    print("[progress] MTM client created", flush=True)
    return arm, ral


def _package_data_collection_template() -> Path:
    return Path(get_package_share_directory("dvrk_mtm_gc")) / "config" / "data_collection.json"


def _apply_workspace_ranges(config: dict, workspace: dict) -> dict:
    arm_name = str(workspace["arm"])
    data_collection = config["data_collection"]
    joint_ranges = workspace["jointRanges"]

    config["ARM_NAME"] = arm_name
    config["workspace"] = workspace

    for joint_name, limits in joint_ranges.items():
        if joint_name not in data_collection:
            continue

        block = data_collection[joint_name]
        min_deg = float(limits["min_deg"])
        max_deg = float(limits["max_deg"])

        if isinstance(block.get("train_angle_min"), dict):
            block["train_angle_min"][arm_name] = min_deg
        else:
            block["train_angle_min"] = min_deg

        if isinstance(block.get("train_angle_max"), dict):
            block["train_angle_max"][arm_name] = max_deg
        else:
            block["train_angle_max"] = max_deg

    return config


def _sample_arm(arm) -> tuple[np.ndarray, np.ndarray]:
    setpoint = arm.setpoint_js()
    measured = arm.measured_js()
    effort = np.asarray(setpoint[2], dtype=float).reshape(7)
    position = np.asarray(measured[0], dtype=float).reshape(7)
    return position, effort


def _collect_one_theta(arm, trajectory: np.ndarray, sample_num: int, steady_time: float) -> tuple[np.ndarray, np.ndarray]:
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
    info_path = root / "dataCollection_info.json"
    save_json(info_path, cfg)

    plan = build_collection_plan(cfg, root)

    total_blocks = 0
    for job in plan.jobs:
        directions = int(job.is_pos_dir) + int(job.is_neg_dir)
        total_blocks += len(job.theta_angle_list_rad) * max(job.repeat_times, 1) * directions

    print(f"Collection root: {root}", flush=True)
    print(f"Planned collection blocks: {total_blocks}", flush=True)

    print("[progress] Initializing robot connection", flush=True)
    arm, ral = _create_arm_client(arm_name)

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
    for job in plan.jobs:
        dir_specs: list[tuple[str, list[float], Path]] = []
        if job.is_pos_dir:
            dir_specs.append(("pos", job.train_angle_list_rad, job.pos_data_path))
        if job.is_neg_dir:
            dir_specs.append(("neg", list(reversed(job.train_angle_list_rad)), job.neg_data_path))

        for direction, train_vals, out_dir in dir_specs:
            for rep in range(max(job.repeat_times, 1)):
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
                    print(
                        f"Joint {job.train_joint_no}, dir={direction}, theta={theta_deg} deg, "
                        f"rep={rep+1}, samples={trajectory.shape[1]} ({done + 1}/{total_blocks})"
                    , flush=True)

                    current_position, desired_effort = _collect_one_theta(
                        arm=arm,
                        trajectory=trajectory,
                        sample_num=job.sample_num,
                        steady_time=job.steady_time,
                    )
                    _save_block(out_dir, theta_deg, trajectory, current_position, desired_effort)
                    done += 1

    print("[progress] Returning arm to home pose", flush=True)
    arm.move_jp(np.zeros(7)).wait()
    print("[progress] Final home pose reached", flush=True)
    if ral is not None:
        print("[progress] Shutting down CRTK RAL", flush=True)
        ral.shutdown()
    print(f"Collection complete. Info file: {info_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
