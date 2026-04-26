#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dvrk_gc.config import load_json, save_json
from dvrk_gc.controller import GCControllerConfig, GravityCompController
from dvrk_gc.arm_utils import create_arm_client, sample_arm


def _build_controller(gc_file: Path) -> tuple[dict, GravityCompController]:
    cfg = load_json(gc_file)
    gc = cfg["GC_controller"]
    lse_cfg = cfg["lse"]
    controller_cfg = GCControllerConfig(
        dynamic_params_pos=np.asarray(gc["gc_dynamic_params_pos"], dtype=float),
        dynamic_params_neg=np.asarray(gc["gc_dynamic_params_neg"], dtype=float),
        safe_upper_torque_limit=np.asarray(gc["safe_upper_torque_limit"], dtype=float),
        safe_lower_torque_limit=np.asarray(gc["safe_lower_torque_limit"], dtype=float),
        db_vel_vec=np.asarray(gc["db_vel_vec"], dtype=float),
        sat_vec_vec=np.asarray(gc["sat_vec_vec"], dtype=float),
        fric_comp_ratio_vec=np.asarray(gc["fric_comp_ratio_vec"], dtype=float),
    )
    controller = GravityCompController(controller_cfg, g_constant=float(lse_cfg["g_constant"]))
    return cfg, controller


def _resolve_arm_and_serial(data_dir: Path) -> tuple[str, str]:
    workspace_path = data_dir / "workspace.json"
    if not workspace_path.exists():
        print(f"Error: {workspace_path} not found. Run define_workspace.py first.")
        sys.exit(1)
    ws = load_json(workspace_path)
    arm_name = ws.get("arm")
    serial_number = ws.get("serialNumber")
    if not arm_name or not serial_number:
        print(f"Error: workspace.json is missing 'arm' or 'serialNumber'")
        sys.exit(1)
    return arm_name, str(serial_number)


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 4: run gravity compensation drift test")
    parser.add_argument("-d", "--data-dir", required=True, help="Directory containing workspace.json and gc-*.json")
    parser.add_argument("--duration", type=float, default=None, help="Override test duration (seconds)")
    parser.add_argument("--dry-run", action="store_true", help="Validate config and print expected behavior")
    args = parser.parse_args()

    data_root = Path(args.data_dir).resolve()
    arm_name, serial_number = _resolve_arm_and_serial(data_root)

    gc_file = data_root / f"gc-{arm_name}-{serial_number}.json"
    if not gc_file.exists():
        print(f"Error: {gc_file} not found. Run identify_parameters.py first.")
        return 1

    cfg, controller = _build_controller(gc_file)
    gc_test = cfg["GC_Test"]["ONLINE_GC_DRT"]
    duration = float(args.duration if args.duration is not None else gc_test["duration"])
    rate = float(gc_test["rate"])
    safe_vel_limit = np.asarray(gc_test["safe_vel_limit"], dtype=float)

    if args.dry_run:
        print(f"Controller loaded for {arm_name}")
        print(f"Duration={duration}s, rate={rate}Hz")
        return 0

    arm, ral = create_arm_client("gc_test_drift", arm_name)

    init_deg = np.asarray(cfg["GC_controller"]["GC_init_pos"], dtype=float)
    arm.move_jp(np.radians(init_deg)).wait()

    period = 1.0 / max(rate, 1.0)
    end_t = time.time() + duration
    max_abs_vel = np.zeros(7, dtype=float)

    try:
        print("Running online gravity compensation drift test")
        arm.free()
        while time.time() < end_t:
            remaining = max(0, int(end_t - time.time()))
            print(f"\rTime remaining: {remaining}s  ", end="", flush=True)

            q, qd = sample_arm(arm)
            max_abs_vel = np.maximum(max_abs_vel, np.abs(qd))

            if np.any(np.abs(qd) > safe_vel_limit):
                print("\nTest failed: velocity limit exceeded")
                arm.hold()
                return 2

            tau = controller.compute_torque(q=q, qd=qd)
            arm.servo_jf(tau)
            time.sleep(period)

        arm.hold()
        print("\nTest passed")
        print(f"Peak abs velocity: {max_abs_vel.tolist()}")
        return 0
    finally:
        if ral is not None:
            print("[progress] Shutting down CRTK RAL", flush=True)
            ral.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
