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

from dvrk_gc.config import load_json
from dvrk_gc.controller import GCControllerConfig, GravityCompController
from dvrk_gc.arm_utils import create_arm_client, sample_arm


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


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 5: Manual test of gravity compensation")
    parser.add_argument("-d", "--data-dir", required=True, help="Directory containing workspace.json and gc-*.json")
    args = parser.parse_args()

    data_root = Path(args.data_dir).resolve()
    arm_name, serial_number = _resolve_arm_and_serial(data_root)

    gc_file = data_root / f"gc-{arm_name}-{serial_number}.json"
    if not gc_file.exists():
        print(f"Error: {gc_file} not found. Run identify_parameters.py first.")
        return 1

    cfg, controller = _build_controller(gc_file)
    rate = float(cfg["GC_Test"]["ONLINE_GC_DRT"]["rate"])

    arm, ral = create_arm_client("gc_test", arm_name)

    print("\nStarting manual test.")
    print("Commands:")
    print("  'f' to FREE the arm (gravity compensation ENABLED)")
    print("  'h' to HOLD the arm (gravity compensation DISABLED)")
    print("  'q' to QUIT")

    import select
    
    period = 1.0 / max(rate, 1.0)
    is_free = False
    
    try:
        while True:
            # Check for user input without blocking
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip().lower()
                if line == 'f':
                    print("\nFREEing arm - gravity compensation ON", flush=True)
                    arm.free()
                    is_free = True
                elif line == 'h':
                    print("\nHOLDing arm - gravity compensation OFF", flush=True)
                    arm.hold()
                    is_free = False
                elif line == 'q':
                    break
            
            if is_free:
                q, qd = sample_arm(arm)
                tau = controller.compute_torque(q=q, qd=qd)
                arm.servo_jf(tau)
            
            time.sleep(period)
            
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down...", flush=True)
        arm.hold()
        if ral:
            ral.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
