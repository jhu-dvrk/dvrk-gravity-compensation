#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import pwd
import select
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dvrk_gc.config import save_json


def _require_dvrk():
    try:
        import dvrk  # type: ignore
        import crtk  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("dvrk_python import failed. Ensure dvrk_python is installed and sourced.") from exc
    return dvrk, crtk


def _current_unix_user_id() -> str:
    return pwd.getpwuid(os.getuid()).pw_name


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phase 1: explore joint free-space and save workspace ranges"
    )
    parser.add_argument("-a", "--arm", required=True, choices=["MTML", "MTMR"], help="Arm name")
    parser.add_argument("-s", "--serial", required=True, help="Serial number")
    return parser.parse_args()


def _is_keypress_pending():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def main() -> int:
    args = _parse_args()

    print("Connecting to dVRK...")
    dvrk, crtk = _require_dvrk()
    ral = crtk.ral("gc_setup_workspace")
    arm = dvrk.mtm(ral, args.arm)
    ral.spin()

    print("Checking connections...")
    try:
        ral.check_connections(5.0)
    except TimeoutError as exc:
        print(f"Failed to connect to MTM:\n{exc}")
        ral.shutdown()
        return 1

    # Wait for valid timestamp to ensure topics are active
    print("Waiting for valid state from arm...")
    start_t = time.time()
    ts = 0
    while ts == 0 and not ral.is_shutdown() and (time.time() - start_t) < 5.0:
        try:
            _, ts = arm.setpoint_jp()
        except TimeoutError:
            ts = 0
        if ts == 0:
            time.sleep(0.1)

    if ts == 0:
        print("Timed out waiting for valid data from arm")
        ral.shutdown()
        return 1

    print("Enabling arm...")
    if not arm.enable(10.0):
        print("Failed to enable arm")
        ral.shutdown()
        return 1
    print("Homing arm...")
    if not arm.home(30.0):
        print("Failed to home arm")
        ral.shutdown()
        return 1

    print("Moving to zero joint position")
    q_zero = np.zeros(7)
    arm.move_jp(q_zero).wait()

    q_measured, _ = arm.measured_jp()
    joint_count = 6 # We only need to explore joints 1-6
    print("\nWorkspace exploration mode")
    print("- For each joint i: move it manually to explore range.")
    print("- All other joints are locked at their setpoints.")
    print("- Press any key to move to the next joint.\n")

    ranges: dict[str, dict[str, float]] = {}
    started_at = datetime.now().isoformat()
    total_start_t = time.time()

    # Save terminal settings for non-blocking input
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        for joint_no in range(1, joint_count + 1):
            joint_index = joint_no - 1
            
            # Define exploration setpoints based on the joint being explored
            q_exploration_setpoint = np.zeros(7)
            if joint_index == 1 or joint_index == 2:
                q_exploration_setpoint[4] = np.radians(90.0)
            
            print(f"Moving to exploration setup for joint {joint_no}...")
            arm.move_jp(q_exploration_setpoint).wait()

            print(f"--> Exploring joint {joint_no}/{joint_count}. Move it now. Press any key to lock and continue...")
            
            min_rad = float("inf")
            max_rad = float("-inf")
            
            rate = ral.create_rate(500.0)
            while not ral.is_shutdown():
                if _is_keypress_pending():
                    sys.stdin.read(1) # Consume the key
                    break
                
                measured, _ = arm.measured_jp()
                
                # Command target joint to follow measured, others stay at exploration setpoint
                q_cmd = q_exploration_setpoint.copy()
                q_cmd[joint_index] = measured[joint_index]
                arm.servo_jp(q_cmd)

                val = float(measured[joint_index])
                min_rad = min(min_rad, val)
                max_rad = max(max_rad, val)
                
                rate.sleep()

            min_deg = float(np.degrees(min_rad))
            max_deg = float(np.degrees(max_rad))
            ranges[f"joint{joint_no}"] = {
                "min_deg": min_deg,
                "max_deg": max_deg,
            }
            print(f"    Joint {joint_no} range: [{min_deg:.2f}, {max_deg:.2f}] deg")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    payload = {
        "userId": _current_unix_user_id(),
        "arm": args.arm,
        "serialNumber": args.serial,
        "startedAt": started_at,
        "collectedAt": datetime.now().isoformat(),
        "durationSec": float(time.time() - total_start_t),
        "rateHz": 500.0,
        "jointRanges": ranges,
    }

    out_dir = Path(".").resolve()
    # Create a sub-directory based on the start time
    timestamp = datetime.fromisoformat(started_at).strftime("%Y-%m-%d_%H-%M-%S")
    out_dir = out_dir / timestamp
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "workspace.json"
    save_json(out_path, payload)

    print(f"Saved workspace ranges: {out_path}")
    print("Returning arm to zero position...")
    arm.move_jp(q_zero).wait()
    if ral is not None:
        ral.shutdown()

    print("Workspace setup complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
