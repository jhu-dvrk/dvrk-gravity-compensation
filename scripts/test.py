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


def _require_dvrk():
    try:
        import dvrk  # type: ignore
        import crtk  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("dvrk_python import failed. Ensure dvrk_python is installed and sourced.") from exc
    return dvrk, crtk


def _create_arm_client(arm_name: str):
    dvrk, crtk = _require_dvrk()

    # Newer dvrk_python APIs require a CRTK RAL object.
    ral = crtk.ral("gc_test", namespace="/dvrk")
    try:
        arm = dvrk.mtm(ral, arm_name)
        return arm, ral
    except TypeError:
        # Backward compatibility for older one-argument constructors.
        arm = dvrk.mtm(arm_name)
        return arm, None


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


def _sample(arm):
    measured = arm.measured_js()
    q = np.asarray(measured[0], dtype=float).reshape(7)
    qd = np.asarray(measured[1], dtype=float).reshape(7)
    return q, qd


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 4: run gravity compensation drift test")
    parser.add_argument("--gc-file", required=True, help="Path to gc-ARM-SN.json")
    parser.add_argument("--arm", required=True, choices=["MTML", "MTMR"], help="Arm name")
    parser.add_argument("--duration", type=float, default=None, help="Override test duration (seconds)")
    parser.add_argument("--dry-run", action="store_true", help="Validate config and print expected behavior")
    args = parser.parse_args()

    cfg, controller = _build_controller(Path(args.gc_file))
    gc_test = cfg["GC_Test"]["ONLINE_GC_DRT"]
    duration = float(args.duration if args.duration is not None else gc_test["duration"])
    rate = float(gc_test["rate"])
    safe_vel_limit = np.asarray(gc_test["safe_vel_limit"], dtype=float)

    if args.dry_run:
        print(f"Controller loaded for {args.arm}")
        print(f"Duration={duration}s, rate={rate}Hz")
        return 0

    arm, ral = _create_arm_client(args.arm)

    init_deg = np.asarray(cfg["GC_controller"]["GC_init_pos"], dtype=float)
    arm.move_jp(np.radians(init_deg)).wait()

    period = 1.0 / max(rate, 1.0)
    end_t = time.time() + duration
    max_abs_vel = np.zeros(7, dtype=float)

    print("Running online gravity compensation drift test")
    while time.time() < end_t:
        q, qd = _sample(arm)
        max_abs_vel = np.maximum(max_abs_vel, np.abs(qd))

        if np.any(np.abs(qd) > safe_vel_limit):
            print("Test failed: velocity limit exceeded")
            return 2

        tau = controller.compute_torque(q=q, qd=qd)
        arm.servo_jf(tau)
        time.sleep(period)

    if ral is not None:
        ral.shutdown()

    print("Test passed")
    print(f"Peak abs velocity: {max_abs_vel.tolist()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
