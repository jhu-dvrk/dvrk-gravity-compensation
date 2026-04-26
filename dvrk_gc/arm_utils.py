from __future__ import annotations
import sys
import numpy as np
from pathlib import Path

def require_dvrk():
    try:
        import dvrk  # type: ignore
        import crtk  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("dvrk_python import failed. Ensure dvrk_python is installed and sourced.") from exc
    return dvrk, crtk

def create_arm_client(node_name: str, arm_name: str):
    print(f"[progress] Loading dvrk_python modules for {node_name}", flush=True)
    dvrk, crtk = require_dvrk()

    print(f"[progress] Creating CRTK RAL node: {node_name}", flush=True)
    ral = crtk.ral(node_name)
    print(f"[progress] Creating MTM client for arm '{arm_name}'", flush=True)
    arm = dvrk.mtm(ral, arm_name)
    
    # Wait for the arm to be ready and have data
    print("[progress] Waiting for arm state...", flush=True)
    
    # Give the RAL node a moment to spin/initialize and discover topics
    ral.spin()

    start_time = 0
    q = np.array([])
    while start_time < 50: # 5 second timeout (100ms * 50)
        try:
            q, _ = arm.measured_jp()
            if q.size > 0:
                break
        except Exception:
            pass
        import time
        time.sleep(0.1)
        start_time += 1
    
    if q.size == 0:
        print("[error] Failed to get data from arm. Is the dVRK console running?")
        sys.exit(1)

    print("[progress] MTM client created and ready", flush=True)
    return arm, ral

def sample_arm(arm):
    """Utility to get jp and jv as flat numpy arrays."""
    q, _ = arm.measured_jp()
    qd, _ = arm.measured_jv()
    return q.reshape(7), qd.reshape(7)
