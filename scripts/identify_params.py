#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dvrk_gc.config import load_json, save_json
from dvrk_gc.lse import lse
from dvrk_gc.regressor import analytical_regressor_mat_dual_dir


def _list_data_files(path: Path) -> list[Path]:
    return sorted(path.glob("*.npz"))


def _torques_data_process(current_position: np.ndarray, desired_effort: np.ndarray, std_filter: float) -> np.ndarray:
    d_size = desired_effort.shape
    out = np.zeros((7, 2, d_size[1]), dtype=float)

    for i in range(d_size[1]):
        for j in range(d_size[0]):
            effort_data = desired_effort[j, i, :]
            position_data = current_position[j, i, :]
            effort_std = float(np.std(effort_data))
            effort_mean = float(np.mean(effort_data))
            effort_std = max(effort_std, 1e-4)
            lo = effort_mean - effort_std * std_filter
            hi = effort_mean + effort_std * std_filter
            select = (effort_data >= lo) & (effort_data <= hi)
            if not np.any(select):
                out[j, 0, i] = float(np.mean(position_data))
                out[j, 1, i] = float(np.mean(effort_data))
            else:
                out[j, 0, i] = float(np.mean(position_data[select]))
                out[j, 1, i] = float(np.mean(effort_data[select]))
    return out


def _data2augmat(torques_data: np.ndarray, joint_no: int, direction: str, g_constant: float) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    torques = []
    for k in range(torques_data.shape[2]):
        q = torques_data[:, 0, k]
        tau = torques_data[joint_no - 1, 1, k]
        reg = analytical_regressor_mat_dual_dir(direction, g_constant, q)
        rows.append(reg[joint_no - 1, :])
        torques.append(tau)
    if not rows:
        return np.zeros((0, 70), dtype=float), np.zeros((0, 1), dtype=float)
    return np.vstack(rows), np.asarray(torques, dtype=float).reshape(-1, 1)


def _fit_method_priors(joint_no: int, fit_method: str) -> tuple[list[int], list[float]]:
    # Matches config_lse.m: zero out higher-order terms depending on fit method.
    base = 10 + 5 * joint_no
    mapping = {
        "4POL": [0],
        "3POL": [0, base, base + 30],
        "2POL": [0, base - 1, base, base + 29, base + 30],
        "1POL": [0, base - 2, base - 1, base, base + 28, base + 29, base + 30],
        "drift": [0, base - 3, base - 2, base - 1, base, base + 27, base + 28, base + 29, base + 30],
        "origin": [0, base - 4, base - 3, base - 2, base - 1, base, base + 26, base + 27, base + 28, base + 29, base + 30],
    }
    idx = mapping.get(fit_method, [0])
    idx = [i for i in idx if i > 0]
    return idx, [0.0] * len(idx)


def _load_joint_torques(data_root: Path, joint_no: int, std_filter: float) -> tuple[np.ndarray, np.ndarray]:
    pos_files = _list_data_files(data_root / f"Train_Joint{joint_no}" / "data_pos")
    neg_files = _list_data_files(data_root / f"Train_Joint{joint_no}" / "data_neg")

    pos_blocks = []
    for f in pos_files:
        data = np.load(f, allow_pickle=True)
        pos_blocks.append(_torques_data_process(data["current_position"], data["desired_effort"], std_filter))

    neg_blocks = []
    for f in neg_files:
        data = np.load(f, allow_pickle=True)
        neg_blocks.append(_torques_data_process(data["current_position"], data["desired_effort"], std_filter))

    pos = np.concatenate(pos_blocks, axis=2) if pos_blocks else np.zeros((7, 2, 0), dtype=float)
    neg = np.concatenate(neg_blocks, axis=2) if neg_blocks else np.zeros((7, 2, 0), dtype=float)
    return pos, neg


def _train_joint(
    data_root: Path,
    joint_no: int,
    lse_cfg: dict,
    old_param_map: dict[int, float],
    old_param_rel_std_map: dict[int, float],
) -> tuple[np.ndarray, dict[int, float], dict[int, float]]:
    fit_method = lse_cfg[f"joint{joint_no}"]["fit_method"]
    std_filter = float(lse_cfg["std_filter"])
    g_constant = float(lse_cfg["g_constant"])

    pos_data, neg_data = _load_joint_torques(data_root, joint_no, std_filter)
    r_pos, t_pos = _data2augmat(pos_data, joint_no, "pos", g_constant)
    r_neg, t_neg = _data2augmat(neg_data, joint_no, "neg", g_constant)
    r_aug = np.vstack([r_pos, r_neg])
    t_aug = np.vstack([t_pos, t_neg])

    prior_idx, prior_vals = _fit_method_priors(joint_no, fit_method)
    prior_map = {k: v for k, v in zip(prior_idx, prior_vals)}

    # Old values override fit priors, same behavior as MATLAB pipeline.
    for k in old_param_map:
        prior_map.pop(k, None)

    input_param_map = dict(old_param_map)
    input_param_map.update(prior_map)

    output_param_map, output_param_full_map, output_param_rel_std_map = lse(
        input_param_map=input_param_map,
        input_param_rel_std_map=old_param_rel_std_map,
        r2_augmented=r_aug,
        t2_augmented=t_aug,
        train_joint_no_list=[joint_no],
        output_param_joint_no_list=list(range(joint_no, 8)),
    )

    dynamic_parameters = np.array([output_param_full_map[i] for i in sorted(output_param_full_map)], dtype=float).reshape(-1, 1)
    return dynamic_parameters, output_param_map, output_param_rel_std_map


def _training_data_torque_limits(all_torques: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if all_torques.size == 0:
        return np.full(7, 0.2), np.full(7, -0.2)

    effort = all_torques[:, 1, :]
    upper = np.quantile(effort, 0.95, axis=1)
    lower = np.quantile(effort, 0.05, axis=1)
    return upper, lower


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 3: identify gravity compensation parameters")
    parser.add_argument("--data-info", required=True, help="Path to dataCollection_info.json")
    parser.add_argument("--mlse-config", default="mlse_config.json")
    parser.add_argument("--gc-controller-config", default="gc_controller_config.json")
    parser.add_argument("--gc-test-config", default="gc_test_config.json")
    args = parser.parse_args()

    data_info_path = Path(args.data_info).resolve()
    data_root = data_info_path.parent

    config = load_json(data_info_path)
    config["lse"] = load_json(args.mlse_config)["lse"]
    config["GC_controller"] = load_json(args.gc_controller_config)["GC_controller"]
    config["GC_Test"] = load_json(args.gc_test_config)["GC_Test"]

    old_param_map: dict[int, float] = {}
    old_param_rel_std_map: dict[int, float] = {}
    output_dynamic_matrix = np.zeros((70, 1), dtype=float)
    torques_acc = []

    for joint_no in [6, 5, 4, 3, 2, 1]:
        dynamic_parameters, old_param_map, old_param_rel_std_map = _train_joint(
            data_root=data_root,
            joint_no=joint_no,
            lse_cfg=config["lse"],
            old_param_map=old_param_map,
            old_param_rel_std_map=old_param_rel_std_map,
        )
        output_dynamic_matrix = dynamic_parameters

        pos_data, neg_data = _load_joint_torques(data_root, joint_no, float(config["lse"]["std_filter"]))
        if pos_data.size:
            torques_acc.append(pos_data)
        if neg_data.size:
            torques_acc.append(neg_data)

    all_torques = np.concatenate(torques_acc, axis=2) if torques_acc else np.zeros((7, 2, 0), dtype=float)
    torque_upper, torque_lower = _training_data_torque_limits(all_torques)

    config.pop("data_collection", None)
    config["version"] = "2.0"
    config["GC_controller"]["safe_upper_torque_limit"] = np.round(torque_upper, 5).tolist()
    config["GC_controller"]["safe_lower_torque_limit"] = np.round(torque_lower, 5).tolist()
    config["GC_controller"]["gc_dynamic_params_pos"] = output_dynamic_matrix[0:40, 0].tolist()
    config["GC_controller"]["gc_dynamic_params_neg"] = np.concatenate([output_dynamic_matrix[0:10, 0], output_dynamic_matrix[40:70, 0]]).tolist()

    output_file = data_root / f"gc-{config['ARM_NAME']}-{config['SN']}.json"
    save_json(output_file, config)
    print(f"Saved GC parameter file: {output_file}")
    print("Next: run scripts/gc_test.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
