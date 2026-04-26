from __future__ import annotations

from typing import Iterable

import numpy as np

from .regressor import analytical_bool_regressor_mat


def _std_dynamic_param(r2: np.ndarray, t2: np.ndarray, beta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if r2.shape[0] != t2.shape[0]:
        raise ValueError("Rows of regressor and torque vectors do not match")
    if t2.ndim != 2 or t2.shape[1] != 1:
        raise ValueError("t2 must be a column vector")
    if beta.ndim != 2 or beta.shape[1] != 1:
        raise ValueError("beta must be a column vector")
    if r2.shape[1] != beta.shape[0]:
        raise ValueError("Columns of regressor and rows of beta do not match")

    dof = max(r2.shape[0] - beta.shape[0], 1)
    var_e = (np.linalg.norm(t2 - r2 @ beta) ** 2) / dof
    cov = var_e * np.linalg.pinv(r2.T @ r2)
    std_var_beta = np.sqrt(np.diag(cov)).reshape(-1, 1)

    denom = np.linalg.norm(std_var_beta)
    if denom < 1e-12:
        rel_std = np.zeros_like(std_var_beta)
    else:
        rel_std = std_var_beta * 100.0 / denom
    return std_var_beta, rel_std


def lse(
    input_param_map: dict[int, float],
    input_param_rel_std_map: dict[int, float],
    r2_augmented: np.ndarray,
    t2_augmented: np.ndarray,
    train_joint_no_list: Iterable[int],
    output_param_joint_no_list: Iterable[int],
) -> tuple[dict[int, float], dict[int, float], dict[int, float]]:
    param_num = r2_augmented.shape[1]
    known_index = np.zeros(param_num, dtype=bool)
    known_parameter = np.zeros(param_num, dtype=float)

    for key, value in input_param_map.items():
        idx = int(key) - 1
        if idx < 0 or idx >= param_num:
            continue
        known_index[idx] = True
        known_parameter[idx] = float(value)

    bool_regressor = analytical_bool_regressor_mat()
    trained_bool_row = np.zeros(param_num, dtype=bool)
    for joint_no in train_joint_no_list:
        trained_bool_row |= bool_regressor[int(joint_no) - 1, :]

    t2 = t2_augmented.astype(float).copy().reshape(-1, 1)
    for i in range(param_num):
        if known_index[i]:
            t2 -= r2_augmented[:, [i]] * known_parameter[i]

    unknown_index = ~known_index
    solve_mask = unknown_index & trained_bool_row
    r2 = r2_augmented[:, solve_mask]
    beta = np.linalg.pinv(r2) @ t2

    _, rel_std = _std_dynamic_param(r2, t2, beta)

    dynamic_result = np.zeros((param_num, 3), dtype=float)
    k = 0
    for i in range(param_num):
        if known_index[i]:
            dynamic_result[i, 0] = 2.0
            dynamic_result[i, 1] = known_parameter[i]
        elif trained_bool_row[i]:
            dynamic_result[i, 0] = 1.0
            dynamic_result[i, 1] = beta[k, 0]
            dynamic_result[i, 2] = rel_std[k, 0]
            k += 1

    output_param_full_map = {i + 1: float(dynamic_result[i, 1]) for i in range(param_num)}
    output_param_map = dict(output_param_full_map)
    output_param_rel_std_map = {i + 1: float(dynamic_result[i, 2]) for i in range(param_num)}

    bool_joint_array = np.zeros(param_num, dtype=bool)
    for joint_no in output_param_joint_no_list:
        bool_joint_array |= bool_regressor[int(joint_no) - 1, :]

    remove_keys = [i + 1 for i in range(param_num) if not bool_joint_array[i]]
    for key in remove_keys:
        output_param_map.pop(key, None)
        output_param_rel_std_map.pop(key, None)

    for key, value in input_param_rel_std_map.items():
        output_param_rel_std_map[int(key)] = float(value)

    return output_param_map, output_param_full_map, output_param_rel_std_map
