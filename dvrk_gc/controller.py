from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .regressor import analytical_regressor_mat


@dataclass
class GCControllerConfig:
    dynamic_params_pos: np.ndarray
    dynamic_params_neg: np.ndarray
    safe_upper_torque_limit: np.ndarray
    safe_lower_torque_limit: np.ndarray
    db_vel_vec: np.ndarray
    sat_vec_vec: np.ndarray
    fric_comp_ratio_vec: np.ndarray


class GravityCompController:
    def __init__(self, config: GCControllerConfig, g_constant: float) -> None:
        self.config = config
        self.g_constant = float(g_constant)

    def _blend_alpha(self, vel: np.ndarray) -> np.ndarray:
        v = np.abs(vel)
        db = self.config.db_vel_vec
        sat = self.config.sat_vec_vec
        ratio = self.config.fric_comp_ratio_vec

        alpha = np.full_like(v, 0.5, dtype=float)

        high = v >= sat
        alpha[high] = 0.5 + 0.5 * ratio[high]

        mid = (v > db) & (v < sat)
        alpha[mid] = 0.5 + 0.5 * ratio[mid] * ((v[mid] - db[mid]) / np.maximum(sat[mid] - db[mid], 1e-9))
        return alpha

    def compute_torque(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        reg = analytical_regressor_mat(self.g_constant, q)
        tau_pos = reg @ self.config.dynamic_params_pos.reshape(-1, 1)
        tau_neg = reg @ self.config.dynamic_params_neg.reshape(-1, 1)

        alpha = self._blend_alpha(np.asarray(qd, dtype=float).reshape(-1))
        tau = alpha.reshape(-1, 1) * tau_pos + (1.0 - alpha.reshape(-1, 1)) * tau_neg

        tau = np.clip(
            tau.reshape(-1),
            self.config.safe_lower_torque_limit.reshape(-1),
            self.config.safe_upper_torque_limit.reshape(-1),
        )
        return tau
