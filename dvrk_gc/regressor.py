from __future__ import annotations

from typing import Literal

import numpy as np


def analytical_bool_regressor_mat() -> np.ndarray:
    mat = np.zeros((7, 70), dtype=bool)
    mat[1, 0:2] = True
    mat[1:3, 2:4] = True
    mat[1:4, 4:6] = True
    mat[1:5, 6:8] = True
    mat[1:6, 8:10] = True

    mat[0, 10:15] = True
    mat[1, 15:20] = True
    mat[2, 20:25] = True
    mat[3, 25:30] = True
    mat[4, 30:35] = True
    mat[5, 35:40] = True

    mat[0, 40:45] = True
    mat[1, 45:50] = True
    mat[2, 50:55] = True
    mat[3, 55:60] = True
    mat[4, 60:65] = True
    mat[5, 65:70] = True
    return mat


def _poly5(x: float) -> list[float]:
    return [1.0, x, x * x, x * x * x, x * x * x * x]


def analytical_regressor_mat(g: float, q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(-1)
    if q.shape[0] < 6:
        raise ValueError("q must contain at least 6 joints")

    q1, q2, q3, q4, q5, q6 = q[:6]
    s2, c2 = np.sin(q2), np.cos(q2)
    s3, c3 = np.sin(q3), np.cos(q3)
    s4, c4 = np.sin(q4), np.cos(q4)
    s5, c5 = np.sin(q5), np.cos(q5)
    s6, c6 = np.sin(q6), np.cos(q6)
    s23 = np.sin(q2 + q3)

    r = np.zeros((7, 40), dtype=float)

    # gravity/coupling terms 1..10
    r[1, 0] = g * s2
    r[1, 1] = g * c2
    r[1:3, 2] = g * c2 * c3 - g * s2 * s3
    r[1:3, 3] = -g * c2 * s3 - g * c3 * s2
    r[1:4, 4] = g * c2 * c3 * c4 - g * c4 * s2 * s3
    r[1:4, 5] = g * s2 * s3 * s4 - g * c2 * c3 * s4
    r[1:5, 6] = g * c4 * s2 * s3 * s5 - g * c3 * c5 * s2 - g * c2 * c3 * c4 * s5 - g * c2 * c5 * s3
    r[1:5, 7] = g * c2 * c3 * c4 * c5 - g * c3 * s2 * s5 - g * c2 * s3 * s5 - g * c4 * c5 * s2 * s3
    r[1:6, 8] = (
        g * c2 * c3 * s4 * s6
        + g * c2 * c6 * s3 * s5
        + g * c3 * c6 * s2 * s5
        - g * s2 * s3 * s4 * s6
        + g * c4 * c5 * c6 * s2 * s3
        - g * c2 * c3 * c4 * c5 * c6
    )
    r[1:6, 9] = (
        g * c2 * c3 * c6 * s4
        - g * c6 * s2 * s3 * s4
        - g * c2 * s3 * s5 * s6
        - g * c3 * s2 * s5 * s6
        - g * c4 * c5 * s2 * s3 * s6
        + g * c2 * c3 * c4 * c5 * s6
    )

    # Joint 4 row has simpler closed forms for gravity terms 5..10
    r[3, 4] = -g * s23 * s4
    r[3, 5] = -g * s23 * c4
    r[3, 6] = g * s23 * s4 * s5
    r[3, 7] = -g * s23 * c5 * s4
    r[3, 8] = g * s23 * (c4 * s6 + c5 * c6 * s4)
    r[3, 9] = g * s23 * (c4 * c6 - c5 * s4 * s6)

    # Joint 5 row closed forms for terms 7..10
    r[4, 6] = -g * (c2 * c3 * s5 - s2 * s3 * s5 + c2 * c4 * c5 * s3 + c3 * c4 * c5 * s2)
    r[4, 7] = -g * (c5 * s2 * s3 - c2 * c3 * c5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5)
    r[4, 8] = g * (c5 * c6 * s2 * s3 - c2 * c3 * c5 * c6 + c2 * c4 * c6 * s3 * s5 + c3 * c4 * c6 * s2 * s5)
    r[4, 9] = -g * (c5 * s2 * s3 * s6 - c2 * c3 * c5 * s6 + c2 * c4 * s3 * s5 * s6 + c3 * c4 * s2 * s5 * s6)

    # Joint 6 row closed forms for terms 9..10
    r[5, 8] = g * (c2 * c6 * s3 * s4 + c3 * c6 * s2 * s4 + c2 * c3 * s5 * s6 - s2 * s3 * s5 * s6 + c2 * c4 * c5 * s3 * s6 + c3 * c4 * c5 * s2 * s6)
    r[5, 9] = g * (c2 * c3 * c6 * s5 - c2 * s3 * s4 * s6 - c3 * s2 * s4 * s6 - c6 * s2 * s3 * s5 + c2 * c4 * c5 * c6 * s3 + c3 * c4 * c5 * c6 * s2)

    # polynomial blocks 11..40
    r[0, 10:15] = _poly5(q1)
    r[1, 15:20] = _poly5(q2)
    r[2, 20:25] = _poly5(q3)
    r[3, 25:30] = _poly5(q4)
    r[4, 30:35] = _poly5(q5)
    r[5, 35:40] = _poly5(q6)

    return r


def analytical_regressor_mat_dual_dir(direction: Literal["pos", "neg"], g: float, q: np.ndarray) -> np.ndarray:
    base = analytical_regressor_mat(g, q)
    out = np.zeros((7, 70), dtype=float)
    out[:, 0:10] = base[:, 0:10]
    if direction == "pos":
        out[:, 10:40] = base[:, 10:40]
        return out
    if direction == "neg":
        out[:, 40:70] = base[:, 10:40]
        return out
    raise ValueError("direction must be 'pos' or 'neg'")
