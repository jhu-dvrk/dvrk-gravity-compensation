from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .config import arm_value, deg_list_to_rad, make_range_deg


@dataclass
class CollectJob:
    train_joint_no: int
    theta_joint_no: int
    train_angle_list_rad: list[float]
    theta_angle_list_rad: list[float]
    init_joint_range_rad: list[float]
    is_pos_dir: bool
    is_neg_dir: bool
    sample_num: int
    steady_time: float
    repeat_times: int
    pos_data_path: Path
    neg_data_path: Path


@dataclass
class CollectionPlan:
    arm_name: str
    output_root: Path
    jobs: list[CollectJob]


def _resolve_joint_plan(data_collection: dict, arm_name: str, root_path: Path) -> list[CollectJob]:
    is_pos_dir = bool(data_collection["is_pos_dir"])
    is_neg_dir = bool(data_collection["is_neg_dir"])
    sample_num = int(data_collection["sample_num"])
    steady_time = float(data_collection["steady_time"])

    jobs: list[CollectJob] = []

    for joint_no in [1, 2, 3, 4, 5, 6]:
        block = data_collection[f"joint{joint_no}"]
        train_joint_no = int(block["Train_Joint_No"])
        theta_joint_no = int(block.get("Theta_Joint_No", 1))

        train_min = arm_value(block["train_angle_min"], arm_name)
        train_max = arm_value(block["train_angle_max"], arm_name)
        train_step = float(block["train_angle_delta"])
        train_angle_list_deg = make_range_deg(float(train_min), float(train_max), train_step)

        if train_joint_no in (1, 2):
            theta_angle_list_deg = [0.0]
        else:
            theta_min = float(block["theta_angle_min"])
            theta_max = float(block["theta_angle_max"])
            theta_step = float(block["theta_angle_delta"])
            theta_angle_list_deg = make_range_deg(theta_min, theta_max, theta_step)

        if train_joint_no == 2 and "joint3" in data_collection:
            joint3 = data_collection["joint3"]
            couple_upper = float(joint3["couple_upper_limit"])
            couple_lower = float(joint3["couple_lower_limit"])
            filtered: list[float] = []
            for t in train_angle_list_deg:
                total = t + theta_angle_list_deg[0]
                if couple_lower <= total <= couple_upper:
                    filtered.append(t)
            train_angle_list_deg = filtered

        init_joint_range_deg = arm_value(block["init_joint_range"], arm_name)
        repeat_times = int(block.get("repeat_times", 1))

        jobs.append(
            CollectJob(
                train_joint_no=train_joint_no,
                theta_joint_no=theta_joint_no,
                train_angle_list_rad=deg_list_to_rad(train_angle_list_deg),
                theta_angle_list_rad=deg_list_to_rad(theta_angle_list_deg),
                init_joint_range_rad=deg_list_to_rad([float(v) for v in init_joint_range_deg]),
                is_pos_dir=is_pos_dir,
                is_neg_dir=is_neg_dir,
                sample_num=sample_num,
                steady_time=steady_time,
                repeat_times=repeat_times,
                pos_data_path=root_path / f"Train_Joint{train_joint_no}" / "data_pos",
                neg_data_path=root_path / f"Train_Joint{train_joint_no}" / "data_neg",
            )
        )

    return jobs


def build_collection_plan(config: dict, root_path: Path) -> CollectionPlan:
    arm_name = str(config["ARM_NAME"])
    jobs = _resolve_joint_plan(config["data_collection"], arm_name, root_path)
    return CollectionPlan(arm_name=arm_name, output_root=root_path, jobs=jobs)


def build_joint_trajectory(init_joint_range_rad: list[float], train_joint_no: int, theta_joint_no: int, theta: float, train_vals: list[float]) -> np.ndarray:
    trajectory = []
    # Copy the initial range as a base
    base_q = np.array(init_joint_range_rad, dtype=float)
    # Set the theta joint which is constant for this trajectory
    base_q[theta_joint_no - 1] = theta
    
    for t in train_vals:
        q = base_q.copy()
        q[train_joint_no - 1] = t
        trajectory.append(q)
    
    if not trajectory:
        return np.zeros((7, 0), dtype=float)
    return np.stack(trajectory, axis=1)
