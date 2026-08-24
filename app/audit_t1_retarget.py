# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Quantitative balance and upper-body audit for a T1 retargeted CSV."""

import argparse
import json
from pathlib import Path

import newton
import numpy as np
import warp as wp
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from soma_retargeter.assets import bvh as bvh_utils
from soma_retargeter.assets.csv import T123DOF_CSVConfig
from soma_retargeter.pipelines import utils as pipeline_utils
from soma_retargeter.pipelines.newton_pipeline import NewtonPipeline
from soma_retargeter.utils import io_utils
from soma_retargeter.utils.newton_utils import get_name_from_label
from soma_retargeter.utils.space_conversion_utils import (
    SpaceConverter,
    get_facing_direction_type_from_str,
)


_FOOT_CORNERS = np.array(
    [
        [-0.1023, -0.06, -0.06],
        [-0.1023, 0.06, -0.06],
        [0.1225, -0.06, -0.06],
        [0.1225, 0.06, -0.06],
    ],
    dtype=np.float32,
)
_SOLE_CENTER = np.array([0.0101, 0.0, -0.06], dtype=np.float32)


def _percentiles(values, percentiles=(50, 95)):
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return {f"p{p}": None for p in percentiles}
    return {
        f"p{p}": float(value)
        for p, value in zip(percentiles, np.percentile(values, percentiles))
    }


def _load_csv_joint_q(csv_path: Path):
    rows = np.loadtxt(csv_path, delimiter=",", skiprows=1, ndmin=2)
    root_quat = Rotation.from_euler("xyz", rows[:, 4:7], degrees=False).as_quat()
    joint_q = np.concatenate((rows[:, 1:4], root_quat, rows[:, 7:]), axis=1)
    if joint_q.shape[1] != 30:
        raise ValueError(
            f"Expected T1 floating base + 23 joints (30 coordinates), got {joint_q.shape[1]}.")
    return rows, joint_q.astype(np.float32)


def _fk_body_transforms(joint_q: np.ndarray, batch_size: int):
    robot_builder = newton.ModelBuilder()
    robot_builder.add_mjcf(pipeline_utils.get_robot_mjcf_path(pipeline_utils.TargetType.T1))
    body_names = [get_name_from_label(label) for label in robot_builder.body_label]
    body_count = robot_builder.body_count

    builder = newton.ModelBuilder()
    for _ in range(batch_size):
        builder.add_builder(robot_builder, wp.transform_identity())
    model = builder.finalize()
    state = model.state()

    if model.joint_coord_count != batch_size * joint_q.shape[1]:
        raise RuntimeError("Unexpected Newton T1 coordinate layout.")

    body_q_chunks = []
    padded_q = np.empty((batch_size, joint_q.shape[1]), dtype=np.float32)
    for start in range(0, len(joint_q), batch_size):
        count = min(batch_size, len(joint_q) - start)
        padded_q[:count] = joint_q[start:start + count]
        padded_q[count:] = joint_q[start + count - 1]
        wp.copy(model.joint_q, wp.array(padded_q.reshape(-1), dtype=wp.float32))
        newton.eval_fk(model, model.joint_q, model.joint_qd, state, None)
        body_q_chunks.append(
            state.body_q.numpy().reshape(batch_size, body_count, 7)[:count].copy())

    body_q = np.concatenate(body_q_chunks, axis=0)
    body_mass = model.body_mass.numpy().reshape(batch_size, body_count)[0].copy()
    body_com = model.body_com.numpy().reshape(batch_size, body_count, 3)[0].copy()
    return body_names, body_q, body_mass, body_com


def _body_point(body_q: np.ndarray, local_point: np.ndarray):
    return body_q[:, :3] + Rotation.from_quat(body_q[:, 3:7]).apply(local_point)


def _body_points(body_q: np.ndarray, local_points: np.ndarray):
    rotation = Rotation.from_quat(body_q[:, 3:7])
    return body_q[:, None, :3] + np.stack(
        [rotation.apply(point) for point in local_points], axis=1)


def _support_margin(point_xy: np.ndarray, support_points_xy: np.ndarray):
    hull = ConvexHull(support_points_xy)
    equations = hull.equations
    signed_distances = -(equations[:, :2] @ point_xy + equations[:, 2])
    return float(np.min(signed_distances))


def _relative_heading_degrees(reference: Rotation, target: Rotation):
    reference_xy = reference.apply(np.array([1.0, 0.0, 0.0]))[:, :2]
    target_xy = target.apply(np.array([1.0, 0.0, 0.0]))[:, :2]
    reference_xy /= np.maximum(np.linalg.norm(reference_xy, axis=1, keepdims=True), 1.0e-6)
    target_xy /= np.maximum(np.linalg.norm(target_xy, axis=1, keepdims=True), 1.0e-6)
    cross = reference_xy[:, 0] * target_xy[:, 1] - reference_xy[:, 1] * target_xy[:, 0]
    dot = np.sum(reference_xy * target_xy, axis=1)
    return np.rad2deg(np.arctan2(cross, dot))


def _compute_balance_metrics(rows, body_names, body_q, body_mass, body_com, fps):
    index = {name: i for i, name in enumerate(body_names)}
    trunk_q = body_q[:, index["Trunk"]]
    waist_q = body_q[:, index["Waist"]]
    left_foot_q = body_q[:, index["left_foot_link"]]
    right_foot_q = body_q[:, index["right_foot_link"]]

    trunk_rot = Rotation.from_quat(trunk_q[:, 3:7])
    waist_rot = Rotation.from_quat(waist_q[:, 3:7])
    left_rot = Rotation.from_quat(left_foot_q[:, 3:7])
    right_rot = Rotation.from_quat(right_foot_q[:, 3:7])

    left_heading = _relative_heading_degrees(waist_rot, left_rot)
    right_heading = _relative_heading_degrees(waist_rot, right_rot)
    trunk_up = trunk_rot.apply(np.array([0.0, 0.0, 1.0]))
    trunk_lean = np.rad2deg(np.arccos(np.clip(trunk_up[:, 2], -1.0, 1.0)))

    left_sole = _body_point(left_foot_q, _SOLE_CENTER)
    right_sole = _body_point(right_foot_q, _SOLE_CENTER)
    left_corners = _body_points(left_foot_q, _FOOT_CORNERS)
    right_corners = _body_points(right_foot_q, _FOOT_CORNERS)
    left_bottom = np.min(left_corners[:, :, 2], axis=1)
    right_bottom = np.min(right_corners[:, :, 2], axis=1)
    frame_floor = np.minimum(left_bottom, right_bottom)
    left_contact = left_bottom <= frame_floor + 0.015
    right_contact = right_bottom <= frame_floor + 0.015

    left_up = left_rot.apply(np.array([0.0, 0.0, 1.0]))
    right_up = right_rot.apply(np.array([0.0, 0.0, 1.0]))
    left_tilt = np.rad2deg(np.arccos(np.clip(left_up[:, 2], -1.0, 1.0)))
    right_tilt = np.rad2deg(np.arccos(np.clip(right_up[:, 2], -1.0, 1.0)))
    contact_tilt = np.concatenate((left_tilt[left_contact], right_tilt[right_contact]))

    foot_delta_world = left_sole - right_sole
    foot_delta_waist = waist_rot.inv().apply(foot_delta_world)
    stance_width = np.abs(foot_delta_waist[:, 1])

    body_rot = Rotation.from_quat(body_q[:, :, 3:7].reshape(-1, 4))
    world_com = body_q[:, :, :3] + body_rot.apply(
        np.broadcast_to(body_com, (len(body_q), len(body_com), 3)).reshape(-1, 3)
    ).reshape(len(body_q), len(body_com), 3)
    total_com = np.sum(world_com * body_mass[None, :, None], axis=1) / np.sum(body_mass)

    support_margins = []
    for frame in range(len(body_q)):
        support = []
        if left_contact[frame]:
            support.extend(left_corners[frame, :, :2])
        if right_contact[frame]:
            support.extend(right_corners[frame, :, :2])
        support_margins.append(_support_margin(total_com[frame, :2], np.asarray(support)))
    support_margins = np.asarray(support_margins)

    left_speed = np.linalg.norm(np.diff(left_sole[:, :2], axis=0), axis=1) * fps
    right_speed = np.linalg.norm(np.diff(right_sole[:, :2], axis=0), axis=1) * fps
    contact_slip = np.concatenate((
        left_speed[left_contact[1:] & left_contact[:-1]],
        right_speed[right_contact[1:] & right_contact[:-1]],
    ))

    return {
        "root_trunk_lean_deg": _percentiles(trunk_lean),
        "left_foot_yaw_relative_waist_deg": _percentiles(left_heading, (5, 50, 95)),
        "right_foot_yaw_relative_waist_deg": _percentiles(right_heading, (5, 50, 95)),
        "absolute_foot_splay_deg": _percentiles(
            np.concatenate((np.abs(left_heading), np.abs(right_heading)))),
        "contact_foot_splay_deg": _percentiles(np.concatenate((
            np.abs(left_heading[left_contact]),
            np.abs(right_heading[right_contact]),
        ))),
        "contact_foot_tilt_deg": _percentiles(contact_tilt),
        "stance_width_m": _percentiles(stance_width, (5, 50, 95)),
        "com_support_margin_m": {
            **_percentiles(support_margins, (5, 50, 95)),
            "outside_fraction": float(np.mean(support_margins < 0.0)),
        },
        "contact_foot_slip_m_per_s": _percentiles(contact_slip),
        "root_rpy_deg": {
            "roll": _percentiles(np.rad2deg(rows[:, 4]), (5, 50, 95)),
            "pitch": _percentiles(np.rad2deg(rows[:, 5]), (5, 50, 95)),
        },
    }


def _point_error(output, target):
    error = np.linalg.norm(output - target, axis=1)
    return {
        "rmse_m": float(np.sqrt(np.mean(np.square(error)))),
        **_percentiles(error),
    }


def _compute_continuity_metrics(rows, fps):
    """Report time-domain joint discontinuities independently of FK quality."""
    joint_names = T123DOF_CSVConfig.csv_header[7:]
    joints = np.asarray(rows[:, 7:], dtype=np.float64)
    velocity = np.diff(joints, axis=0) * fps
    acceleration = np.diff(joints, n=2, axis=0) * fps * fps

    def top_entries(values, frame_offset, count=8):
        entries = []
        for joint_idx, joint_name in enumerate(joint_names):
            frame = int(np.argmax(np.abs(values[:, joint_idx])))
            entries.append({
                "joint": joint_name,
                "frame": frame + frame_offset,
                "absolute_value": float(abs(values[frame, joint_idx])),
            })
        return sorted(
            entries,
            key=lambda entry: entry["absolute_value"],
            reverse=True)[:count]

    abs_velocity = np.abs(velocity)
    abs_acceleration = np.abs(acceleration)
    return {
        "joint_velocity_rad_per_s": {
            **_percentiles(abs_velocity.reshape(-1), (95, 99, 99.9)),
            "maximum": float(np.max(abs_velocity)),
            "frames_over_18_rad_per_s": int(np.sum(
                np.any(abs_velocity > 18.0, axis=1))),
            "worst_joints": top_entries(velocity, 1),
        },
        "joint_acceleration_rad_per_s2": {
            **_percentiles(abs_acceleration.reshape(-1), (95, 99, 99.9)),
            "maximum": float(np.max(abs_acceleration)),
            "worst_joints": top_entries(acceleration, 2),
        },
    }


def _compute_upper_metrics(bvh_path, retarget_config, body_names, body_q):
    skeleton, animation = bvh_utils.load_bvh(bvh_path)
    pipeline = NewtonPipeline(
        skeleton,
        "soma",
        "t1",
        retarget_config=retarget_config,
    )
    converter = SpaceConverter(get_facing_direction_type_from_str("Mujoco"))
    pipeline.add_input_motions(
        [animation], [converter.transform(wp.transform_identity())], True)
    targets = pipeline.input_targets[0][-len(body_q):]
    target_index = {name: i for i, name in enumerate(pipeline.mapped_joints)}
    body_index = {name: i for i, name in enumerate(body_names)}

    metrics = {}
    pairs = (
        ("left_shoulder_anchor", "AL1", "LeftArm", None),
        ("right_shoulder_anchor", "AR1", "RightArm", None),
        ("left_shoulder", "AL2", "LeftArm", None),
        ("right_shoulder", "AR2", "RightArm", None),
        ("left_elbow", "AL3", "LeftForeArm", None),
        ("right_elbow", "AR3", "RightForeArm", None),
        ("left_hand", "left_hand_link", "LeftHand", np.array([0.0, 0.04, 0.0])),
        ("right_hand", "right_hand_link", "RightHand", np.array([0.0, -0.04, 0.0])),
    )
    for label, body_name, target_name, local_offset in pairs:
        output_body = body_q[:, body_index[body_name]]
        output_point = (
            output_body[:, :3]
            if local_offset is None
            else _body_point(output_body, local_offset)
        )
        metrics[f"{label}_position_error"] = _point_error(
            output_point, targets[:, target_index[target_name], :3])

    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--bvh", type=Path)
    parser.add_argument(
        "--retarget-config",
        type=Path,
        default=io_utils.get_config_file("t1", "soma_to_t1_retargeter_config.json"),
    )
    parser.add_argument("--fps", type=float, default=120.0)
    parser.add_argument("--fk-batch-size", type=int, default=64)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rows, joint_q = _load_csv_joint_q(args.csv)
    body_names, body_q, body_mass, body_com = _fk_body_transforms(
        joint_q, args.fk_batch_size)
    result = {
        "csv": str(args.csv),
        "frames": int(len(rows)),
        "fps": args.fps,
        "balance": _compute_balance_metrics(
            rows, body_names, body_q, body_mass, body_com, args.fps),
        "continuity": _compute_continuity_metrics(rows, args.fps),
    }
    if args.bvh:
        retarget_config = io_utils.load_json(args.retarget_config)
        result["bvh"] = str(args.bvh)
        result["upper_body"] = _compute_upper_metrics(
            args.bvh, retarget_config, body_names, body_q)

    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
