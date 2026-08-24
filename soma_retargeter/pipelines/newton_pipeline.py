# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import warp as wp
import numpy as np
import newton
import newton.ik as ik
from scipy.spatial.transform import Rotation
from tqdm import trange

import soma_retargeter.assets.bvh as bvh_utils
import soma_retargeter.utils.newton_utils as newton_utils
import soma_retargeter.utils.io_utils as io_utils
import soma_retargeter.pipelines.utils as pipeline_utils
from soma_retargeter.pipelines.ik_objectives import IKSmoothJointFilter, IKJointPosePreference
from soma_retargeter.animation.skeleton import Skeleton, SkeletonInstance
from soma_retargeter.animation.animation_buffer import AnimationBuffer
from soma_retargeter.robotics.human_to_robot_scaler import HumanToRobotScaler
from soma_retargeter.robotics.csv_animation_buffer import CSVAnimationBuffer
from soma_retargeter.pipelines.feet_stabilizer import FeetStabilizer
from soma_retargeter.pipelines.joint_limit_clamper import JointLimitClamper

_DEFAULT_IK_SOLVER_ITERATIONS = 24
_DEFAULT_JOINT_LIMIT_OBJECTIVE_WEIGHT = 10.0
_DEFAULT_SMOOTH_JOINT_FILTER_OBJECTIVE_WEIGHT = 5.5
_DEFAULT_NUM_INITIALIZATION_FRAMES = 10
_DEFAULT_NUM_STABILIZATION_FRAMES = 5


class NewtonPipeline:
    """
    Newton-based motion retargeting pipeline.

    This pipeline retargets human motion captured on a common skeleton
    to a target robot using inverse kinematics (IK),
    custom objectives, and optional post-processing filters such as
    joint limit clamping and feet stabilization.
    """
    def __init__(self, skeleton: Skeleton, source_type='soma', robot_type='unitree_g1', retarget_config: dict = None):
        """
        Initialize the Newton retargeting pipeline.

        Args:
            skeleton: Common skeleton definition used by the input clips to be retargeted.
            source_type: Source skeleton type name. Currently only "soma" is supported.
            robot_type: Target robot type name.
            retarget_config: Optional configuration dictionary. If None, a
                configuration is loaded from disk based on the source/target
                types.

        Raises:
            ValueError: If the target robot type is not supported.
        """
        self.source_type = pipeline_utils.get_source_type_from_str(source_type)
        self.target_type = pipeline_utils.get_target_type_from_str(robot_type)
        self.input_targets = []
        self.input_sample_rates = []
        self.max_frames = -1

        if retarget_config is None:
            retargeter_config = pipeline_utils.get_retargeter_config(self.source_type, self.target_type)
        else:
            retargeter_config = retarget_config

        self.ik_iterations = retargeter_config.get('ik_iterations', _DEFAULT_IK_SOLVER_ITERATIONS)
        self.joint_limit_weight = retargeter_config.get('joint_limit_weight', _DEFAULT_JOINT_LIMIT_OBJECTIVE_WEIGHT)
        self.smooth_joint_filter_weight = retargeter_config.get('smooth_joint_filter_weight', _DEFAULT_SMOOTH_JOINT_FILTER_OBJECTIVE_WEIGHT)
        self.post_processing_enabled = retargeter_config.get('enable_post_processing', True)
        self.enable_self_penetration = False
        self.smooth_joint_filter_coord_masks = None
        self.joint_limit_clamper = None
        self.default_joint_pose = retargeter_config.get('default_joint_pose', {})
        self.default_joint_pose_body_map = retargeter_config.get('default_joint_pose_body_map', {})
        self.joint_delta_limits = retargeter_config.get('joint_delta_limits', {})
        self.joint_delta_limit_body_map = retargeter_config.get('joint_delta_limit_body_map', {})
        self.preferred_joint_pose = retargeter_config.get('preferred_joint_pose', {})
        self.preferred_joint_pose_body_map = retargeter_config.get('preferred_joint_pose_body_map', {})
        self.preferred_joint_pose_weights = retargeter_config.get('preferred_joint_pose_weights', {})
        self.preferred_joint_pose_weight = retargeter_config.get('preferred_joint_pose_weight', 0.0)
        self.retarget_options = retargeter_config.get('retarget_options', {})
        self.temporal_smoothing = self.retarget_options.get('temporal_smoothing', {})

        self.robot_builder = newton.ModelBuilder()
        self.robot_builder.add_mjcf(pipeline_utils.get_robot_mjcf_path(self.target_type))

        self.human_robot_scaler = HumanToRobotScaler(
            skeleton, retargeter_config['model_height'], io_utils.get_config_file(retargeter_config['human_robot_scaler_config']))

        self.num_body_count = self.robot_builder.body_count
        self.num_dofs = self.robot_builder.joint_dof_count
        self.ik_model = self._build_model(1)

        (
            self.mapped_joints,
            self.mapped_joint_indices,
            self.mapped_body_link_pos_data,
            self.mapped_body_link_rot_data
        ) = self._build_target_mapping(
            self.ik_model,
            self.human_robot_scaler.skeleton,
            retargeter_config)

        smooth_joint_filter_objective_body_masks = retargeter_config.get('smooth_joint_filter_objective_body_masks', None)
        if smooth_joint_filter_objective_body_masks is not None:
            self.smooth_joint_filter_coord_masks = newton_utils.create_joint_coord_masks(
                self.ik_model, smooth_joint_filter_objective_body_masks, 0.0)
        self.preferred_joint_pose_target = None
        self.preferred_joint_pose_coord_masks = None
        if self.preferred_joint_pose and self.preferred_joint_pose_weight > 0.0:
            (
                self.preferred_joint_pose_target,
                self.preferred_joint_pose_coord_masks
            ) = self._build_preferred_joint_pose_arrays()

        effector_names = self.human_robot_scaler.effector_names()
        self.target_effector_indices = [effector_names.index(name) for name in self.mapped_joints]
        self.feet_effector_indices = [
            self.mapped_joints.index("LeftFoot"),
            self.mapped_joints.index("RightFoot")]

        self.feet_stabilizer = FeetStabilizer(io_utils.get_config_file(retargeter_config['feet_stabilizer_config']))
        self.joint_limit_clamper = JointLimitClamper(self.ik_model)

        self.initialization_pose = None
        self.num_initialization_frames = 0
        self.num_stabilization_frames = 0
        if (retargeter_config['initialization_pose']):
            init_skel, init_anim = bvh_utils.load_bvh(io_utils.get_config_file(retargeter_config['initialization_pose']))
            self.initialization_pose = SkeletonInstance(init_skel, [0, 0, 0], wp.transform_identity())
            self.initialization_pose.set_local_transforms(init_anim.get_local_transforms(0))
            self.num_initialization_frames = retargeter_config.get('num_initialization_frames', _DEFAULT_NUM_INITIALIZATION_FRAMES)
            self.num_stabilization_frames = retargeter_config.get('num_stabilization_frames', _DEFAULT_NUM_STABILIZATION_FRAMES)

    def clear(self):
        """
        Clear all accumulated input motions and reset internal state.

        This removes all previously added motions set for retargeting.
        It does not modify static configuration such as the robot model or IK settings.
        """
        self.input_targets = []
        self.input_sample_rates = []
        self.max_frames = -1

    def add_input_motions(self, buffers: list[AnimationBuffer], offsets: list[wp.transform], scale_animation: bool):
        """
        Add input motions to be retargeted.
        Each buffer is converted into IK targets using the human-to-robot scaler.

        Args:
            buffers: List of input animation buffers defined on the common skeleton.
            offsets: List of root transforms applied to each buffer. If the
                length does not match `buffers`, identity transforms are used
                for all.
            scale_animation: Whether to rescale the source motion using the
                configured HumanToRobotScaler.
        """
        offsets = offsets if len(offsets) == len(buffers) else [wp.transform_identity()] * len(buffers)
        for i in trange(len(buffers), desc="[INFO] Converting Motions for Newton"):
            buffer = buffers[i]
            if self.initialization_pose and self.num_initialization_frames > 0:
                buffer = newton_utils.create_buffer_with_initialization_frames(
                    self.initialization_pose, buffers[i], self.num_initialization_frames, self.num_stabilization_frames)

            self.max_frames = max(self.max_frames, buffer.num_frames)
            buffer_effectors = self.human_robot_scaler.compute_effectors_from_buffer(buffer, scale_animation, offsets[i])

            targets = np.asarray(
                buffer_effectors[:, self.target_effector_indices, :],
                dtype=np.float32).copy()
            self._apply_target_conditioning(targets)
            self.input_targets.append(targets)
            self.input_sample_rates.append(buffers[i].sample_rate)

    def _apply_target_conditioning(self, targets: np.ndarray) -> None:
        """Apply optional, robot-aware conditioning to scaled IK targets in-place."""
        upper_body = self.retarget_options.get('upper_body', {})
        limb_lengths = upper_body.get('limb_length_conditioning', {})
        if limb_lengths.get('enabled', False):
            length_blend = float(np.clip(limb_lengths.get('blend', 1.0), 0.0, 1.0))
            for chain_name, chain in limb_lengths.get('chains', {}).items():
                shoulder_name = chain['shoulder_joint']
                elbow_name = chain['elbow_joint']
                hand_name = chain['hand_joint']
                missing = [
                    name for name in (shoulder_name, elbow_name, hand_name)
                    if name not in self.mapped_joints
                ]
                if missing:
                    raise ValueError(
                        f"[ERROR]: Upper-body chain '{chain_name}' references "
                        f"joints missing from ik_map: {missing}.")

                shoulder_idx = self.mapped_joints.index(shoulder_name)
                elbow_idx = self.mapped_joints.index(elbow_name)
                hand_idx = self.mapped_joints.index(hand_name)
                shoulder = targets[:, shoulder_idx, 0:3]
                old_elbow = targets[:, elbow_idx, 0:3].copy()
                old_hand = targets[:, hand_idx, 0:3].copy()

                upper_length = float(chain['upper_arm_length_m'])
                forearm_length = float(chain['forearm_length_m'])
                hand_axis = old_hand - shoulder
                old_hand_distance = np.linalg.norm(
                    hand_axis, axis=1, keepdims=True)
                hand_axis /= np.maximum(old_hand_distance, 1.0e-6)

                # Keeping a robot arm exactly at maximum reach makes the elbow
                # plane under-determined. A tiny amount of target noise can then
                # select the opposite IK branch and visibly flip the arm. Keep a
                # configurable bend margin and reconstruct the elbow from the
                # source elbow plane instead.
                maximum_reach_ratio = float(np.clip(
                    chain.get('maximum_reach_ratio', 0.94), 0.5, 0.999))
                minimum_reach = abs(upper_length - forearm_length) + 1.0e-4
                maximum_reach = (upper_length + forearm_length) * maximum_reach_ratio
                conditioned_distance = np.clip(
                    old_hand_distance, minimum_reach, maximum_reach)
                conditioned_hand = shoulder + hand_axis * conditioned_distance

                elbow_plane = old_elbow - shoulder
                elbow_plane -= hand_axis * np.sum(
                    elbow_plane * hand_axis, axis=1, keepdims=True)
                elbow_plane_norm = np.linalg.norm(
                    elbow_plane, axis=1, keepdims=True)
                bend_directions = np.zeros_like(elbow_plane)
                bend_alpha = float(np.clip(
                    chain.get('bend_direction_smoothing', 0.25), 0.0, 1.0))
                previous_bend = None
                for frame_idx in range(len(bend_directions)):
                    if elbow_plane_norm[frame_idx, 0] > 1.0e-5:
                        candidate = elbow_plane[frame_idx] / elbow_plane_norm[frame_idx, 0]
                    elif previous_bend is not None:
                        candidate = previous_bend
                    else:
                        fallback = np.array([0.0, 0.0, 1.0], dtype=np.float32)
                        fallback -= hand_axis[frame_idx] * np.dot(
                            fallback, hand_axis[frame_idx])
                        if np.linalg.norm(fallback) <= 1.0e-5:
                            fallback = np.array([0.0, 1.0, 0.0], dtype=np.float32)
                            fallback -= hand_axis[frame_idx] * np.dot(
                                fallback, hand_axis[frame_idx])
                        candidate = fallback / max(np.linalg.norm(fallback), 1.0e-6)

                    if previous_bend is not None:
                        if np.dot(candidate, previous_bend) < 0.0:
                            candidate = -candidate
                        candidate = (
                            previous_bend * (1.0 - bend_alpha)
                            + candidate * bend_alpha)
                        candidate /= max(np.linalg.norm(candidate), 1.0e-6)
                    bend_directions[frame_idx] = candidate
                    previous_bend = candidate

                distance = conditioned_distance[:, 0]
                along_axis = (
                    upper_length * upper_length
                    - forearm_length * forearm_length
                    + distance * distance) / np.maximum(2.0 * distance, 1.0e-6)
                bend_radius = np.sqrt(np.maximum(
                    upper_length * upper_length - along_axis * along_axis, 0.0))
                conditioned_elbow = (
                    shoulder
                    + hand_axis * along_axis[:, None]
                    + bend_directions * bend_radius[:, None])

                new_elbow = (
                    old_elbow * (1.0 - length_blend)
                    + conditioned_elbow * length_blend)
                new_hand = (
                    old_hand * (1.0 - length_blend)
                    + conditioned_hand * length_blend)

                targets[:, elbow_idx, 0:3] = new_elbow.astype(np.float32)
                targets[:, hand_idx, 0:3] = new_hand.astype(np.float32)

        lower_body = self.retarget_options.get('lower_body', {})
        foot_heading = lower_body.get('foot_heading', {})
        if not foot_heading.get('enabled', False):
            return

        reference_name = foot_heading.get('reference_joint', 'Hips')
        if reference_name not in self.mapped_joints:
            raise ValueError(
                f"[ERROR]: Foot-heading reference joint '{reference_name}' is not in ik_map.")

        reference_idx = self.mapped_joints.index(reference_name)
        reference_rot = Rotation.from_quat(targets[:, reference_idx, 3:7])
        target_yaws = foot_heading.get('target_relative_yaw_degrees', {})
        toe_joints = foot_heading.get('toe_joints', {})
        blend = float(np.clip(foot_heading.get('blend', 1.0), 0.0, 1.0))
        max_correction = np.deg2rad(float(foot_heading.get('max_correction_degrees', 180.0)))
        foot_names = foot_heading.get('foot_joints', ['LeftFoot', 'RightFoot'])
        missing_feet = [name for name in foot_names if name not in self.mapped_joints]
        if missing_feet:
            raise ValueError(
                f"[ERROR]: Foot-heading joints missing from ik_map: {missing_feet}.")

        stance_flattening = lower_body.get('stance_flattening', {})
        lowest_foot_z = np.min(
            np.stack([
                targets[:, self.mapped_joints.index(name), 2]
                for name in foot_names
            ], axis=1),
            axis=1)

        for foot_name in foot_names:
            foot_idx = self.mapped_joints.index(foot_name)
            old_foot_rot = Rotation.from_quat(targets[:, foot_idx, 3:7])
            relative_rot = reference_rot.inv() * old_foot_rot
            relative_euler = relative_rot.as_euler('xyz', degrees=False)
            old_yaw = relative_euler[:, 2]
            target_yaw = np.deg2rad(float(target_yaws.get(foot_name, 0.0)))
            correction = np.arctan2(
                np.sin(target_yaw - old_yaw),
                np.cos(target_yaw - old_yaw))
            correction = np.clip(correction, -max_correction, max_correction)
            relative_euler[:, 2] = old_yaw + blend * correction
            new_foot_rot = reference_rot * Rotation.from_euler('xyz', relative_euler)

            if stance_flattening.get('enabled', False):
                flatten_height = max(
                    float(stance_flattening.get('height_above_lowest_m', 0.025)),
                    1.0e-6)
                relative_height = np.maximum(
                    targets[:, foot_idx, 2] - lowest_foot_z, 0.0)
                stance_weight = np.clip(
                    1.0 - relative_height / flatten_height, 0.0, 1.0)
                # Smoothstep removes the binary rotation jump at the contact
                # classification boundary while retaining a flat lowest foot.
                stance_weight = stance_weight * stance_weight * (3.0 - 2.0 * stance_weight)
                flatten_blend = float(np.clip(
                    stance_flattening.get('blend', 1.0), 0.0, 1.0))
                world_euler = new_foot_rot.as_euler('xyz', degrees=False)
                world_euler[:, 0:2] *= (
                    1.0 - flatten_blend * stance_weight[:, None])
                new_foot_rot = Rotation.from_euler('xyz', world_euler)

            targets[:, foot_idx, 3:7] = new_foot_rot.as_quat().astype(np.float32)

            toe_name = toe_joints.get(foot_name)
            if toe_name is None:
                continue
            if toe_name not in self.mapped_joints:
                raise ValueError(
                    f"[ERROR]: Toe-heading joint '{toe_name}' is not in ik_map.")

            # Rotate the foot-to-toe segment by the same orientation correction.
            # This keeps the virtual toe positional objective consistent with the
            # conditioned foot rotation objective.
            toe_idx = self.mapped_joints.index(toe_name)
            foot_to_toe_world = targets[:, toe_idx, 0:3] - targets[:, foot_idx, 0:3]
            foot_to_toe_local = old_foot_rot.inv().apply(foot_to_toe_world)
            targets[:, toe_idx, 0:3] = (
                targets[:, foot_idx, 0:3] + new_foot_rot.apply(foot_to_toe_local)
            ).astype(np.float32)

    def execute(self):
        """
        Run the retargeting pipeline on all added input motions.

        This method builds a multi-environment Newton model, sets up IK
        objectives, and performs frame-by-frame IK solving.

        Returns:
            list[CSVAnimationBuffer]: A list of retargeted robot motions, one per input motion.
        """
        num_envs = len(self.input_targets)
        if num_envs == 0:
            self.retargeted_motions = []
            return

        # Clamp objective weights to valid values
        self.ik_iterations = max(1, self.ik_iterations)
        self.joint_limit_weight = max(0.0, self.joint_limit_weight)
        self.smooth_joint_filter_weight = max(0.0, self.smooth_joint_filter_weight)

        print("[INFO] Newton Retargeter Settings: ")
        print(f"[INFO]\t  Source Skeleton Type: {pipeline_utils.get_source_str_from_type(self.source_type)}")
        print(f"[INFO]\t  Target Robot Type: {pipeline_utils.get_target_str_from_type(self.target_type)}")
        print(f"[INFO]\t  Post-Processing Enabled: {self.post_processing_enabled}")
        print(f"[INFO]\t  Initialization Pose: {self.initialization_pose is not None}")
        print(f"[INFO]\t  Initialization Frame Count: {self.num_initialization_frames}")
        print(f"[INFO]\t  Constraint Stabilization Frame Count: {self.num_stabilization_frames}")
        print(f"[INFO]\t  IK Solver Iterations: {self.ik_iterations}")
        print(f"[INFO]\t  Joint Limit Objective Weight: {self.joint_limit_weight}")
        print(f"[INFO]\t  Smooth Joint Filter Objective Weight: {self.smooth_joint_filter_weight}")
        print(f"[INFO]\t  Default Joint Pose Entries: {len(self.default_joint_pose)}")
        print(f"[INFO]\t  Joint Delta Limit Entries: {len(self.joint_delta_limits)}")
        print(f"[INFO]\t  Preferred Joint Pose Entries: {len(self.preferred_joint_pose)}")
        foot_heading = self.retarget_options.get('lower_body', {}).get('foot_heading', {})
        print(f"[INFO]\t  Foot Heading Conditioning: {foot_heading.get('enabled', False)}")
        print(f"[INFO]\t  Temporal Smoothing: {self.temporal_smoothing.get('enabled', False)}")

        model = self._build_model(num_envs)
        self._apply_default_joint_pose(model, num_envs)
        state = model.state()

        if self.post_processing_enabled:
            self.feet_stabilizer.setup_num_envs(num_envs)
            env_feet_tx = np.empty((num_envs, len(self.feet_effector_indices), 7), dtype=np.float32)

        (
            position_objectives,
            rotation_objectives,
            joint_limit_objective,
            smooth_joint_filter_objective,
            preferred_joint_pose_objective
        ) = self._create_ik_objectives(num_envs, model, state)

        # Add optional objectives
        ik_solver_active_objectives = [*position_objectives, *rotation_objectives]
        if self.joint_limit_weight > 0.0:
            ik_solver_active_objectives.append(joint_limit_objective)
        if self.smooth_joint_filter_weight > 0.0:
            ik_solver_active_objectives.append(smooth_joint_filter_objective)
        if preferred_joint_pose_objective is not None:
            ik_solver_active_objectives.append(preferred_joint_pose_objective)

        ik_solver = ik.IKSolver(
            model=self.ik_model,
            n_problems=num_envs,
            objectives=ik_solver_active_objectives,
            lambda_initial=0.1,
            jacobian_mode=ik.IKJacobianType.ANALYTIC)

        joint_q = wp.empty(shape=(num_envs, self.ik_model.joint_coord_count))
        wp.copy(joint_q, model.joint_q)

        # Solver initialization
        ik_solver.reset()

        graph_capture = None

        def single_step():
            ik_solver.step(joint_q, joint_q, iterations=self.ik_iterations)

        if wp.get_device().is_cuda:
            with wp.ScopedCapture() as cap:
                single_step()
            graph_capture = cap.graph
        else:
            ik_solver.step(joint_q, joint_q, iterations=self.ik_iterations)

        #import time
        num_frames_to_remove = self.num_initialization_frames + self.num_stabilization_frames
        prev_joint_q_np = joint_q.numpy().copy()
        joint_q_data = [np.full((len(self.input_targets[i]),), None) for i in range(num_envs)]
        for frame in trange(self.max_frames, desc="[INFO] Retargeting Motions"):
            if frame <= num_frames_to_remove:
                smooth_joint_filter_objective.set_weight(self.smooth_joint_filter_weight * (frame / float(num_frames_to_remove)))

            #start_time = time.time()
            for env in range(num_envs):
                if frame > (len(self.input_targets[env])-1):
                    continue
                frame_targets = self.input_targets[env][frame]
                for i, target in enumerate(frame_targets):
                    position_objectives[i].set_target_position(env, wp.vec3(*target[0:3]))
                    rotation_objectives[i].set_target_rotation(env, wp.quat(*target[3:7]))

            if graph_capture is not None:
                wp.capture_launch(graph_capture)
            else:
                single_step()

            prev_joint_q_np = self._apply_joint_delta_limits(joint_q, prev_joint_q_np, model, num_envs)

            data = None
            if self.post_processing_enabled:
                self.feet_stabilizer.reset_state(joint_q)

                for env in range(num_envs):
                    if frame > (len(self.input_targets[env])-1):
                        env_feet_tx[env] = np.asarray(self.input_targets[env][-1][self.feet_effector_indices])
                    else:
                        env_feet_tx[env] = np.asarray(self.input_targets[env][frame][self.feet_effector_indices])

                self.feet_stabilizer.solve(env_feet_tx)
                data = self.joint_limit_clamper.apply(self.feet_stabilizer.current_state()).numpy()
            else:
                data = self.joint_limit_clamper.apply(joint_q).numpy()

            for env in range(num_envs):
                if frame > (len(self.input_targets[env])-1):
                    continue

                # ``WarpArray.numpy()`` may expose storage reused by the next
                # frame. Keep an owned snapshot or every CSV row can collapse
                # to the final solved pose on the CPU execution path.
                joint_q_data[env][frame] = data[env].copy()

            #end_time = time.time()
            #print(f"Time taken for frame {frame}: {end_time - start_time} seconds")

        output_buffers = []
        for env in range(num_envs):
            raw_motion = np.stack(
                joint_q_data[env][num_frames_to_remove:]).astype(np.float32)
            raw_motion = self._apply_temporal_smoothing(
                raw_motion, self.input_sample_rates[env])
            output_buffers.append(CSVAnimationBuffer.create_from_raw_data(
                raw_motion, self.input_sample_rates[env]))
        return output_buffers

    def _apply_temporal_smoothing(
            self, raw_motion: np.ndarray, sample_rate: float) -> np.ndarray:
        """Remove isolated IK branch spikes without adding causal playback lag.

        Retargeting is an offline operation, so a zero-phase filter can smooth
        both sides of a discontinuity while keeping events aligned with the BVH.
        Quaternion signs are made continuous before filtering and normalized
        afterwards. Joint limits are re-applied to guard against filter overshoot.
        """
        config = self.temporal_smoothing
        if not config.get('enabled', False) or len(raw_motion) < 5:
            return raw_motion

        from scipy.ndimage import median_filter
        from scipy.signal import butter, sosfiltfilt

        sample_rate = float(sample_rate)
        nyquist = 0.5 * sample_rate
        filter_order = max(1, int(config.get('filter_order', 4)))

        def low_pass(values: np.ndarray, cutoff_hz: float) -> np.ndarray:
            cutoff_hz = float(cutoff_hz)
            if cutoff_hz <= 0.0 or cutoff_hz >= nyquist:
                return values.copy()
            sos = butter(
                filter_order,
                cutoff_hz / nyquist,
                btype='lowpass',
                output='sos')
            try:
                return sosfiltfilt(sos, values, axis=0)
            except ValueError:
                # Very short clips may be shorter than scipy's padding. They
                # still retain the median cleanup below.
                return values.copy()

        output = raw_motion.astype(np.float64, copy=True)
        output[:, 0:3] = low_pass(
            output[:, 0:3],
            config.get('root_translation_cutoff_hz', 8.0))

        quaternion = output[:, 3:7].copy()
        for frame_idx in range(1, len(quaternion)):
            if np.dot(quaternion[frame_idx - 1], quaternion[frame_idx]) < 0.0:
                quaternion[frame_idx] *= -1.0
        quaternion = low_pass(
            quaternion,
            config.get('root_rotation_cutoff_hz', 8.0))
        quaternion /= np.maximum(
            np.linalg.norm(quaternion, axis=1, keepdims=True), 1.0e-8)
        output[:, 3:7] = quaternion

        joints = output[:, 7:]
        median_kernel = max(1, int(config.get('median_kernel_size', 5)))
        if median_kernel % 2 == 0:
            median_kernel += 1
        if median_kernel > 1:
            joints = median_filter(
                joints,
                size=(median_kernel, 1),
                mode='nearest')
        output[:, 7:] = low_pass(
            joints,
            config.get('joint_cutoff_hz', 8.0))

        filtered = wp.array(output.astype(np.float32), dtype=wp.float32)
        output = self.joint_limit_clamper.apply(filtered).numpy().copy()
        output[:, 3:7] /= np.maximum(
            np.linalg.norm(output[:, 3:7], axis=1, keepdims=True), 1.0e-8)
        return output.astype(np.float32)

    def _build_model(self, num_envs: int):
        builder = newton.ModelBuilder()
        for _ in range(num_envs):
            builder.add_builder(self.robot_builder, xform=wp.transform_identity())

        builder.add_ground_plane()
        model = builder.finalize(requires_grad=True)

        return model

    def _apply_default_joint_pose(self, model, num_envs: int):
        newton_utils.apply_default_joint_pose(
            model,
            self.robot_builder,
            self.default_joint_pose,
            self.default_joint_pose_body_map,
            num_envs)

    def _apply_joint_delta_limits(self, joint_q, prev_joint_q_np, model, num_envs: int):
        if not self.joint_delta_limits:
            return joint_q.numpy().copy()

        joint_q_np = joint_q.numpy().copy()
        joint_q_start_np = model.joint_q_start.numpy()
        joint_dof_dim_np = model.joint_dof_dim.numpy()
        base_body_names = [newton_utils.get_name_from_label(label) for label in self.robot_builder.body_label]
        base_body_name_to_idx = {name: idx for idx, name in enumerate(base_body_names)}

        for limit_name, max_delta in self.joint_delta_limits.items():
            body_name = self.joint_delta_limit_body_map.get(limit_name, limit_name)
            if body_name not in base_body_name_to_idx:
                raise ValueError(
                    f"[ERROR]: Joint delta limit entry '{limit_name}' maps to unknown body '{body_name}'.")

            body_idx = base_body_name_to_idx[body_name]
            for env in range(num_envs):
                model_body_idx = env * self.num_body_count + body_idx
                coord_start = int(joint_q_start_np[model_body_idx])
                lin_dim, ang_dim = joint_dof_dim_np[model_body_idx]
                coord_dim = int(lin_dim + ang_dim)
                if coord_dim != 1:
                    raise ValueError(
                        f"[ERROR]: Joint delta limit entry '{limit_name}' maps to body '{body_name}' "
                        f"with {coord_dim} coordinates; only 1-DoF joints are supported.")

                if joint_q_np.ndim == 1:
                    prev_value = prev_joint_q_np[coord_start]
                    joint_q_np[coord_start] = np.clip(
                        joint_q_np[coord_start],
                        prev_value - max_delta,
                        prev_value + max_delta)
                elif joint_q_np.ndim == 2:
                    coord_start = coord_start % joint_q_np.shape[1]
                    prev_value = prev_joint_q_np[env, coord_start]
                    joint_q_np[env, coord_start] = np.clip(
                        joint_q_np[env, coord_start],
                        prev_value - max_delta,
                        prev_value + max_delta)
                else:
                    raise ValueError(f"[ERROR]: Unsupported joint_q shape: {joint_q_np.shape}")

        wp.copy(joint_q, wp.array(joint_q_np, dtype=wp.float32))
        return joint_q_np

    def _build_target_mapping(self, model, skeleton, retargeter_config):
        mapped_joints = []
        mapped_joint_indices = []
        mapped_body_link_pos_data = []
        mapped_body_link_rot_data = []
        body_names = [newton_utils.get_name_from_label(label) for label in self.robot_builder.body_label]
        for joint, mapping_data in retargeter_config["ik_map"].items():
            mapped_joints.append(joint)
            mapped_joint_indices.append(skeleton.joint_index(joint))
            mapped_body_link_pos_data.append((
                body_names.index(mapping_data['t_body']),
                mapping_data['t_weight'],
                wp.vec3(*mapping_data.get('t_offset', [0.0, 0.0, 0.0]))))
            mapped_body_link_rot_data.append((body_names.index(mapping_data['r_body']), mapping_data['r_weight']))

        return (
            mapped_joints,
            mapped_joint_indices,
            mapped_body_link_pos_data,
            mapped_body_link_rot_data)

    def _build_preferred_joint_pose_arrays(self):
        target_q = self.ik_model.joint_q.numpy().copy()
        coord_masks = np.zeros(self.ik_model.joint_coord_count, dtype=np.float32)
        joint_q_start_np = self.ik_model.joint_q_start.numpy()
        joint_dof_dim_np = self.ik_model.joint_dof_dim.numpy()
        body_names = [newton_utils.get_name_from_label(label) for label in self.robot_builder.body_label]
        body_name_to_idx = {name: idx for idx, name in enumerate(body_names)}

        for pose_name, pose_value in self.preferred_joint_pose.items():
            body_name = self.preferred_joint_pose_body_map.get(pose_name, pose_name)
            if body_name not in body_name_to_idx:
                raise ValueError(
                    f"[ERROR]: Preferred joint pose entry '{pose_name}' maps to unknown body '{body_name}'.")
            body_idx = body_name_to_idx[body_name]
            coord_start = int(joint_q_start_np[body_idx])
            lin_dim, ang_dim = joint_dof_dim_np[body_idx]
            coord_dim = int(lin_dim + ang_dim)
            if coord_dim != 1:
                raise ValueError(
                    f"[ERROR]: Preferred joint pose entry '{pose_name}' maps to body '{body_name}' "
                    f"with {coord_dim} coordinates; only 1-DoF joints are supported.")

            target_q[coord_start] = pose_value
            coord_masks[coord_start] = float(self.preferred_joint_pose_weights.get(pose_name, 1.0))

        return target_q, coord_masks

    def _create_ik_objectives(self, num_envs, model, state):
        newton.eval_fk(model, model.joint_q, model.joint_qd, state)

        # Gather default body position and rotation based on model state to initialize
        # position and rotation objectives
        num_body_link_pos = len(self.mapped_body_link_pos_data)
        num_body_link_rot = len(self.mapped_body_link_rot_data)
        pos_targets = np.zeros((num_envs, num_body_link_pos), dtype=wp.vec3)
        rot_targets = np.zeros((num_envs, num_body_link_rot), dtype=wp.quat)

        body_q = state.body_q.numpy()
        for env in range(num_envs):
            base = env * self.num_body_count
            for ee_idx, (link_idx, _, link_offset) in enumerate(self.mapped_body_link_pos_data):
                pos_targets[env, ee_idx] = body_q[base + link_idx][0:3]

            for ee_idx, (link_idx, _) in enumerate(self.mapped_body_link_rot_data):
                rot_wp = wp.quat(body_q[base + link_idx][3:7])
                rot_targets[env, ee_idx] = wp.normalize(rot_wp)

        pos_num_ees = len(self.mapped_body_link_pos_data)
        rot_num_ees = len(self.mapped_body_link_rot_data)
        pos_target_arrays, rot_target_arrays = [], []
        for ee_idx in range(pos_num_ees):
            pos_wp = wp.array(pos_targets[:, ee_idx], dtype=wp.vec3)
            pos_target_arrays.append(pos_wp)

        for ee_idx in range(rot_num_ees):
            rot_wp = wp.array(rot_targets[:, ee_idx], dtype=wp.vec4)
            rot_target_arrays.append(rot_wp)

        position_objectives = []
        for i, (link_idx, w, link_offset) in enumerate(self.mapped_body_link_pos_data):
            objective = ik.IKObjectivePosition(
                link_index=link_idx,
                link_offset=link_offset,
                target_positions=pos_target_arrays[i],
                weight=w)
            position_objectives.append(objective)

        rotation_objectives = []
        for i, (link_idx, w) in enumerate(self.mapped_body_link_rot_data):
            objective = ik.IKObjectiveRotation(
                link_index=link_idx,
                link_offset_rotation=wp.quat_identity(),
                target_rotations=rot_target_arrays[i],
                weight=w)
            rotation_objectives.append(objective)

        joint_limit_objective = ik.IKObjectiveJointLimit(
            joint_limit_lower=self.ik_model.joint_limit_lower,
            joint_limit_upper=self.ik_model.joint_limit_upper,
            weight=self.joint_limit_weight)

        # Weight is set to desired value once initialization frames have been processed
        smooth_joint_limiter_objective = IKSmoothJointFilter(
            joint_limit_lower=self.ik_model.joint_limit_lower,
            joint_limit_upper=self.ik_model.joint_limit_upper,
            weight=0.0,
            coord_masks=self.smooth_joint_filter_coord_masks)

        preferred_joint_pose_objective = None
        if self.preferred_joint_pose_target is not None and self.preferred_joint_pose_coord_masks is not None:
            preferred_joint_pose_objective = IKJointPosePreference(
                target_q=self.preferred_joint_pose_target,
                coord_masks=self.preferred_joint_pose_coord_masks,
                n_dofs=self.ik_model.joint_dof_count,
                weight=self.preferred_joint_pose_weight)

        return (
            position_objectives,
            rotation_objectives,
            joint_limit_objective,
            smooth_joint_limiter_objective,
            preferred_joint_pose_objective)
