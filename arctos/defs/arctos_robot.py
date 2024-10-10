# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from pathlib import Path
from typing import List, Optional

import carb
import numpy as np
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper

# def import_arctos(cfg_path, load_yaml, script_path, radius_reduction=0.001):
#     cfg_path = str((script_path / Path("./arctos_defs/arctos_config.yaml")).absolute())
#     robot_cfg = load_yaml(cfg_path)["robot_cfg"]

#     robot_cfg['kinematics']['usd_path'] = str(script_path / Path(robot_cfg['kinematics']['usd_path']))
#     robot_cfg['kinematics']['urdf_path'] = str(script_path / Path(robot_cfg['kinematics']['urdf_path']))

#     # TODO still having issues with no movement due to self collision
#     subdict=robot_cfg['kinematics']['collision_spheres']
#     if radius_reduction and subdict:
#         for k,v in subdict.items():
#             for sphere in (subdict[k]):
#                 sphere['radius'] *= radius_reduction

#     return robot_cfg, cfg_path


class Arctos(Robot):
    """[summary]

    Args:
        prim_path (str): [description]
        name (str, optional): [description]. Defaults to "arctos_robot".
        usd_path (Optional[str], optional): [description]. Defaults to None.
        position (Optional[np.ndarray], optional): [description]. Defaults to None.
        orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        end_effector_prim_name (Optional[str], optional): [description]. Defaults to None.
        gripper_dof_names (Optional[List[str]], optional): [description]. Defaults to None.
        gripper_open_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        gripper_closed_position (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        prim_path: str = "/World/arctos",
        name: str = "arctos_robot",
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        self.usd_path = f"{Path(__file__).parent}/robot.usd"
        self._prim_path = prim_path

        prim = get_prim_at_path(self._prim_path)
        self._end_effector = None
        self._gripper = None
        self.end_effector_prim_path = f"{self._prim_path}/gripper_assembly_outer"
        self.grip_dt = 0.015

        if not prim.IsValid():
            add_reference_to_stage(usd_path=self.usd_path, prim_path=self._prim_path)

        self.gripper_dof_names = ["slide_right", "slide_left"]
        self.gripper_open_position = np.array([0.0, 0.0]) / get_stage_units()
        self.gripper_closed_position = np.array([-self.grip_dt, -self.grip_dt])

        super().__init__(
            prim_path=self._prim_path,
            name=name,
            position=position,
            orientation=orientation,
            articulation_controller=None,
        )

    def set_robot(self) -> SingleManipulator:
        add_reference_to_stage(usd_path=self.usd_path, prim_path=self._prim_path)
        self._gripper = ParallelGripper(
            end_effector_prim_path=self.end_effector_prim_path,
            joint_prim_names=["slide_left", "slide_right"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([-self.grip_dt, -self.grip_dt]),
            action_deltas=np.array([self.grip_dt, -self.grip_dt]),
        )
        self._manipulator = SingleManipulator(
            prim_path=self._prim_path,
            name="arctos_robot",
            end_effector_prim_path=self.end_effector_prim_path,
            gripper=self._gripper,
        )
        joints_default_positions = np.zeros(8)
        joints_default_positions[6] = self.grip_dt
        joints_default_positions[7] = self.grip_dt
        self._manipulator.set_joints_default_state(positions=joints_default_positions)

        # self.initialize()
        return self._manipulator

    @property
    def end_effector(self) -> RigidPrim:
        """[summary]

        Returns:
            RigidPrim: [description]
        """
        return self._end_effector

    @property
    def gripper(self) -> ParallelGripper:
        """[summary]

        Returns:
            ParallelGripper: [description]
        """
        return self._gripper

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]"""
        super().initialize(physics_sim_view)
        self._end_effector = RigidPrim(
            prim_path=self.end_effector_prim_path, name=self.name + "_end_effector"
        )
        self._end_effector.initialize(physics_sim_view)
        self.gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self.dof_names,
        )
        return

    def post_reset(self) -> None:
        """[summary]"""
        super().post_reset()
        self._gripper.post_reset()
        self._articulation_controller.switch_dof_control_mode(
            dof_index=self._gripper.joint_dof_indicies[0], mode="position"
        )
        self._articulation_controller.switch_dof_control_mode(
            dof_index=self._gripper.joint_dof_indicies[1], mode="position"
        )
        return
