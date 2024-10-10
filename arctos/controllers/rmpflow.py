# Standard Library
from pathlib import Path

# Third Party
import omni.isaac.motion_generation as mg
from omni.isaac.core.articulations import Articulation


class RMPFlowController(mg.MotionPolicyController):
    def __init__(
        self,
        name: str,
        robot_articulation: Articulation,
        physics_dt: float = 1.0 / 60.0,
    ) -> None:
        root_dir = Path(
            f"{Path.home()}/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.examples/omni/isaac/examples/user_examples/arctos/defs/"
        )

        self.rmpflow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=f"{root_dir}/robot_descriptor.yaml",
            rmpflow_config_path=f"{root_dir}/robot_rmpflow_config.yaml",
            urdf_path=f"{root_dir}/robot.urdf",
            end_effector_frame_name="gripper_assembly_outer",
            maximum_substep_size=0.001,
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(
            robot_articulation, self.rmpflow, physics_dt
        )

        mg.MotionPolicyController.__init__(
            self, name=name, articulation_motion_policy=self.articulation_rmp
        )
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
