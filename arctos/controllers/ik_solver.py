from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from omni.isaac.core.articulations import Articulation
from typing import Optional
from pathlib import Path

class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        #TODO: change the config path
        root_dir = Path(f"{Path.home()}/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.examples/omni/isaac/examples/user_examples/arctos/defs")
        self._kinematics = LulaKinematicsSolver(robot_description_path=f"{root_dir}/robot_descriptor.yaml",
                                                urdf_path=f"{root_dir}/robot.urdf")
        if end_effector_frame_name is None:
            end_effector_frame_name = "gripper_assembly_outer"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
        return