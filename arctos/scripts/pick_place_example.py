# Third Party
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


# Standard Library
import sys
from pathlib import Path

sys.path.append(Path(__file__).parent.parent.as_posix())


# Third Party
import random

import numpy as np
import omni
from controllers.pick_place import PickPlaceController
from defs.arctos_robot import Arctos
from omni.isaac.core import World
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics, Vt
from tasks.pick_place import PickPlace

my_world = World(stage_units_in_meters=1.0)
# Add lighting
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateAngleAttr(50)
distantLight.CreateIntensityAttr(1000)


arctos_cls = Arctos()
arctos_manipulator = arctos_cls.set_robot()

cube_dims = [
    random.uniform(0.02, 0.5),
    random.uniform(0.015, arctos_cls.grip_dt - 0.001),
    0.02,
]
target_position = np.array([-0.2, 0.4, cube_dims[2] / 2])

my_task = PickPlace(
    name="arctos_pick_place",
    target_position=target_position,
    cube_initial_orientation=euler_angles_to_quat(
        [0.0, 0.0, random.uniform(-np.pi / 8, np.pi / 8)]
    ),
    cube_dims=cube_dims,
    robot=arctos_manipulator,
)
my_world.add_task(my_task)
my_world.reset()

task_params = my_world.get_task("arctos_pick_place").get_params()
arctos_name = task_params["robot_name"]["value"]
my_arctos = my_world.scene.get_object(arctos_name)

# initialize the controller
my_controller = PickPlaceController(
    name="controller", robot_articulation=my_arctos, gripper=my_arctos.gripper
)
# task_params = my_world.get_task("arctos_pick_place").get_params()
articulation_controller = my_arctos.get_articulation_controller()


task_params = my_world.get_task("arctos_pick_place").get_params()
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        cube_z_rotation = quat_to_euler_angles(
            observations[task_params["cube_name"]["value"]]["orientation"]
        )[2]
        end_effector_rotation = euler_angles_to_quat([0.0, np.pi, cube_z_rotation])

        # forward the observation values to the controller to get the actions
        task_params = my_world.get_task("arctos_pick_place").get_params()
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]][
                "position"
            ],
            placing_position=observations[task_params["cube_name"]["value"]][
                "target_position"
            ],
            current_joint_positions=observations[task_params["robot_name"]["value"]][
                "joint_positions"
            ],
            end_effector_orientation=end_effector_rotation,
            # This offset needs tuning as well
            end_effector_offset=np.array([0.00386, -0.00091, 0.15]),
        )
        if my_controller.is_done():
            ...

        articulation_controller.apply_action(actions)

simulation_app.close()
