# Third Party
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


# Standard Library
import sys
from pathlib import Path

# Third Party
import carb
import numpy as np
from omni.isaac.core import World

sys.path.append(Path(__file__).parent.parent.as_posix())


# Third Party
from controllers.rmpflow import RMPFlowController
from tasks.follow_target import FollowTarget
from defs.arctos_robot import Arctos

my_world = World(stage_units_in_meters=1.0)

arctos_cls = Arctos()
arctos_manipulator = arctos_cls.set_robot()

# Initialize the Follow Target task with a target location for the cube to be followed by the end effector
my_task = FollowTarget(name="arctos_follow_target", target_position=np.array([1, 0, 0]), robot=arctos_manipulator)
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("arctos_follow_target").get_params()
target_name = task_params["target_name"]["value"]
arctos_name = task_params["robot_name"]["value"]
my_arctos = my_world.scene.get_object(arctos_name)

# initialize the controller
my_controller = RMPFlowController(
    name="target_follower_controller", robot_articulation=my_arctos
)
my_controller.reset()
articulation_controller = my_arctos.get_articulation_controller()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()

        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        articulation_controller.apply_action(actions)
simulation_app.close()
simulation_app.close()
