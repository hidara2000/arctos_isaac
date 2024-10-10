# Standard Library
from typing import Optional

# Third Party
import numpy as np
import omni.isaac.core.tasks as tasks
from omni.isaac.manipulators import SingleManipulator


class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        cube_dims: np.ndarray,
        name: str = "arctos_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        robot: SingleManipulator = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=np.array([*cube_dims]),
            offset=offset,
        )
        self.manipulator = robot

    def set_robot(self) -> SingleManipulator:
        return self.manipulator
