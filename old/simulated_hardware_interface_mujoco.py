import os
import numpy as np
import mujoco
import mujoco_viewer

import time

from data_types import *

class SimulatedHardwareInterface():
    """
    Class that runs a Mujoco simulation and provides an interface to the simulated robot.
    """
    def __init__(self):
        xml_path = os.path.join('urdf', 'rhea.urdf')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        # Add a ground plane
        self.model.add_geom(
            name="ground",
            type=mujoco.GEOM_PLANE,
            size=[1, 1, 1],
            rgba=[0.8, 0.9, 0.8, 1],
            pos=[0, 0, 0],
            quat=[1, 0, 0, 0],
            contype=1,
            conaffinity=0,
            condim=3,
            friction=[1, 0.5, 0.1],
            solimp=[0.9, 0.8, 0.01],
            solref=[0.02, 1],
        )


    def update(self, action):
        if self.viewer.is_alive:
            mujoco.mj_step(self.model, self.data)
            print(self.data.geom_xpos)
            self.viewer.render()
        else:
            raise Exception("Viewer is not alive.")

if __name__ == "__main__":
    interface = SimulatedHardwareInterface()
    while True:
        interface.update(action=np.zeros(1, dtype=action_dtype))
        time.sleep(0.01)