import carb
import carb.events
import omni
from omni.isaac.kit import SimulationApp
import os
import argparse

parser = argparse.ArgumentParser(description="Ros2 Bridge Sample")
parser.add_argument(
    "--ros2_bridge",
    default="omni.isaac.ros2_bridge",
    nargs="?",
    choices=["omni.isaac.ros2_bridge", "omni.isaac.ros2_bridge-humble"],
)
args, unknown = parser.parse_known_args()
simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False, "window_width": 2000, "window_height":1500})
omni.usd.get_context().open_stage("thesis/props/scene.usd")
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
import omni
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_object_type
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects import DynamicCone
from omni.isaac.core.objects import VisualSphere
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import RotatingLidarPhysX
from omni.isaac.sensor import Camera
from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.objects import DynamicCylinder
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from omni.isaac.universal_robots import KinematicsSolver as Ur10KinematicsSolver
from omni.isaac.manipulators.grippers import ParallelGripper, SurfaceGripper
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import time
from math import atan2
import roboticstoolbox as rbt
from pxr import Usd
from ur5e import Ur5e

enable_extension(args.ros2_bridge)
import rclpy
from rclpy.node import Node

data_path = os.getcwd() + "/fluently/data/"

class Thesis(Node):
    def __init__(self):
        super().__init__("case")

        self.timeline = None
        self.world = None
        
        self.setup_world()
        self.world.reset()

    # ---------------- ISAAC ---------------- #
    def setup_world(self):
        """
        setup the world initializing all the needed objects
        """
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)

        self.stage = omni.usd.get_context().get_stage()
        
        self.robot = Ur5e("ur5e", self.world)
        self.world.scene.add(self.robot)

    # ---------------- MAIN SIMULATION ---------------- #
    def run_simulation(self):
        """
        nothing happens here, everything triggers trough ros2 from the bt
        """
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.world.is_playing():
                self.robot.move_to_joint_position([0, 0, 0, 0, 0, 0])
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    demo = Thesis()
    demo.run_simulation()
