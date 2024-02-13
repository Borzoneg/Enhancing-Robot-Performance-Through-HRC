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
omni.usd.get_context().open_stage("./Enhancing-Robot-Performance-Through-HRC/props/flat_scene.usd")
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading
while is_stage_loading():
    simulation_app.update()

import omni
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World
from omni.isaac.core.objects import VisualSphere
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np
from ur5e import Ur5e

enable_extension(args.ros2_bridge)
import rclpy
from rclpy.node import Node

data_path = os.getcwd() + "./Enhancing-Robot-Performance-Through-HRC/data/"

class Thesis(Node):
    def __init__(self):
        super().__init__("case")

        self.timeline = None
        self.world = None
        
        self.setup_world()
        self.world.reset()
        # closest part to robot [array([-0.02495248,  0.45746576,  0.98969998]), array([-1.10622863e-03,  9.99998467e-01,  5.44925846e-04, -1.24342641e-03])]
        self.part1_pose = np.array([ 7.6579368e-01, -2.1491818e+00, 2.6174536e+00, -2.0349963e+00, 4.7113948e+00, -5.07814e+00])
        # over left [array([ 0.22879327, -0.23200678,  1.08989936]), array([-3.03280679e-03,  9.99994477e-01,  2.28049964e-04, -1.34053906e-03])]
        self.left_hold_pose = np.array([6.1565105e-02, -7.8946501e-01,  1.0186660e+00, -1.7937291e+00, 4.7132058e+00, -5.8202195e+00])
    # ---------------- ISAAC ---------------- #
    def setup_world(self):
        """
        setup the world initializing all the needed objects
        """
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)

        self.stage = omni.usd.get_context().get_stage()
        
        self.robot = Ur5e("ur5e", self.world)
        self.target_pose = self.world.scene.add(VisualSphere(prim_path="/World/target", name="target", color=np.array([0, 0, 1]),
                                                             translation=np.array([0.2, 0, 1]), 
                                                             orientation=rot_utils.euler_angles_to_quats([0, np.pi, 0]),
                                                             scale=np.array([0.01, 0.01, 0.01]), visible=True))
        self.world.scene.add(self.robot)

    # ---------------- MAIN SIMULATION ---------------- #
    def run_simulation(self):
        """
        nothing happens here, everything triggers trough ros2 from the bt
        """
        done = False
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.world.is_playing():
                self.robot.hold_object(self.part1_pose, self.left_hold_pose, use_jspace=True)
                # if not done:
                    # self.robot.move_to_joint_position(self.part1_pose)
                    # done = True
                # self.target_pose.set_world_pose(position=[-0.02, 0.47, 0.87])
                # self.robot.move_to_target(self.target_pose)
                # self.robot.move_to_cart_position(self.part1_pose[0], self.part1_pose[1])
                # print(self.robot.get_joint_positions())
                # self.robot.move_to_cart_position(self.left_hold_pose[0], self.left_hold_pose[1])
                # print(self.robot.get_joint_positions())
                # break
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    demo = Thesis()
    demo.run_simulation()
