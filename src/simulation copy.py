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
omni.usd.get_context().open_stage("fluently/props/case_scene.usd")
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

enable_extension(args.ros2_bridge)
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import JointPoses
from custom_interfaces.srv import SendPose
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_srvs.srv import SetBool
from std_msgs.msg import String

using_windows = os.name == 'nt'
if using_windows:
    import msvcrt
else:
    rclpy.init()
simulation_app.update()

data_path = os.getcwd() + "/fluently/data/"

class Thesis(Node):
    def __init__(self):
        super().__init__("case")

        self.timeline = None
        self.ros_world = None
        self.manipulator = None
        self.manipulator_controller = None       
        self.gripper_status = np.array([0, 0])
        self.gripper_open = np.array([0, 0])
        self.gripper_closed = np.array([0.4, 0.4])
        
        self.setup_world()
       
        self.ros_world.reset()

    # ---------------- ISAAC ---------------- #
    def setup_world(self):
        """
        setup the world initializing all the needed objects
        """
        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)

        self.stage = omni.usd.get_context().get_stage()
        
        manipulator_prim_path = "/World/ur5e"
        # for prim in Usd.PrimRange(self.stage.GetPrimAtPath("/")):
        #     path = str(prim.GetPath())
        #     print(path)
        
        # the representation of the robot in the simulation
        self.manipulator = self.ros_world.scene.add(Robot(prim_path=manipulator_prim_path, name="manipulator"))
        
        self.manipulator_controller = self.manipulator.get_articulation_controller()

        mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
        rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
        self.rmpflow = RmpFlow(
            robot_description_path=rmp_config_dir + "/ur10/rmpflow/ur10_robot_description.yaml",
            urdf_path=rmp_config_dir + "/ur10/ur10_robot.urdf",
            rmpflow_config_path=rmp_config_dir + "/ur10/rmpflow/ur10_rmpflow_config.yaml",
            end_effector_frame_name="tool0",
            maximum_substep_size=1 / 60,
        )
        self.tcp = self.ros_world.scene.add(XFormPrim(prim_path=manipulator_prim_path + "/flange", name="flange"))
        self.pointing_vector = self.ros_world.scene.add(XFormPrim(prim_path="/World/arrow", name="arrow"))
        robot_base_translation, robot_base_orientation = self.manipulator.get_world_pose()
        self.rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)
        self.lula_solver = self.rmpflow.get_kinematics_solver()

    def close_gripper(self):
        self.move_to_joint_position(np.hstack((self.manipulator.get_joint_positions()[:6], 
                                               self.gripper_closed)))
        self.gripper_status = self.gripper_closed

    def open_gripper(self):
        self.move_to_joint_position(np.hstack((self.manipulator.get_joint_positions()[:6], 
                                               self.gripper_open)))
        self.gripper_status = self.gripper_open

    def move_to_joint_position(self, q, t=100):
        """
        Move the robot to the specified joint position, by creating a 100 step long trajectory
        q must be 8 joint, the last two represents the gripper status, if you want to mantain the current one 
        just use self.gripper_status

        Args:
            q (np.array): joint values
            t (int, optional): step in the traj. Defaults to 100.
        """
        if len(q) == 6:
            q = np.hstack((q, self.gripper_status))
        traj = rbt.jtraj(self.manipulator.get_joint_positions(), q, t)
        for q in traj.q:
            action = ArticulationAction(joint_positions=q)
            self.manipulator_controller.apply_action(action)
            self.ros_world.step(render=True)

    def move_to_cart_position(self, target_pos, target_orient=None):
        joint_tool = self.manipulator.get_joint_positions()[:6]
        if target_orient is None:
            target_orient = rot_utils.rot_matrices_to_quats(self.rmpflow.get_end_effector_pose(joint_tool)[1])
        joint_pos, reachable = self.lula_solver.compute_inverse_kinematics(
                                                                        frame_name="tool0",
                                                                        warm_start=joint_tool,
                                                                        target_position=target_pos,
                                                                        target_orientation=target_orient
                                                                    )
        if reachable:
            self.move_to_joint_position(np.hstack((joint_pos, self.gripper_status)))

    # ---------------- MAIN SIMULATION ---------------- #
    def run_simulation(self):
        """
        nothing happens here, everything triggers trough ros2 from the bt
        """
        self.timeline.play()
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.ros_world.is_playing():
                if self.ros_world.current_time_step_index == 100:
                    self.close_gripper()
                if self.ros_world.current_time_step_index == 200:
                    self.open_gripper()
                if self.ros_world.current_time_step_index == 300:
                    self.move_to_joint_position([0, 0, 0, 0, 0, 0])
                if self.ros_world.current_time_step_index == 400:
                    self.move_to_cart_position(np.array([0.3, 0.3, 0.3]))  
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    demo = Thesis()
    demo.run_simulation()
