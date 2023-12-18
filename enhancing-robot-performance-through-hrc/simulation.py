import sys
import os
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": (len(sys.argv) > 1),  # add anything after the call to the script to enable headless
                                "window_width": 2000,  # full screen: {w:2560; h:1440}
                                "window_height": 1080})

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects import DynamicCone
from omni.isaac.core.objects import VisualSphere
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import RotatingLidarPhysX
from omni.isaac.sensor import Camera
import numpy as np
import cv2
import msvcrt
import open3d as o3d
from scipy.spatial.transform import Rotation
from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.motion_generation import ArticulationKinematicsSolver
import omni.isaac.core.utils.numpy.rotations as rot_utils
import roboticstoolbox as rbt


assets_root_path = get_assets_root_path()
assert assets_root_path is not None, "Could not find nucleus server"
data_path = os.getcwd() + "/fluently/data/"
mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

x_offset = -2
z_table = 0.71
z_camera = 3.0


def setup_world():  
    """
    Setup a world with a table composed by two table sitting next to each other, a ur10e on one of the table and a cube on the other table.

    Returns:
        world (World): world object that represent the simulation
        ik_solver (ArticulationKinematicsSolver): the ik solver for the robot that we will use to control it
        rmpflow(RmpFlow): the environment representation with collision
        articulation_rmpflow (ArticulationMotionPolicy): articulation controller with collision
    """
    world = World()

    world.scene.add_default_ground_plane()

    manipulator_usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
    # table_usd_path = assets_root_path + "/Isaac/Environments/Simple_Room/Props/table_low.usd"
    table_usd_path = "fluently/props/table_low.usd"
    impeller_usd_path = "fluently/props/impeller.usd"
    # impeller_usd_path = "fluently/props/broken_impeller.usd"

    base_prim_path = "/World/base"
    table_prim_path = "/World/base/table"
    impeller_prim_path = "/World/base/impeller"
    manipulator_prim_path = "/World/base/manipulator"


    world.scene.add(XFormPrim(prim_path=base_prim_path, position=[x_offset, 0, 0]))
    add_reference_to_stage(usd_path=table_usd_path, prim_path=table_prim_path)
    add_reference_to_stage(usd_path=manipulator_usd_path, prim_path=manipulator_prim_path)
    add_reference_to_stage(usd_path=impeller_usd_path, prim_path=impeller_prim_path)
    world.scene.add(XFormPrim(prim_path=table_prim_path, name="table"))
    manipulator = world.scene.add(Robot(prim_path=manipulator_prim_path, name="manipulator", translation=[-0.6, 0, 0.67]))

    world.scene.add(VisualSphere(
                                    prim_path="/World/base/target",
                                    name="target",
                                    translation=np.array([0.2, 0, z_table + 0.5]),
                                    scale=np.array([0.01, 0.01, 0.01]),
                                    color=np.array([0, 1, 1]),
                                    visible=False,
                                    orientation=rot_utils.euler_angles_to_quats([np.pi, 0, 0])
                                )
                    )

    rmpflow = RmpFlow(
        robot_description_path=rmp_config_dir + "/ur10/rmpflow/ur10_robot_description.yaml",
        urdf_path=rmp_config_dir + "/ur10/ur10_robot.urdf",
        rmpflow_config_path=rmp_config_dir + "/ur10/rmpflow/ur10_rmpflow_config.yaml",
        end_effector_frame_name="tool0",
        maximum_substep_size=1/300,
    )
    robot_base_translation, robot_base_orientation = manipulator.get_world_pose()
    rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)
    physics_dt = 1 / 60
    articulation_rmpflow = ArticulationMotionPolicy(manipulator, rmpflow, physics_dt)
    plane_obstacle = FixedCuboid("/World/base/obstacles/plane_obstacle", scale=[1.6, 2.16, 0.01], translation=np.array([0.0, 0.54, z_table - 0.05]), visible=False)

    rmpflow.add_obstacle(plane_obstacle)

    lula_solver = rmpflow.get_kinematics_solver()
    ik_solver = ArticulationKinematicsSolver(manipulator, lula_solver, "tool0")

    return world, rmpflow, articulation_rmpflow, ik_solver, lula_solver


def robot_to_home_position(controller):
    """
    Reset robot to the home position

    Args:
        controller (ArticulationController): The controller of the robot that has to be reset to home position
    """
    controller.apply_action(ArticulationAction(
                                               joint_positions=np.array([np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]),
                                               ))


def create_pose(t, euler_angles):
    """
    Create a pose from a translation xyz and a rotation in euler angle

    Args:
        xyz (tuple(float, float, float)): position of the pose
        euler_angle (tuple(float, float, float): orientation of the pose in euler angle

    Returns:
        _type_: _description_
    """
    T = np.zeros((4, 4))
    R = Rotation.from_euler("xyz", [euler_angles[0], euler_angles[1], euler_angles[2] + np.pi/2], degrees=False)
    T[3, 3] = 1
    T[0:3, 3] = [t[0], t[1], t[2]]
    T[0:3, 0:3] = R.as_matrix()
    return T


world, rmpflow, articulation_rmpflow, ik_solver, lula_solver = setup_world()
world.reset()

manipulator = world.scene.get_object("manipulator")
manipulator_controller = manipulator.get_articulation_controller()

target_pose = world.scene.get_object("target")
""" SIMULATION """
while simulation_app.is_running():
    world.step(render=True)

    if world.current_time_step_index == 100:
        robot_to_home_position(manipulator_controller)

    if world.current_time_step_index == 200:
        print("[{:07.2f}] [{:06d}] [INPUT] Scenario loaded".format(world.current_time, world.current_time_step_index))

    if world.current_time_step_index > 200:  # after the world has been setup properly we start the detection
        pass
            
    if msvcrt.kbhit():
        cmd = msvcrt.getch()
        if cmd == b'q':
            done = False
