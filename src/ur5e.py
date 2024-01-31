from typing import List
from numpy import ndarray
from omni.isaac.core.robots import Robot
import numpy as np
import os
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.motion_generation.lula import RmpFlow
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.utils.types import ArticulationAction
import roboticstoolbox as rbt

class Ur5e(Robot):
    def __init__(self, name, world, translation=[0,0,0], prim_path=None):
        self.gripper_options = {'open': np.array([0, 0]), 'closed': np.array([0.4, 0.4])}
        self.gripper_status = 'open'
        # self.gripper_status = np.array([0, 0])
        # self.gripper_open = np.array([0, 0])
        # self.gripper_closed = np.array([0.4, 0.4])
        self.world = world
        self.holding_obj = False

        if prim_path is None:
            prim_path = "/World/" + name

        super().__init__(prim_path=prim_path, name=name)
        self.manipulator_controller = self.get_articulation_controller()
        
        rmp_config_dir = "/home/gu/.local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1/exts/omni.isaac.motion_generation/motion_policy_configs"
        self.rmpflow = RmpFlow(
                        robot_description_path=rmp_config_dir + "/universal_robots/ur5e/rmpflow/ur5e_robot_description.yaml",
                        urdf_path=rmp_config_dir + "/universal_robots/ur5e/ur5e.urdf",
                        rmpflow_config_path=rmp_config_dir + "/universal_robots/ur5e/rmpflow/ur5e_rmpflow_config.yaml",
                        end_effector_frame_name="tool0",
                        maximum_substep_size=1/300,
        )
        robot_base_translation, robot_base_orientation = self.get_world_pose()
        self.rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)

        self.lula_solver = self.rmpflow.get_kinematics_solver()

    # def get_joint_positions(self, joint_indices: List | ndarray | None = None) -> ndarray:
    #     return super().get_joint_positions(joint_indices)[:6]
    
    def get_tcp_pose(self):
        tcp_pose = self.rmpflow.get_end_effector_pose(self.get_joint_positions()[:6])
        return[tcp_pose[0], rot_utils.rot_matrices_to_quats(tcp_pose[1])]

    def close_gripper(self):
        self.gripper_status = 'closed'
        self.move_to_joint_position(np.hstack((self.get_joint_positions()[:6], 
                                               self.gripper_options[self.gripper_status])))

    def open_gripper(self):
        self.gripper_status = 'open'
        self.move_to_joint_position(np.hstack((self.get_joint_positions()[:6], 
                                               self.gripper_options[self.gripper_status])))

    def move_to_joint_position(self, q, t=100):
        """
        Move the robot to the specified joint position, by creating a 100 step long trajectory
        q must be 8 joint, the last two represents the gripper status, if you want to mantain the current one 
        just use self.gripper_status

        Args:
            q (np.array): joint values
            t (int, optional): step in the traj. Defaults to 100.
        """
        q = np.array(q)
        if q.shape[0] == 6:
            q = np.hstack((q, self.gripper_status))
        traj = rbt.jtraj(self.get_joint_positions(), q, t)
        for q in traj.q:
            # q = np.hstack((q, self.gripper_options[self.gripper_status]))
            action = ArticulationAction(joint_positions=q)
            self.manipulator_controller.apply_action(action)
            self.world.step(render=True)

    def move_to_cart_position(self, target_pos, target_orient=None):
        joint_tool = self.get_joint_positions()[:6]
        if target_orient is None:
            target_orient = rot_utils.rot_matrices_to_quats(self.rmpflow.get_end_effector_pose(joint_tool)[1])
        joint_pos, reachable = self.lula_solver.compute_inverse_kinematics(
                                                                        frame_name="tool0",
                                                                        warm_start=joint_tool,
                                                                        target_position=target_pos,
                                                                        target_orientation=target_orient
                                                                    )
        if reachable:
            self.move_to_joint_position(np.hstack((joint_pos, self.gripper_options[self.gripper_status])))

    def grab_object(self, obj_pose):
        self.open_gripper()
        self.move_to_cart_position(obj_pose[0], obj_pose[1])
        self.close_gripper()
        current_pose = self.get_tcp_pose()
        self.move_to_cart_position(current_pose[0] + np.array([0, 0, 0.1]), current_pose[1])
    
    def hold_object(self, obj_pose, hold_pose):
        self.grab_object(obj_pose)
        self.move_to_cart_position(hold_pose[0], hold_pose[1])

