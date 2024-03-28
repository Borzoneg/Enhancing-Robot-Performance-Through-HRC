# from typing import List
from numpy import ndarray
from omni.isaac.core.robots import Robot
import numpy as np
import os
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.motion_generation.lula import RmpFlow
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.utils.types import ArticulationAction
import roboticstoolbox as rbt
import spatialmath as sm

class Ur5e(Robot):
    def __init__(self, name, world, position=None, prim_path=None, orientation=None, gripper_articulation=2):
        self.phys_queue = []
        self.has_gripper = True if (not gripper_articulation == 0) else False
        self.gripper_articulation = gripper_articulation
        self.gripper_status = np.full(self.gripper_articulation, 0)
        self.world = world
        self.holding_obj = False

        if prim_path is None:
            prim_path = "/World/" + name

        super().__init__(prim_path=prim_path, name=name)
        self.manipulator_controller = self.get_articulation_controller()
        rmp_config_dir = "./exts/omni.isaac.motion_generation/motion_policy_configs"
        self.rmpflow = RmpFlow(
                        robot_description_path=rmp_config_dir + "/universal_robots/ur5e/rmpflow/ur5e_robot_description.yaml",
                        urdf_path=rmp_config_dir + "/universal_robots/ur5e/ur5e.urdf",
                        rmpflow_config_path=rmp_config_dir + "/universal_robots/ur5e/rmpflow/ur5e_rmpflow_config.yaml",
                        end_effector_frame_name="tool0",
                        maximum_substep_size=1/300,
        )
        if position is not None:
            self.set_world_pose(position=position)
        if orientation is not None:
            self.set_world_pose(orientation=orientation)

        robot_base_translation, robot_base_orientation = self.get_world_pose()
        self.rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)
        self.lula_solver = self.rmpflow.get_kinematics_solver()
        
        self.last_target_pose = None

   
    def get_tcp_pose(self, q=None):
        if q is None:
            if len(self.phys_queue) == 0:
                tcp_pose = self.rmpflow.get_end_effector_pose(self.get_joint_positions()[:6])
            else:
                tcp_pose = self.rmpflow.get_end_effector_pose(self.phys_queue[-1][:6])
        else:
            tcp_pose = self.rmpflow.get_end_effector_pose(q)
        actual_tcp_pose = [tcp_pose[0], rot_utils.rot_matrices_to_quats(tcp_pose[1])]
        if self.has_gripper:
            actual_tcp_pose[0] -= np.array([0, 0, 0.12])
        return actual_tcp_pose
    
    def from_world_to_base_frame(self, pose):
        """
        given a pose in form of [position, quaternion] gives basck the pose in base frame of the robot
        """
        T_obj_world = sm.SE3.Rt(rot_utils.quats_to_rot_matrices(pose[1]), pose[0])
        T_robot_world = sm.SE3.Rt(rot_utils.quats_to_rot_matrices(self.get_world_pose()[1]), self.get_world_pose()[0])
        T_obj_robot = T_obj_world * T_robot_world
        pose_obj_robot = [T_obj_robot.t, rot_utils.rot_matrices_to_quats(T_obj_robot.R)]
        return pose_obj_robot

    def close_gripper(self, force=None):
        if force is None:
            force = 0.5
        if len(self.phys_queue) == 0:
            starting_j_pos = self.get_joint_positions()[:6]
        else:
            starting_j_pos = self.phys_queue[-1][:6]
        self.gripper_status = np.full(self.gripper_articulation, force)
        self.move_to_joint_position(np.hstack((starting_j_pos, 
                                               self.gripper_status)), t=10)

    def open_gripper(self):
        if len(self.phys_queue) == 0:
            starting_j_pos = self.get_joint_positions()[:6]
        else:
            starting_j_pos = self.phys_queue[-1][:6]  
        self.gripper_status = np.full(self.gripper_articulation, 0)
        self.move_to_joint_position(np.hstack((starting_j_pos, 
                                               self.gripper_status)), t=10)

    def move_to_joint_position(self, q, t=200):
        """
        Move the robot to the specified joint position, by creating a 100 step long trajectory
        q must be 8 joint, the last two represents the gripper status, if you want to mantain the current one 
        just use self.gripper_status

        Args:
            q (np.array): joint values
            t (int, optional): step in the traj. Defaults to 100.
        """
        q = np.array(q)
        if len(self.phys_queue) == 0:
            starting_j_pos = self.get_joint_positions()
        else:
            starting_j_pos = self.phys_queue[-1]
        if q.shape[0] == 6:
            q = np.hstack((q, self.gripper_status))
        traj = rbt.jtraj(starting_j_pos, q, t)
        for q in traj.q:
              self.phys_queue.append(q)

    def move_to_cart_position(self, target_pose, t=200):       
        target_pose_copy = [np.copy(target_pose[0]), np.copy(target_pose[1])]
        if len(self.phys_queue) == 0:
            starting_j_pos = self.get_joint_positions()[:6]
        else:
            starting_j_pos = self.phys_queue[-1][:6]
        if self.has_gripper:
            target_pose_copy[0] += np.array([0, 0, 0.12])
        joint_pos, reachable = self.lula_solver.compute_inverse_kinematics(
                                                                            frame_name="tool0",
                                                                            warm_start=starting_j_pos,
                                                                            target_position=target_pose_copy[0],
                                                                            target_orientation=target_pose_copy[1])
        if reachable:
            self.move_to_joint_position((joint_pos), t=t)
        return reachable

    def move_to_target(self, frame):
        try:
            if not ((frame.get_world_pose()[0] == self.last_target_pose[0]).all() and (frame.get_world_pose()[1] == self.last_target_pose[1]).all()):
                self.move_to_cart_position(list(frame.get_world_pose()), t=2)
        except TypeError:
            self.move_to_cart_position(list(frame.get_world_pose()))
        self.last_target_pose = frame.get_world_pose()

    def move_up(self, offset):
        return self.move_to_cart_position([self.get_tcp_pose()[0] + np.array([0, 0, offset]), self.get_tcp_pose()[1]])

    def grab_object(self, obj_pose, use_jspace=False, force=None):
        self.open_gripper()
        if not use_jspace:
            # move over the target to approach from atop
            self.move_to_cart_position([obj_pose[0] + np.array([0, 0, 0.1]), obj_pose[1]])
            # move on the object
            self.move_up(-0.1)
        else:
            self.move_to_joint_position(obj_pose)
            self.move_to_cart_position([self.get_tcp_pose()[0] - np.array([0, 0, 0.3]), self.get_tcp_pose()[1]])
        self.close_gripper(force=force)
        # move up to lift the object
        self.move_to_cart_position([self.get_tcp_pose()[0] + np.array([0, 0, 0.2]), self.get_tcp_pose()[1]])
    
    def hold_object(self, obj_pose, hold_pose, use_jspace_obj=False, use_jspace_hold=False, force=None):
        self.grab_object(obj_pose, use_jspace=use_jspace_obj, force=force)
        
        if not use_jspace_hold:
            self.move_to_cart_position(hold_pose)
        else:
            self.move_to_joint_position(hold_pose)

    def move_to_home_position(self):
        self.open_gripper()
        self.move_to_joint_position(np.array([1.17, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2]))

    def flush_queue(self):
        self.phys_queue = []

    def physisc_step(self):
        try:
            req_j = self.phys_queue.pop(0)
            action = ArticulationAction(joint_positions=req_j)
            self.manipulator_controller.apply_action(action)
        except IndexError:
            # print("No trajectory in queue")
            pass