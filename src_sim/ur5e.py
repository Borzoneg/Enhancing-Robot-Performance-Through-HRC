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
    def __init__(self, name, world, translation=[0,0,0], prim_path=None, orientation=None):
        self.phys_queue = []
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
        rmp_config_dir = "./exts/omni.isaac.motion_generation/motion_policy_configs"
        self.rmpflow = RmpFlow(
                        robot_description_path=rmp_config_dir + "/universal_robots/ur5e/rmpflow/ur5e_robot_description.yaml",
                        urdf_path=rmp_config_dir + "/universal_robots/ur5e/ur5e.urdf",
                        rmpflow_config_path=rmp_config_dir + "/universal_robots/ur5e/rmpflow/ur5e_rmpflow_config.yaml",
                        end_effector_frame_name="tool0",
                        maximum_substep_size=1/300,
        )
        if orientation is not None:
            self.set_world_pose(orientation=orientation)

        robot_base_translation, robot_base_orientation = self.get_world_pose()
        self.rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)
        self.lula_solver = self.rmpflow.get_kinematics_solver()

    # def get_joint_positions(self, joint_indices: List | ndarray | None = None) -> ndarray:
    #     return super().get_joint_positions(joint_indices)[:6]
    
    def get_tcp_pose(self, q=None):
        if q is None:
            if len(self.phys_queue) == 0:
                tcp_pose = self.rmpflow.get_end_effector_pose(self.get_joint_positions()[:6])
            else:
                tcp_pose = self.rmpflow.get_end_effector_pose(self.phys_queue[-1][:6])
        else:
            tcp_pose = self.rmpflow.get_end_effector_pose(q)

        return[tcp_pose[0], rot_utils.rot_matrices_to_quats(tcp_pose[1])]

    def close_gripper(self):
        self.gripper_status = 'closed'
        if len(self.phys_queue) == 0:
            starting_j_pos = self.get_joint_positions()[:6]
        else:
            starting_j_pos = self.phys_queue[-1][:6]
        self.move_to_joint_position(np.hstack((starting_j_pos, 
                                               self.gripper_options[self.gripper_status])))

    def open_gripper(self):
        self.gripper_status = 'open'
        if len(self.phys_queue) == 0:
            starting_j_pos = self.get_joint_positions()[:6]
        else:
            starting_j_pos = self.phys_queue[-1][:6]  
        self.move_to_joint_position(np.hstack((starting_j_pos, 
                                               self.gripper_options[self.gripper_status])))

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
        if q.shape[0] == 6:
            q = np.hstack((q, self.gripper_options[self.gripper_status]))
        if len(self.phys_queue) == 0:
            starting_j_pos = self.get_joint_positions()
        else:
            starting_j_pos = self.phys_queue[-1]
        traj = rbt.jtraj(starting_j_pos, q, t)
        # self.phys_queue.append(q for q in traj.q)
        for q in traj.q:
              self.phys_queue.append(q)
        #     action = ArticulationAction(joint_positions=q)
        #     self.manipulator_controller.apply_action(action)
        #     self.world.step(render=True)

    def move_to_cart_position(self, target_pose, t=200):
        if len(self.phys_queue) == 0:
            starting_j_pos = self.get_joint_positions()[:6]
        else:
            starting_j_pos = self.phys_queue[-1][:6]
        target_pose[0] += np.array([0, 0, 0.12])
        joint_pos, reachable = self.lula_solver.compute_inverse_kinematics(
                                                                            frame_name="tool0",
                                                                            warm_start=starting_j_pos,
                                                                            target_position=target_pose[0],
                                                                            target_orientation=target_pose[1])
        if reachable:
            self.move_to_joint_position(np.hstack((joint_pos, self.gripper_options[self.gripper_status])), t=t)
        return reachable

    def move_to_target(self, frame):
        # distance = np.linalg.norm(self.get_tcp_pose()[0] - frame.get_world_pose()[0])
        # if distance > 0.2:
        #     print("distance: ", distance)
        #     self.move_to_cart_position(list(frame.get_world_pose()), t=2)
        self.move_to_cart_position(list(frame.get_world_pose()), t=2)


    def grab_object(self, obj_pose, use_jspace=False):
        self.open_gripper()
        if not use_jspace:
            # move over the target to approach from atop
            obj_pose[0] += np.array([0, 0, 0.1])
            self.move_to_cart_position(obj_pose)
            # move on the object
            obj_pose[0] -= np.array([0, 0, 0.1])
            self.move_to_cart_position(obj_pose)
        else:
            self.move_to_joint_position(obj_pose)
            current_pose_tcp = self.get_tcp_pose(obj_pose)
            current_pose_tcp[0] -= np.array([0, 0, 0.1])
            self.move_to_cart_position(current_pose_tcp)
        self.close_gripper()
        current_pose_tcp = self.get_tcp_pose()
        # move up to lift the object
        current_pose_tcp[0] += np.array([0, 0, 0.1])
        self.move_to_cart_position(current_pose_tcp)
    
    def hold_object(self, obj_pose, hold_pose, use_jspace_obj=False, use_jspace_hold=False):
        self.grab_object(obj_pose, use_jspace=use_jspace_obj)
        if not use_jspace_hold:
            print(hold_pose[0], hold_pose[1])
            self.move_to_cart_position(hold_pose[0], hold_pose[1])
        else:
            self.move_to_joint_position(hold_pose)

    def physisc_step(self):
        try:
            req_j = self.phys_queue.pop(0)
            action = ArticulationAction(joint_positions=req_j)
            self.manipulator_controller.apply_action(action)
        except IndexError:
            # print("No trajectory in queue")
            pass