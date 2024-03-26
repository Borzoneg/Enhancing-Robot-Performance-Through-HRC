import carb.events
import omni
from omni.isaac.kit import SimulationApp
import os
import sys
sys.path.append(os.path.expanduser("~") + "/.local/share/ov/pkg/isaac_sim-2023.1.1/Enhancing-Robot-Performance-Through-HRC/ros_nodes/src/enhancing_robot_performance_through_hrc/enhancing_robot_performance_through_hrc/")

simulation_app = SimulationApp({"headless": False, "window_width": 2000, "window_height":1500})
omni.usd.get_context().open_stage("./Enhancing-Robot-Performance-Through-HRC/props/scene.usd")
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading
while is_stage_loading():
    simulation_app.update()

from omni.isaac.core import PhysicsContext
from omni.isaac.core import World
from omni.isaac.core.objects import VisualSphere
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.prims import XFormPrim
import numpy as np

from ur5e import Ur5e
from log_manager import CustomLogger

import rclpy
from rclpy.node import Node
from custom_interfaces.srv import String

data_path = os.getcwd() + "./Enhancing-Robot-Performance-Through-HRC/data/"

class ThesisSim(Node):
    def __init__(self):
        super().__init__("case")

        self.timeline = None
        self.world = None
        
        self.logger = CustomLogger("Simulation", os.path.expanduser('~') + 
                                   "/.local/share/ov/pkg/isaac_sim-2023.1.1/Enhancing-Robot-Performance-Through-HRC/logs/sim.log",
                                   overwrite=True)
        self.logger.info("="*10 + "Starting simulation" + "="*10)
        
        self.part1_grab_pose = ([np.array([0.25,  0.50, 0.9]), rot_utils.euler_angles_to_quats([np.pi, 0, np.pi])])
        self.part2_grab_pose = ([np.array([0.10,  0.50, 0.9]), rot_utils.euler_angles_to_quats([np.pi, 0, np.pi])])

        self.left_hold_pose =   ([np.array([0.25, -0.25, 1.0]), rot_utils.euler_angles_to_quats([np.pi, 0, np.pi])])
        self.right_hold_pose =  ([np.array([0.25, -0.05, 1.0]), rot_utils.euler_angles_to_quats([np.pi, 0, np.pi])])

        self.setup_world()
        self.world.reset()

        self.quit_request = False


       

        self.hold_left_sim_srv = self.create_service(String, 'hold_left_sim', self.hold_left)
        self.place_left_srv = self.create_service(String, 'place_left', self.place_left)
        self.reset_left_srv = self.create_service(String, 'reset_left', self.reset_left)
        self.hold_right_sim_srv = self.create_service(String, 'hold_right_sim', self.hold_right)
        self.place_right_srv = self.create_service(String, 'place_right', self.place_right)
        self.reset_right_srv = self.create_service(String, 'reset_right', self.reset_right)
        self.hold_joint_sim_srv = self.create_service(String, 'hold_joint_sim', self.hold_joint)
        self.reset_task_srv = self.create_service(String, 'reset_task', self.reset_task)
        self.reset_task_srv = self.create_service(String, 'quit_sim', self.quit)
    
    # ---------------- ISAAC ---------------- #
    def setup_world(self):
        """
        setup the world initializing all the needed objects
        """
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.world.get_physics_context().enable_gpu_dynamics(True)
        self.stage = omni.usd.get_context().get_stage()
        
        self.robot = Ur5e("ur5e", self.world, orientation=rot_utils.euler_angles_to_quats([0, 0, -67], degrees=True), gripper_articulation=4)
        self.target_pose = self.world.scene.add(VisualSphere(prim_path="/World/target", name="target", color=np.array([0, 0, 1]),
                                                             translation=np.array([0.2, 0, 1]), 
                                                             orientation=rot_utils.euler_angles_to_quats([0, np.pi, 0]),
                                                             scale=np.array([0.01, 0.01, 0.01]), visible=True))
        self.part1 = self.world.scene.add(XFormPrim(prim_path="/World/Thesis_part_01", name="Thesis_part_01"))
        self.part2 = self.world.scene.add(XFormPrim(prim_path="/World/Thesis_part_02", name="Thesis_part_02"))
         
        self.parts_state = [{'obj': self.part1, 'starting_pose': self.part1.get_world_pose(), 'grab_pose': self.part1_grab_pose, 'state': 'start'},
                            {'obj': self.part2, 'starting_pose': self.part2.get_world_pose(), 'grab_pose': self.part2_grab_pose, 'state': 'start'}]
                            
        self.world.scene.add(self.robot)
    
    # ----------------- ROS ----------------- #
    def hold_left(self, request, response):
        self.logger.info("Received request for hold left service " + str(request))
        for part in self.parts_state:
            if part['state'] == "start": # if the part is available
                self.robot.hold_object(part['grab_pose'], self.left_hold_pose, tight=True)
                part['state'] = "left"
                break
        response.ans = "success"
        return response
    
    def place_left(self, request, response):
        self.logger.info("Received request for place left service " + str(request))
        self.robot.move_up(-0.1)
        self.robot.open_gripper()
        self.robot.move_to_home_position()
        response.ans = "success"
        return response
    
    def reset_left(self, request, response):
        self.logger.info("Received request for reset left service " + str(request))
        for part in self.parts_state:
            if part['state'] == "left": # if the part is left
                print(part['starting_pose'])
                part['obj'].set_world_pose(position=part['starting_pose'][0], orientation=part['starting_pose'][1])
                part['state'] = "start"
                break
        response.ans = "success"
        return response
    
    def hold_right(self, request, response):
        self.logger.info("Received request for hold right service " + str(request))
        print("Request: ", request)
        for part in self.parts_state:
            if part['state'] == "start": # if the part is available
                self.robot.hold_object(part['grab_pose'], self.right_hold_pose, tight=True)
                part['state'] = "right"
                break
        response.ans = "success"
        return response

    def place_right(self, request, response):
        self.logger.info("Received request for place right service " + str(request))
        self.robot.move_up(-0.1)
        self.robot.open_gripper()
        self.robot.move_to_home_position()
        response.ans = "success"
        return response
   
    def reset_right(self, request, response):
        self.logger.info("Received request for reset right service " + str(request))
        for part in self.parts_state:
            if part['state'] == "right": # if the part is right
                part['obj'].set_world_pose(position=part['starting_pose'][0], orientation=part['starting_pose'][1])
                part['state'] = "start"
                break
        response.ans = "success"
        return response
   
    def hold_joint(self, request, response):
        self.logger.info("Received request for hold joint service " + str(request))
        response.ans = "success"
        return response
   
    def place_joint(self, request, response):
        self.logger.info("Received request for place joint service " + str(request))
        response.ans = "success"
        return response
   
    def reset_task(self, request, response):
        self.logger.info("Received request for reset joint service " + str(request))
        response.ans = "success"
        return response
   
    def quit(self, request, response):
        self.logger.info("Received request for quit service " + str(request))
        self.quit_request = True
        response.ans = "success"
        return response
   
    # ----------- MAIN SIMULATION ----------- #
    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running() and not self.quit_request:
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.world.is_playing():
                self.world.step(render=True)
                self.robot.physisc_step()
                if self.world.current_time_step_index == 100:
                    pass
                    
                
                if self.world.current_time_step_index > 300:
                    # self.robot.move_to_target(self.target_pose)
                    pass
                
                # print("joint: ", self.robot.get_joint_positions()[:6])
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()

def main():
    rclpy.init()
    demo = ThesisSim()
    demo.run_simulation()

if __name__ == "__main__":
    main()