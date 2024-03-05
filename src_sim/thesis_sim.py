import carb.events
import omni
from omni.isaac.kit import SimulationApp
import os
import sys

sys.path.append("/home/fluently/.local/share/ov/pkg/isaac_sim-2023.1.1/Enhancing-Robot-Performance-Through-HRC/ros_nodes/src/enhancing_robot_performance_through_hrc/enhancing_robot_performance_through_hrc/")

simulation_app = SimulationApp({"headless": False, "window_width": 2000, "window_height":1500})
omni.usd.get_context().open_stage("./Enhancing-Robot-Performance-Through-HRC/props/scene_.usd")
# omni.usd.get_context().open_stage("./Enhancing-Robot-Performance-Through-HRC/props/flat_scene.usd")
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading
while is_stage_loading():
    simulation_app.update()

import omni
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

        self.setup_world()
        self.world.reset()

        self.done = False
        # part 1
        # cartesian:  [array([0.17888019, 0.47636152, 1.00502915]), array([-9.99220787e-04,  4.42089397e-05,  9.99999497e-01,  7.00350582e-05])]
        # joint:  [ 0.985967  -1.1072518  1.5933292  1.0867094  1.571023  -1.7541112]

        # over part 1
        # cartesian:  [array([0.17905214, 0.47633416, 1.07132045]), array([-1.00158970e-03,  9.03089518e-06,  9.99999494e-01,  8.99679595e-05])]
        # joint:  [ 0.9859685 -1.1689541  1.5012057  1.2405472  1.5709847 -1.7541801]

        # hold left
        
        self.part1_pose = [np.array([-0.0235979 , 0.46189392, 0.98592044]), np.array([-5.20024701e-03, 2.25993284e-02, 9.99730923e-01, -5.55867726e-04])]
        self.part1_pose_joint = np.array([0.9641214, -1.5162885,  2.1554742, -2.2067714,  4.712087,  -4.917703])
        self.part2_pose_joint = np.array([9.0818042e-01, -1.7856942e+00, 2.4241664e+00, -2.2065356e+00, 4.7122583e+00, -4.9737353e+00])
        self.available_parts = [[self.part1_pose_joint, True], [self.part2_pose_joint, True]]

        self.left_hold_pose_joint = np.array([6.1565105e-02, -7.8946501e-01, 1.0186660e+00, -1.7937291e+00, 4.7132058e+00, -5.8202195e+00])

        # self.world.get_physics_context().enable_gpu_dynamics(True)
        self.hold_left_sim_srv = self.create_service(String, 'hold_left_sim', self.hold_left)
        self.place_left_srv = self.create_service(String, 'place_left', self.place_left)
        self.reset_left_srv = self.create_service(String, 'reset_left', self.reset_left)
        self.hold_right_sim_srv = self.create_service(String, 'hold_right_sim', self.hold_right)
        self.place_right_srv = self.create_service(String, 'place_right', self.place_right)
        self.reset_right_srv = self.create_service(String, 'reset_right', self.reset_right)
        self.hold_joint_sim_srv = self.create_service(String, 'hold_joint_sim', self.hold_joint)
        self.reset_task_srv = self.create_service(String, 'reset_task', self.reset_task)
    
    # ---------------- ISAAC ---------------- #
    def setup_world(self):
        """
        setup the world initializing all the needed objects
        """
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        # self.world.get_physics_context().enable_gpu_dynamics(True)
        self.stage = omni.usd.get_context().get_stage()
        
        self.robot = Ur5e("ur5e", self.world, orientation=rot_utils.euler_angles_to_quats([0, 0, -67], degrees=True))
        self.target_pose = self.world.scene.add(VisualSphere(prim_path="/World/target", name="target", color=np.array([0, 0, 1]),
                                                             translation=np.array([0.2, 0, 1]), 
                                                             orientation=rot_utils.euler_angles_to_quats([0, np.pi, 0]),
                                                             scale=np.array([0.01, 0.01, 0.01]), visible=True))
        self.part1 = self.world.scene.add(XFormPrim(prim_path="/World/Thesis_part", name="Thesis_part", scale=[0.01, 0.01, 0.01]))
        self.world.scene.add(self.robot)
    
    # ----------------- ROS ----------------- #
    def hold_left(self, request, response):
        self.logger.info("Received request for hold left service " + str(request))
        for part in self.available_parts:
            if part[1]: # if the part is available
                self.robot.hold_object(part[0], self.left_hold_pose_joint, use_jspace_hold=True, use_jspace_obj=True)
                part[1] = False
        response.ans = "success"
        return response
    
    def place_left(self, request, response):
        self.logger.info("Received request for place left service " + str(request))
        response.ans = "success"
        return response
    
    def reset_left(self, request, response):
        self.logger.info("Received request for reset left service " + str(request))
        response.ans = "success"
        return response
    
    def hold_right(self, request, response):
        self.logger.info("Received request for hold right service " + str(request))
        print("Request: ", request)
        for part in self.available_parts:
            if part[1]: # if the part is available
                self.robot.hold_object(part[0], self.right_hold_pose_joint, use_jspace_hold=True, use_jspace_obj=True)
                part[1]= False
        response.ans = "success"
        return response

    def place_right(self, request, response):
        self.logger.info("Received request for place right service " + str(request))
        response.ans = "success"
        return response
   
    def reset_right(self, request, response):
        self.logger.info("Received request for reset right service " + str(request))
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
        response.ans = "success"
        return response
   
    # ----------- MAIN SIMULATION ----------- #
    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.world.is_playing():
                self.world.step(render=True)
                self.robot.physisc_step()
                if self.world.current_time_step_index == 300:
                    self.robot.hold_object(self.part1_pose_joint, self.left_hold_pose_joint, use_jspace_obj=True, use_jspace_hold=True)
                # if self.world.current_time_step_index > 300:
                    # self.robot.move_to_target(self.target_pose)
                print("cartesian: ", self.robot.get_tcp_pose())
                print("joint: ", self.robot.get_joint_positions()[:6])
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()

def main():
    rclpy.init()
    demo = ThesisSim()
    demo.run_simulation()

if __name__ == "__main__":
    main()