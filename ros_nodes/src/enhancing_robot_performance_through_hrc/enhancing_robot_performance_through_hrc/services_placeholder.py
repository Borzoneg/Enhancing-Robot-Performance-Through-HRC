from custom_interfaces.srv import String
import rclpy
from rclpy.node import Node
import time

class SendPoseServer(Node):
    def __init__(self):
        super().__init__('services_place_holder')        
        self.hold_left_sim_srv = self.create_service(String, "hold_left_sim", self.string_callback_runnning)
        self.hold_left_real_srv = self.create_service(String, "hold_left_real", self.string_callback_runnning)
        self.hold_right_sim_srv = self.create_service(String, "hold_right_sim", self.string_callback_runnning)
        self.hold_right_real_srv = self.create_service(String, "hold_right_real", self.string_callback_runnning)
        self.placed_left_srv = self.create_service(String, "place_left", self.string_callback)
        self.placed_left_srv = self.create_service(String, "reset_left", self.string_callback)
        self.placed_right_srv = self.create_service(String, "place_right", self.string_callback)
        self.placed_right_srv = self.create_service(String, "reset_right", self.string_callback)
        self.hold_joint_sim_srv = self.create_service(String, "hold_joint_sim", self.string_callback)
        self.hold_joint_real_srv = self.create_service(String, "hold_joint_real", self.string_callback_runnning)
        self.reset_task_srv = self.create_service(String, "reset_task", self.string_callback)
        self.place_joint_srv = self.create_service(String, "place_joint", self.string_callback)
        self.quit_sim_srv = self.create_service(String, "quit_sim", self.quit_sim)
        self.quit_real_srv = self.create_service(String, "quit_real", self.quit_real)
        self.quit_req_sim = False
        self.quit_req_real = False

    def string_callback(self, request, response):
        print("Request: ", request)
        time.sleep(1)
        response.ans = "success"   
        return response
    
    def string_callback_runnning(self, request, response):
        print("Request: ", request)
        time.sleep(1)
        response.ans = "running"   
        return response
    
    def quit_sim(self, request, response):
        print("Request: ", request)
        time.sleep(1)
        response.ans = "running"   
        self.quit_req_sim = True
        return response
    
    def quit_real(self, request, response):
        print("Request: ", request)
        time.sleep(1)
        response.ans = "running"   
        self.quit_req_real = True
        return response

def main(args=None):
    rclpy.init(args=args)
    send_pose_server = SendPoseServer()
    while not (send_pose_server.quit_req_sim and send_pose_server.quit_req_real):
        rclpy.spin_once(send_pose_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()