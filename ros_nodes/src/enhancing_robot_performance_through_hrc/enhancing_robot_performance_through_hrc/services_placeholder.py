from custom_interfaces.srv import String
import rclpy
from rclpy.node import Node
import time

class SendPoseServer(Node):
    def __init__(self):
        super().__init__('send_pose_server')        
        self.hold_left_sim_srv = self.create_service(String, "hold_left_sim", self.string_callback)
        self.hold_left_real_srv = self.create_service(String, "hold_left_real", self.string_callback)
        self.hold_right_sim_srv = self.create_service(String, "hold_right_sim", self.string_callback)
        self.hold_right_real_srv = self.create_service(String, "hold_right_real", self.string_callback)
        self.placed_left_srv = self.create_service(String, "place_left", self.string_callback)
        self.placed_left_srv = self.create_service(String, "reset_left", self.string_callback)
        self.placed_right_srv = self.create_service(String, "place_right", self.string_callback)
        self.placed_right_srv = self.create_service(String, "reset_right", self.string_callback)
        self.hold_joint_sim_srv = self.create_service(String, "hold_joint_sim", self.string_callback)
        self.hold_joint_real_srv = self.create_service(String, "hold_joint_real", self.string_callback)
        self.task_done_srv = self.create_service(String, "reset_task", self.string_callback)
        self.task_done_srv = self.create_service(String, "complete_task", self.string_callback)

    def string_callback(self, request, response):
        print("Request: ", request)
        time.sleep(1)
        response.ans = "success"   
        return response

def main(args=None):
    rclpy.init(args=args)
    send_pose_server = SendPoseServer()

    rclpy.spin(send_pose_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()