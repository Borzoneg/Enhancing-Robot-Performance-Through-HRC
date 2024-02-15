from custom_interfaces.srv import String
import rclpy
from rclpy.node import Node
import time

class SendPoseServer(Node):
    def __init__(self):
        super().__init__('send_pose_server')        
        self.hold_first_sim_srv = self.create_service(String, "hold_first_sim", self.string_callback)
        self.hold_first_real_srv = self.create_service(String, "hold_first_real", self.string_callback)
        self.hold_second_sim_srv = self.create_service(String, "hold_second_sim", self.string_callback)
        self.hold_second_real_srv = self.create_service(String, "hold_second_real", self.string_callback)
        self.placed_first_srv = self.create_service(String, "placed_first", self.string_callback)
        self.placed_second_srv = self.create_service(String, "placed_second", self.string_callback)
        self.hold_joint_sim_srv = self.create_service(String, "hold_joint_sim", self.string_callback)
        self.hold_joint_real_srv = self.create_service(String, "hold_joint_real", self.string_callback)
        self.task_done_srv = self.create_service(String, "task_done", self.string_callback)

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