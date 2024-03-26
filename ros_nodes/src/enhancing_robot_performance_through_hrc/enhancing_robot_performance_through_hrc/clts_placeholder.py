import rclpy
from .send_str_clt import SendStrClient
from custom_interfaces.srv import String
from rclpy.node import Node

class CltsPlaceholder(Node):
    def __init__(self):
        super().__init__('clients_place_holder')    
        self.quit_sim = SendStrClient("quit_sim")
        self.hold_left_sim = SendStrClient("hold_left_sim")
        self.hold_right_sim = SendStrClient("hold_right_sim")
        self.place_left_sim = SendStrClient("place_left")
        self.place_right_sim = SendStrClient("place_right")
        self.hold_joint_sim = SendStrClient("hold_joint_sim")
        self.reset_left = SendStrClient("reset_left")
        self.reset_right = SendStrClient("reset_right")
        self.reset_task = SendStrClient("reset_task")
        self.clts_list = [self.quit_sim, self.hold_left_sim, self.hold_right_sim, self.place_left_sim, self.place_right_sim, self.hold_joint_sim, self.reset_left, self.reset_right, self.reset_task]

def main(args=None):
    rclpy.init(args=args)
    send_pose_client = CltsPlaceholder()
    ans = None
    while ans != 0:
        for i, behavior in enumerate(send_pose_client.clts_list):
            print(i, behavior.get_name())
        ans = input("")
        send_pose_client.clts_list[int(ans)].send_request("")
        rclpy.spin_once(send_pose_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()