# from custom_interfaces.srv import GetInt 
from custom_interfaces.srv import String
import rclpy
from rclpy.node import Node
import sys

class SendStrClient(Node):
    def __init__(self, name=None):
        super().__init__(name + "_client")
        if name is None:
            name = 'generic_send_int'
        self.cli = self.create_client(String, name)

        while not self.cli.wait_for_service(timeout_sec=5):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, data):
        self.req = String.Request()
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    print(len(sys.argv))
    if len(sys.argv) != 3:
        print("Usage: ros2 run <package> <exe> service_name string_data")
        return
    clt = SendStrClient(sys.argv[1])
    clt.send_request(sys.argv[2])

if __name__ == '__main__':
    main()
