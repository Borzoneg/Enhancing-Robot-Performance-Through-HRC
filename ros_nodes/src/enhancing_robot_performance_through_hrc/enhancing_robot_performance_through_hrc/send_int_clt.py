from custom_interfaces.srv import GetInt 
import rclpy
from rclpy.node import Node

class SendIntClient(Node):
    def __init__(self, name=None):
        super().__init__(name + "_client")
        if name is None:
            name = 'generic_send_int'
        self.cli = self.create_client(GetInt, name)
        
        while not self.cli.wait_for_service(timeout_sec=5):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetInt.Request()

    def send_request(self, data):
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()