from example_interfaces.srv import SetBool
from rclpy.node import Node
import rclpy

class A_node(Node):
    def __init__(self):
        super().__init__('A_node')
        self.cli = self.create_client(SetBool, 'set_C_from_B')
        print('started A_node')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('B service not available, waiting again...')

    def set_b(self):
        req = SetBool.Request()
        req.data = True
        future = self.cli.call_async(req)
        print('Calling B')
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        print(result)

def main(args=None):
    rclpy.init(args=args)
    node = A_node()
    node.set_b()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
