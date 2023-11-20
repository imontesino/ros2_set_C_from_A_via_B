import time
from example_interfaces.srv import SetBool
from rclpy.node import Node
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class B_node(Node):
    def __init__(self):
        super().__init__('B_node')


        mux_callback_group = MutuallyExclusiveCallbackGroup()

        self.set_C_srv = self.create_service(
            SetBool,
            'set_C_from_B',
            self.set_C_from_B
        )

        self.cli = self.create_client(
            SetBool,
            'set_C',
            callback_group=mux_callback_group
        )
        print('started B_node')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('C service not available, waiting again...')


    def set_C_from_B(self, request, response):
        print('recieved request')
        req = SetBool.Request()
        req.data = request.data
        print('calling C')
        future = self.cli.call_async(req)

        # clock this thread while waiting for the service to be called
        while rclpy.ok() and not future.done():
            time.sleep(0.001)

        result = future.result()
        print('finished C')
        response.success = result.success
        print('Done')
        return response

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = B_node()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
