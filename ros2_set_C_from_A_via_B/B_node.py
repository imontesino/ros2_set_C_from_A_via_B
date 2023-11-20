import argparse
import time
from example_interfaces.srv import SetBool
from rclpy.node import Node
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class B_node(Node):
    def __init__(self,
                 single_threaded=False):
        """Node for reproducing the callback deadlock error.

        Args:
            single_threaded (bool, optional): Use single threaded executor. Defaults to False.
        """
        super().__init__('B_node')
        self.single_threaded = single_threaded

        self.B_service = self.create_service(
            SetBool,
            'set_C_from_B',
            self.set_C_from_B
        )

        if single_threaded:
            # Each callback group will be executed in a single thread,
            # if no callback group is specified, the callback will be added to the default group
            self.C_client = self.create_client(
                SetBool,
                'set_C'
            )
        else:
            # Here we create a separate callback group for the C_client
            # This will allow the client to be called in a separate thread

            mux_callback_group = MutuallyExclusiveCallbackGroup()
            self.C_client = self.create_client(
                SetBool,
                'set_C',
                callback_group=mux_callback_group
            )

        if single_threaded:
            print('Started B_node in a single threaded executor')
        else:
            print('Started B_node in a multi threaded executor')

        while not self.C_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('C service not available, waiting again...')


    def set_C_from_B(self, request, response):
        """Callback for the B service.

        This will try to call the C_service from within the callback.
        It will not return until the C_service returns.
        """

        print('recieved request')
        req = SetBool.Request()
        req.data = request.data
        print('calling C')
        future = self.C_client.call_async(req)

        if self.single_threaded:
            print('Single Thread: Spinning until future complete (This will cause a deadlock)')
            # This will cause a deadlock
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
        else:
            # Block this thread while waiting for the service to be called
            # similar to rclpy.spin_until_future_complete(self, future)
            # without trting to spinning the executor
            print('Multi Thread: Waiting until future complete')
            while rclpy.ok() and not future.done():
                time.sleep(0.001)

        result = future.result()
        print('finished C')
        response.success = result.success
        print('Done')
        return response

def parse_args():
    """ Parse command line arguments. """

    parser = argparse.ArgumentParser(
        description="Select single or multithreaded to reproduce the calback deadlock error."
    )

    parser.add_argument('--single', action='store_true',
                        help='Use single threaded executor (This will cause a deadlock)')
    return parser.parse_known_args()

def main(args=None):
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)


    if args.single:
        print('Using single threaded executor')
        executor = SingleThreadedExecutor()
    else:
        print('Using multi threaded executor')
        executor = MultiThreadedExecutor()

    # This is the same as calling rclpy.spin(node) but it allows us to use the
    # executor instead of the default single threaded executor.
    node = B_node(single_threaded=args.single)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
