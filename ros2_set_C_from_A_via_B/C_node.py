from example_interfaces.srv import SetBool
from rclpy.node import Node
import rclpy

class C_node(Node):
    def __init__(self):
        super().__init__('C_node')
        self.set_C_srv = self.create_service(SetBool, 'set_C', self.set_C)
        self.C = False
        print('started C_node')

    def set_C(self, request, response):
        print('recieved request')
        self.C = request.data
        print(f'self.C = {self.C}')
        response.success = True
        print('Done')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = C_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
