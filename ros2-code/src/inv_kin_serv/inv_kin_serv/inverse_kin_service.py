import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

# Service Node for inverse kinematics
class InverseKinService(Node):

    def __init__(self):
        super().__init__('inverse_kin_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.calculate_inverse_kin)

    def calculate_inverse_kin(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    inverse_kin_service = InverseKinService()

    rclpy.spin(inverse_kin_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()