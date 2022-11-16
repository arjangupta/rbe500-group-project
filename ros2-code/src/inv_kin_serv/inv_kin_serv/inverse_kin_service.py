import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# Service Node for inverse kinematics
class InverseKinService(Node):

    def __init__(self):
        super().__init__('inverse_kin_service')
        self.srv = self.create_service(Point, 'inverse_kin_service', self.calculate_inverse_kin)

    def calculate_inverse_kin(self, request, response):
        sum = request.x + request.y + request.z # this won't work

        return response


def main():
    rclpy.init()

    inverse_kin_service = InverseKinService()

    rclpy.spin(inverse_kin_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()