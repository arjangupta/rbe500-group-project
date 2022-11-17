import rclpy
from rclpy.node import Node
from rbe500_custom_interfaces.srv import InvKinSCARA

# Service Node for inverse kinematics
class InverseKinService(Node):

    def __init__(self):
        super().__init__('inverse_kin_service')
        self.srv = self.create_service(InvKinSCARA, 'inverse_kin_service', self.calculate_inverse_kin)

    def calculate_inverse_kin(self, request, response):
        # Log to terminal that position was received
        print(f"We received an end effector position of x:{request.x} y:{request.y} z:{request.z}")

        # Calculate joint variables
        response.q1 = float(1.0)
        response.q2 = float(2.0)
        response.q3 = float(3.0)

        return response


def main():
    rclpy.init()

    inverse_kin_service = InverseKinService()

    rclpy.spin(inverse_kin_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()