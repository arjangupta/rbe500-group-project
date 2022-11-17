import rclpy
from rclpy.node import Node
from rbe500_custom_interfaces.srv import InvKinSCARA

# Service Node for inverse kinematics
class InverseKinService(Node):

    def __init__(self):
        # Initialize superclass
        super().__init__('inverse_kin_service')

        # Define the link lengths as given in the Gazebo setup
        self.p1=2
        self.p2=1
        self.p3=1

        # Create the service
        self.srv = self.create_service(InvKinSCARA, 'inverse_kin_service', self.calculate_inverse_kin)

    def calculate_inverse_kin(self, request, response):
        # Log to terminal that position was received
        print(f"We received an end effector position of x:{request.x} y:{request.y} z:{request.z}")

        # ----- Calculate joint variables ------
        
        # As calculated in our report, q3 is simply the translation of the end effector in the negative z direction
        response.q3 = request.z - self.p1

        # To find q1 and q2, we will need to apply law of cosines. For this we need alpha, beta, gamma of the
        # triangle formed
        # alpha = 
        response.q1 = float(1.0)
        response.q2 = float(2.0)

        return response


def main():
    rclpy.init()

    inverse_kin_service = InverseKinService()

    rclpy.spin(inverse_kin_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()