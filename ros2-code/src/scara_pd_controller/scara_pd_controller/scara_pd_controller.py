# RBE 500 Group Assignment #2
# Group 2 - Joshua Gross, Arjan Gupta, Melissa Kelly

import rclpy
from rclpy.node import Node
from rbe500_custom_interfaces.srv import InvKinSCARA

# Node for subscriber, publisher, service, and client
class ScaraPDController(Node):

    def __init__(self):
        # Initialize superclass
        super().__init__('scara_pd_controller')

        # Create the service
        self.srv = self.create_service(InvKinSCARA, 'scara_pd_controller', self.calculate_inverse_kin)


def main():
    rclpy.init()

    scara_pd_controller = ScaraPDController()

    rclpy.spin(scara_pd_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()