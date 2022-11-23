# RBE 500 Group Assignment #2
# Group 2 - Joshua Gross, Arjan Gupta, Melissa Kelly

import rclpy
from rclpy.node import Node
from rbe500_custom_interfaces.srv import ScaraRefPos

# Node for subscriber, publisher, service, and client
class ScaraPDController(Node):

    def __init__(self):
        # Initialize superclass
        super().__init__('scara_pd_controller')

        # Initialize member variables
        self.received_ref_pos: bool = False

        # Create the service
        self.srv = self.create_service(ScaraRefPos, 'scara_pd_controller', self.set_ref)
    
    def set_ref(self, request, response):
        # Log to terminal that goal position was received
        print(f"We received an reference position of x:{request.x} y:{request.y} z:{request.z}")


def main():
    rclpy.init()

    scara_pd_controller = ScaraPDController()

    rclpy.spin(scara_pd_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()