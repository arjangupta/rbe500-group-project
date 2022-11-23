# RBE 500 Group Assignment #2
# Group 2 - Joshua Gross, Arjan Gupta, Melissa Kelly

import rclpy
from rclpy.node import Node
from rbe500_custom_interfaces.srv import ScaraRefPos
from sensor_msgs.msg import JointState

# Node for subscriber, publisher, service, and client
class ScaraPDController(Node):

    def __init__(self):
        # Initialize superclass
        super().__init__('scara_pd_controller')

        # Initialize member variables for reference position
        self.received_ref_pos: bool = False
        self.ref_x: float = 0.0
        self.ref_y: float = 0.0
        self.ref_z: float = 0.0

        # Initialize member variables for joint_states subscription
        self.awaiting_ref_pos_count: int = 0

        # Create the service that receives the reference/goal position
        self.srv = self.create_service(ScaraRefPos, 'scara_pd_controller', self.set_ref)
        print("Done creating service")

        # Create the subscriber that receives the joint state information, with a queue of 50 since it is very fast
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 50)
        print("Done creating subscription")
    
    def joint_states_callback(self, joint_state_message):
        # Only take action if we have a reference/goal position to work against
        if self.received_ref_pos:
            print("We're ready to start our controller!")
        elif self.awaiting_ref_pos_count >= 100:
            print("Receiving joint state info. Awaiting a reference position to be given.")
            self.awaiting_ref_pos_count = 0
        else:
            self.awaiting_ref_pos_count += 1
    
    def set_ref(self, request, response):
        # Log to terminal that reference/goal position was received
        print(f"We received an reference position of x:{request.x} y:{request.y} z:{request.z}")

        # Assign ref values
        self.ref_x= request.x
        self.ref_y= request.y
        self.ref_z= request.z

        # Set flag that we've received the goal position
        self.received_ref_pos = True

        # Return the acknowledgement
        response.ok = True
        return response

def main():
    rclpy.init()

    scara_pd_controller = ScaraPDController()

    rclpy.spin(scara_pd_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()