# RBE 500 Group Assignment #2
# Group 2 - Joshua Gross, Arjan Gupta, Melissa Kelly

import rclpy
from rclpy.node import Node
from rbe500_custom_interfaces.srv import ScaraRefPos
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Node for subscriber, publisher, service, and client
class ScaraPDController(Node):

    def __init__(self):
        # Initialize superclass
        super().__init__('scara_pd_controller')

        # Initialize member variables for reference position
        self.received_ref_pos: bool = False
        self.ref_q1: float = 0.0
        self.ref_q2: float = 0.0
        self.ref_q3: float = 0.0

        # Initialize member variables for joint_states subscription
        self.awaiting_ref_pos_count: int = 0

        # Create the service that receives the reference/goal position
        self.srv = self.create_service(ScaraRefPos, 'scara_pd_controller', self.set_ref)
        print("Done creating service that will receive reference/goal position.")

        # Create the subscriber that receives the joint state information, with a queue of 50 since it is very fast
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 100)
        print("Done creating subscription that receives joint-state information.")

        # Create the publisher that will send the joint efforts
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 10)
        print("Done creating the publisher for sending joint efforts.")
    
    def joint_states_callback(self, joint_state_msg):
        # Only take action if we have a reference/goal position to work against
        if self.received_ref_pos:
            self.run_controller(joint_state_msg)
        elif self.awaiting_ref_pos_count >= 100:
            print("Receiving joint state info. Awaiting a reference position to be given.")
            self.awaiting_ref_pos_count = 0
        else:
            self.awaiting_ref_pos_count += 1

    def run_controller(self, joint_state_msg):
        # Get joint position values
        q1 = joint_state_msg.position[0]
        q2 = joint_state_msg.position[1]
        q3 = joint_state_msg.position[2]
        # Get joint velocity values
        v1 = joint_state_msg.velocity[0]
        v2 = joint_state_msg.velocity[0]
        v3 = joint_state_msg.velocity[0]
        print("Controller is running!")
        print(f"Current q values are q1:{q1}, q2:{q2}, q3:{q3}")
        print(f"Current joint velocities are v1:{v1}, v2:{v2}, v3:{v3}")
    
    def set_ref(self, request, response):
        # Assign ref values
        self.ref_q1 = request.q1
        self.ref_q2 = request.q2
        self.ref_q3 = request.q3

        # Log to terminal that reference/goal position was received
        print(f"We received an reference position of x:{self.ref_q1} y:{self.ref_q2} z:{self.ref_q3}")

        # Set flag that we've received the goal position
        self.received_ref_pos = True
        print("We're ready to start our controller!")

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