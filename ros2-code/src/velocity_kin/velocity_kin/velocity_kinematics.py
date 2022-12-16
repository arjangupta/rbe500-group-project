from math import cos, sin
import rclpy
from rclpy.node import Node
from rbe500_custom_interfaces.srv import CalcScaraJointVelRefs
from sensor_msgs.msg import JointState
import numpy as np
import time

# Service Node for velocity kinematics
class ScaraVelocityKinematics(Node):

    def __init__(self):
        # Initialize superclass
        super().__init__('velocity_kin_service')

        # Define the link lengths as given in the Gazebo setup
        self.p1=2
        self.p2=1
        self.p3=1

        # Define joint variables
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.disp3 = 0

        # Define empty Jacobian
        self.Jacobian = np.zeros((6,3), dtype=float)

        # Create the service
        self.srv = self.create_service(CalcScaraJointVelRefs, 'velocity_inv_kin_service', self.calculate_inverse_velocity_kin)
        print("Done creating service that calculates inverse velocity kinematics.")

        # TODO: Forward velocity kin service

        # Create the subscriber that receives the joint state information, with a queue of 50 since it is very fast
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 100)
        print("Done creating subscription that receives joint-state information.")
    
    def calculate_jacobian(self):
        """
        This function solely computes our Jacobian, so it can be shared by both the forward and inverse velocity kinematics.
        """
        # Define member variables as local variables so that
        # we do not incur dyanmic errors with changing joint values
        theta1 = self.theta1
        theta2 = self.theta2
        theta3 = self.theta3
        disp3  = self.disp3
        p1 = self.p1=2
        p2 = self.p2=1
        p3 = self.p3=1
        # Define DH variables
        a = np.array([p2, p3, 0])
        alpha = np.array([0.0, 0.0, 0.0])
        d = np.array([p1, 0, disp3])
        # Write DH matrices
        A1 = np.array([[cos(theta1), (-sin(theta1) * cos(alpha[0])), (sin(theta1) * sin(alpha[0])), (a[0] * cos(theta1))], [sin(theta1), (cos(theta1) * cos(alpha[0])), (-cos(theta1) * sin(alpha[0])), (a[0] * sin(theta1))], [0, sin(alpha[0]), cos(alpha[0]), d[0]], [0, 0, 0, 1]])
        A2 = np.array([[cos(theta2), (-sin(theta2) * cos(alpha[1])), (sin(theta2) * sin(alpha[1])), (a[1] * cos(theta2))], [sin(theta2), (cos(theta2) * cos(alpha[1])), (-cos(theta2) * sin(alpha[1])), (a[1] * sin(theta2))], [0, sin(alpha[1]), cos(alpha[1]), d[1]], [0, 0, 0, 1]])
        A3 = np.array([[cos(theta3), (-sin(theta3) * cos(alpha[2])), (sin(theta3) * sin(alpha[2])), (a[2] * cos(theta3))], [sin(theta3), (cos(theta3) * cos(alpha[2])), (-cos(theta3) * sin(alpha[2])), (a[2] * sin(theta3))], [0, sin(alpha[2]), cos(alpha[2]), d[2]], [0, 0, 0, 1]])
        # Get z's and o's for Jacobian
        z0 = np.array([[0], [0], [1]])
        z1 = A1[:3, 2:3]
        z2 = A2[:3, 2:3]
        O0 = np.array([[0], [0], [0]])
        O1 = A1[:3, 3:4]
        O3 = A3[:3, 3:4]
        # Write columns of Jacobian
        E11 = np.cross(z0, (O3-O0), axis=0)
        E12 = np.cross(z1, (O3-O1), axis=0)
        E13 = z2
        E21 = z0
        E22 = z1
        E23 = np.array([[0], [0], [0]])
        # Build up the Jacobian
        Jacobian = np.zeros((6,3), dtype=float)
        Jacobian[:3, :1] = E11
        Jacobian[:3, 1:2] = E12
        Jacobian[:3, 2:3] = E13
        Jacobian[3:6, :1] = E21
        Jacobian[3:6, 1:2] = E22
        Jacobian[3:6, 2:3] = E23
        # Assign Jacobian to member var Jacobian so that it can be used
        self.Jacobian = Jacobian

    def joint_states_callback(self, joint_state_msg):
        # Get joint position values
        self.theta1 = joint_state_msg.position[0]
        self.theta2 = joint_state_msg.position[1]
        self.disp3  = joint_state_msg.position[2]

    def calculate_inverse_velocity_kin(self, request, response):
        # Log to terminal that end effector velocity was received
        endeff = np.array([[0],[request.end_effector_ref_vel],[0],[0],[0],[0]], dtype=float)
        print(f"We received an end effector velocity of {endeff}")

        # ----- Calculate joint velocities ------
        self.calculate_jacobian()
        indiv_joint_velocities = np.dot(np.linalg.pinv(self.Jacobian), endeff)
        response.joint1_velocity = float(indiv_joint_velocities[0])
        response.joint2_velocity = float(indiv_joint_velocities[1])
        response.joint3_velocity = float(indiv_joint_velocities[2])

        print(f"Calculated the reference velocity for each joint as v1:{response.joint1_velocity} v2:{response.joint2_velocity} v3:{response.joint3_velocity}, sending response")

        return response


def main():
    rclpy.init()

    velocity_kin_service = ScaraVelocityKinematics()

    rclpy.spin(velocity_kin_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()