# RBE 500 F01 Group Project
# November 15, 2022

import rclpy
import numpy as np
import random

from math import cos, sin, atan2, sqrt, acos
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

class Subscriber(Node):

    def __init__(self):

        super().__init__('Subscriber')
        # Declare the link lengths so that they may be accessed for the inverse and forward kinematics
        self.p1=2
        self.p2=1
        self.p3=1

        # Create subscriebr to listen for the joint values array to be published to the JointValues topic
        self.listening = self.create_subscription(Float32MultiArray, 'JointInputs', self.calculate_forward_kinematics, 10)  

        # Create subscriber to listen for the end effector pose to be published to the EndEffector topic
        #self.listening = self.create_subscription(Pose, 'EndEffectorLoc', self.calculate_inverse_kinematics, 10)

    def calculate_forward_kinematics(self, mesgfrompub):

        joint_positions = mesgfrompub.data

        theta1 = joint_positions[0]
        theta2 = joint_positions[1]
        disp3 = joint_positions[2]

        joints = [theta1, theta2, disp3]

        link_lengths = [self.p1, self.p2, self.p3]



        T30 = np.array([[((cos(theta1)*cos(theta2))-(sin(theta1)*sin(theta2))), ((-cos(theta1)*sin(theta2))-(cos(theta2)*sin(theta1))), 0, ((self.p2*cos(theta1)) + (self.p3*cos(theta1)*cos(theta2)) - (self.p3*sin(theta1)*sin(theta2)))],
        [((cos(theta1)*sin(theta2)) + (cos(theta2)*sin(theta1))), ((cos(theta1)*cos(theta2)) - (sin(theta1)*sin(theta1))), 0, ((self.p2*sin(theta1)) + (self.p3*cos(theta1)*sin(theta2)) + (self.p3*cos(theta2)*sin(theta1)))],
        [0, 0, 1, (self.p1 + disp3)],
        [0, 0, 0, 1]])


        print('With joint positions of ' + str(joints) +' and link lengths of ' + str(link_lengths) + 
            ' the end effector pose is: \n' + str(T30))


def main(args=None):

    rclpy.init(args=args)
    GroupProject_subscriber = Subscriber()
    rclpy.spin(GroupProject_subscriber)

if __name__ == '__main__':

    main()
