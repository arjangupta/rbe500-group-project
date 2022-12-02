# RBE 500 Group Assignment #2
# Group 2 - Joshua Gross, Arjan Gupta, Melissa Kelly

import sys
import rclpy
from rclpy.node import Node
from rbe500_custom_interfaces.srv import ScaraRefPos
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
import numpy as np
import time
from matplotlib import pyplot as plt

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
        # Initialize member variables for run_controller method
        # Define gains for q1
        self.Kp1 = 10
        self.Kd1 = 10
        # Define gains for q2
        self.Kp2 = 10
        self.Kd2 = 10
        # Define gains for q3
        self.Kp3 = 10
        self.Kd3 = 10
        self.num_values_received: int = 0 
        # Initialize member variables for client
        self.req: SwitchController.Request = SwitchController.Request()
        # Initialize member variables for timing/graphing
        self.time_array = np.arange(0, 10, .01)
        self.curr_time_iterator = 0
        self.joint1_data_array = np.array([])
        self.joint2_data_array = np.array([])
        self.joint3_data_array = np.array([])
        self.last_time = time.time()
        self.graph_generated = False

        # Create the client that will activate the required controller. 
        self.client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        # If we find that we had to wait for the service, it means Gazebo is not 
        # running. We exit the program in this case because trying to call the service
        # while Gazebo is starting up causes errors. This Node needs to be started
        # after Gazebo is already running.
        if not self.client.wait_for_service(timeout_sec=2):
            print('The /controller_manager/switch_controller service is not available. Please launch Gazebo before running this package. Exiting.')
            sys.exit()
        # Activate the Gazebo effort controller
        self.activate_effort_controller()

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
            # Record the current positions for graphing
            self.dump_graph_data(joint_state_msg)
            # Perform the work for the PD controller
            self.run_controller(joint_state_msg)
        elif self.awaiting_ref_pos_count >= 100:
            print("Receiving joint state info. Awaiting a reference position to be given.")
            self.awaiting_ref_pos_count = 0
        else:
            self.awaiting_ref_pos_count += 1

    def dump_graph_data(self, joint_state_msg):
        if (self.curr_time_iterator < len(self.time_array)) and (time.time() - self.last_time >= 0.01):
            # Show elapsed time difference
            print(f"The elapsed time is {time.time() - self.last_time}")
            # Get joint position values
            q1 = joint_state_msg.position[0]
            q2 = joint_state_msg.position[1]
            q3 = joint_state_msg.position[2]
            # Append values to joint data arrays
            self.joint1_data_array = np.append(self.joint1_data_array, q1)
            self.joint2_data_array = np.append(self.joint2_data_array, q2)
            self.joint3_data_array = np.append(self.joint3_data_array, q3)
            # Increment iterator, update the last time data was dumped
            self.curr_time_iterator += 1
            self.last_time = time.time()
        
        # Plot graph if we are done sampling
        if (self.curr_time_iterator >= len(self.time_array)) and not self.graph_generated:
            plt.subplot(3, 1, 1)
            plt.plot(self.time_array, self.joint1_data_array)
            plt.plot(self.time_array, np.full((len(self.time_array), 1), self.ref_q1))
            plt.title("Joint 1 Position vs Time")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Position (Radians)")
            plt.subplot(3, 1, 2)
            plt.plot(self.time_array, self.joint2_data_array)
            plt.plot(self.time_array, np.full((len(self.time_array), 1), self.ref_q2))
            plt.title("Joint 2 Position vs Time")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Position (Radians)")
            plt.subplot(3, 1, 3)
            plt.plot(self.time_array, self.joint3_data_array)
            plt.plot(self.time_array, np.full((len(self.time_array), 1), self.ref_q3))
            plt.title("Joint 3 Position vs Time")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Position (meters)")
            plt.subplots_adjust(bottom=0.05,
                                top=.95,
                                wspace=0.6,
                                hspace=0.6)
            plt.show()
            self.graph_generated = True

    
    def run_controller(self, joint_state_msg):
        """
        This function runs our PD controller. It grabs the current positions
        and velocities of all three joints of our SCARA robot, applies the
        proportional and derivative gains for the error and derivative error,
        and computes the output efforts that need to be applied to the
        corresponding joints.
        """
        # Get joint position values
        q1 = joint_state_msg.position[0]
        q2 = joint_state_msg.position[1]
        q3 = joint_state_msg.position[2]
        # Get joint velocity values
        v1 = joint_state_msg.velocity[0]
        v2 = joint_state_msg.velocity[1]
        v3 = joint_state_msg.velocity[2]

        # Conditional printing so that we're not printing continuously
        if self.num_values_received >= 250:
            print("Controller is running!")
            print(f"Current q values are q1:{q1}, q2:{q2}, q3:{q3}")
            print(f"Current joint velocities are v1:{v1}, v2:{v2}, v3:{v3}")
            self.num_values_received = 0
        else:
            self.num_values_received += 1
        
        # ----- Implement controller ------
        # Find errors
        err_q1 = self.ref_q1 - q1
        err_q2 = self.ref_q2 - q2
        err_q3 = self.ref_q3 - q3
        # Find error dots
        err_dot_q1 = -1 * v1
        err_dot_q2 = -1 * v2
        err_dot_q3 = -1 * v3
        # The modeled controller for each joint 
        # (as described in our report) is
        # F = K_p*E - K_d*v
        output_effort_q1: float = self.Kp1*err_q1 + self.Kd1*err_dot_q1
        output_effort_q2: float = self.Kp2*err_q2 + self.Kd2*err_dot_q2
        output_effort_q3: float = self.Kp3*err_q3 + self.Kd3*err_dot_q3
        # For q3, there is also a force of gravity acting upon it
        output_effort_q3 += -9.8
        # Publish the output efforts
        efforts_arr: Float64MultiArray = Float64MultiArray()
        efforts_arr.data = [output_effort_q1, output_effort_q2, output_effort_q3]
        self.publisher.publish(efforts_arr)
    
    def set_ref(self, request, response):
        # Assign ref values
        self.ref_q1 = request.q1
        self.ref_q2 = request.q2
        self.ref_q3 = request.q3

        # Log to terminal that reference/goal position was received
        print(f"We received reference positions of x:{self.ref_q1} y:{self.ref_q2} z:{self.ref_q3}")

        # Set flag that we've received the goal position
        self.received_ref_pos = True
        print("We're ready to start our controller!")

        # Reset the timer iterator and graphing flag
        self.curr_time_iterator = 0
        self.graph_generated = False
        # Reset joint data arrays
        self.joint1_data_array = np.array([])
        self.joint2_data_array = np.array([])
        self.joint3_data_array = np.array([])

        # Return the acknowledgement
        response.ok = True
        return response
    
    def activate_effort_controller(self):
        # Set the controller we want to activate
        self.req.activate_controllers = ["forward_effort_controller"]
        # Set the controllers we want to deactivate
        self.req.deactivate_controllers = ["forward_position_controller", "forward_velocity_controller"]
        # Start an asynchronous call and then block until it is done
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        # Report the result 
        print(f"The result of the attempt to activate the effort controller is ok: {future.result().ok}")

def main():
    rclpy.init()

    scara_pd_controller = ScaraPDController()

    rclpy.spin(scara_pd_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()