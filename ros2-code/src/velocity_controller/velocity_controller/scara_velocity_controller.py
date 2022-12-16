# RBE 500 Group Assignment #3
# Group 2 - Joshua Gross, Arjan Gupta, Melissa Kelly

import sys
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rbe500_custom_interfaces.srv import ScaraEndEffVelRef, CalcScaraJointVelRefs
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
import numpy as np
import time
from matplotlib import pyplot as plt

# Node for subscriber, publisher, service, and client
class ScaraVelocityController(Node):

    def __init__(self):
        # Initialize superclass
        super().__init__('scara_velocity_controller')

        # --- Initialize member variables for reference velocities ---
        self.received_ref_vels: bool = False
        self.end_effector_ref_vel = 3.4 # arbitrarily chosen
        self.ref_v1: float = 1
        self.ref_v2: float = 1
        self.graph_ref_v1: float = self.ref_v1
        self.graph_ref_v2: float = -1*self.ref_v2
        self.ref_v3: float = 0.0
        self.ref_q1 = 1.5708
        self.ref_q2 = 0.0
        # --- Initialize member variables for joint_states subscription ---
        self.awating_ref_vel_count: int = 0
        # --- Initialize member variables for run_controller method ---
        # Define gains for v1
        self.Kp1:float = 0.35
        self.Kd1:float = 0.3
        # Define gains for v2
        self.Kp2:float = -0.35
        self.Kd2:float = -0.15
        # Define gains for v3
        self.Kp3:float = 5
        self.Kd3:float = 5
        self.num_values_received: int = 0 
        self.accel_approx_last_time = time.time()
        # Store the last known velocities
        self.last_v1: float = 0.0
        self.last_v2: float = 0.0
        self.last_v3: float = 0.0
        # --- Initialize member variables for clients ---
        self.switch_controller_req = SwitchController.Request()
        self.convert_end_effector_velocity_req: CalcScaraJointVelRefs.Request() = CalcScaraJointVelRefs.Request()
        # --- Initialize member variables for timing/graphing ---
        self.time_array = np.arange(0, 10, .01)
        self.curr_time_iterator = 0
        self.joint1_data_array = np.array([])
        self.joint2_data_array = np.array([])
        self.joint3_data_array = np.array([])
        self.plot_data_last_time = time.time()
        self.graph_generated = False

        # Create the client that will activate the required controller. 
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        # If we find that we had to wait for the service, it means Gazebo is not 
        # running. We exit the program in this case because trying to call the service
        # while Gazebo is starting up causes errors. This Node needs to be started
        # after Gazebo is already running.
        if not self.switch_controller_client.wait_for_service(timeout_sec=0.5):
            print('The /controller_manager/switch_controller service is not available. Please launch Gazebo before running this package, or simply try running this package again.')
            sys.exit()
        # Activate the Gazebo effort controller
        self.activate_effort_controller()

        # Create another client that will get the joint velocity references from our other ROS Node
        self.velocity_reference_client = self.create_client(CalcScaraJointVelRefs, 'velocity_inv_kin_service')
        self.get_target_velocities()

        # Create the subscriber that receives the joint state information, with a queue of 50 since it is very fast
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 100)
        print("Done creating subscription that receives joint-state information.")

        # Create the publisher that will send the joint efforts
        self.efforts_publisher = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 10)
        print("Done creating the publisher for sending joint efforts.")
    
    def joint_states_callback(self, joint_state_msg):
        # Only take action if we have a reference/goal velocity to work against
        if self.received_ref_vel:
            # Record the current velocitys for graphing
            self.dump_graph_data(joint_state_msg)
            # Perform the work for the PD controller
            self.run_controller(joint_state_msg)
        elif self.awating_ref_vel_count >= 100:
            print("Receiving joint state info. Awaiting a reference end effector velocity to be given.")
            self.awating_ref_vel_count = 0
        else:
            self.awating_ref_vel_count += 1
        # Update the time captured for the last set of velocities we received
        self.accel_approx_last_time = time.time()

    def dump_graph_data(self, joint_state_msg):
        # Collect graphing data
        if (self.curr_time_iterator < len(self.time_array)) and (time.time() - self.plot_data_last_time >= 0.01):
            # Show elapsed time difference
            if self.curr_time_iterator % 50 == 0:
                print(f"Collecting data for plotting...")
            # Get joint velocity values
            v1 = joint_state_msg.velocity[0]
            v2 = joint_state_msg.velocity[1]
            v3 = joint_state_msg.velocity[2]
            # Append values to joint data arrays
            self.joint1_data_array = np.append(self.joint1_data_array, v1)
            self.joint2_data_array = np.append(self.joint2_data_array, v2)
            self.joint3_data_array = np.append(self.joint3_data_array, v3)
            # Increment iterator, update the last time data was dumped
            self.curr_time_iterator += 1
            self.plot_data_last_time = time.time()
        
        # Plot graph if we are done sampling
        if (self.curr_time_iterator >= len(self.time_array)) and not self.graph_generated:
            # Create subplot 1
            plt.subplot(2, 1, 1)
            plt.plot(self.time_array, self.joint1_data_array, label="Current velocity")
            plt.plot(self.time_array, np.full((len(self.time_array), 1), self.graph_ref_v1), label="Reference velocity")
            plt.legend()
            plt.title("Joint 1 Velocity vs Time")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Velocity (Radians/s)")

            # Create subplot 2
            plt.subplot(2, 1, 2)
            plt.plot(self.time_array, self.joint2_data_array, label="Current velocity")
            plt.plot(self.time_array, np.full((len(self.time_array), 1), self.graph_ref_v2), label="Reference velocity")
            plt.legend()
            plt.title("Joint 2 Velocity vs Time")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Velocity (Radians/s)")

            # Size the plots better for better visual appearance
            plt.subplots_adjust(bottom=0.05,
                                top=.95,
                                wspace=0.6,
                                hspace=0.6)
            plt.show()

            # Mark that we have generated this graph so that we don't continuously generate it
            self.graph_generated = True

    
    def run_controller(self, joint_state_msg):
        """
        This function runs our velocity PD controller. It grabs the current
        velocities of all three joints of our SCARA robot, applies the
        proportional and derivative gains for the error and derivative error,
        and computes the output efforts that need to be applied to the
        corresponding joints.
        """
        # # Get joint position values
        p1 = joint_state_msg.position[0]
        p2 = joint_state_msg.position[1]
        # Get joint velocity values
        v1 = joint_state_msg.velocity[0]
        v2 = joint_state_msg.velocity[1]
        v3 = joint_state_msg.velocity[2]

        # Conditional printing so that we're not printing continuously
        if self.num_values_received >= 100:
            print("Controller is running!")
            print(f"Current joint velocities are v1:{v1}, v2:{v2}, v3:{v3}")
            self.num_values_received = 0
        else:
            self.num_values_received += 1
        
        # ----- Implement controller ------
        # Find errors
        err_v1 = self.ref_v1 - v1
        err_v2 = self.ref_v2 - v2
        err_v3 = self.ref_v3 - v3
        # Approximate the acceleration by using the derivative 
        # approximation technique shown in Week 8 lecture
        accel1 = (v1 - self.last_v1)/(time.time() - self.accel_approx_last_time)
        accel2 = (v2 - self.last_v2)/(time.time() - self.accel_approx_last_time)
        accel3 = (v3 - self.last_v3)/(time.time() - self.accel_approx_last_time)
        # Update the last velocities
        self.last_v1 = v1
        self.last_v2 = v2
        self.last_v3 = v3
        # Find error dots
        err_dot_v1 = -1 * accel1
        err_dot_v2 = -1 * accel2
        err_dot_v3 = -1 * accel3
        # The modeled controller for each joint 
        # (as described in our report) is
        # F = K_p*E - K_d*v
        output_effort_v1: float = self.Kp1*err_v1 + self.Kd1*err_dot_v1
        output_effort_v2: float = self.Kp2*err_v2 + self.Kd2*err_dot_v2
        output_effort_v3: float = self.Kp3*err_v3 + self.Kd3*err_dot_v3
        # For v3, there is also a force of gravity acting upon it
        output_effort_v3 += -9.8
        # Cut off the efforts if we're in position range
        if abs(1.57 - p1) <= 0.01:
            self.ref_v1 = 0.0
            output_effort_v1 = 0.0
        if abs(0 - p2) <= 0.01:
            self.ref_v2 = 0.0
            output_effort_v2 = 0.0
        # Show efforts
        print(f"effort1 is {output_effort_v1} and effort2 is {output_effort_v2}")
        # Publish the output efforts
        efforts_arr: Float64MultiArray = Float64MultiArray()
        efforts_arr.data = [output_effort_v1, output_effort_v2, 0.0]
        self.efforts_publisher.publish(efforts_arr)
    
    def get_target_velocities(self):
        """
        This function calls our inverse velocity kinematics node to return our reference joint velocities.
        """
        # Send another client request to convert the end effector velocity to joint velocities
        while not self.velocity_reference_client.wait_for_service(timeout_sec=1.0):
            print("The velocity kinematics services are not online, waiting...")
        # Send the request and get the conversion
        self.convert_end_effector_velocity_req.end_effector_ref_vel = self.end_effector_ref_vel
        
        # Start an asynchronous call and then block until it is done
        future: Future = self.velocity_reference_client.call_async(self.convert_end_effector_velocity_req)
        print("About to spin until future complete.")
        rclpy.spin_until_future_complete(node=self, future=future, timeout_sec=5)
        
        # Capture the result
        self.ref_v1 = future.result().joint1_velocity
        self.ref_v2 = future.result().joint2_velocity
        self.ref_v3 = future.result().joint3_velocity

        # Report the result 
        print(f"Obtained reference velocities for joints - v1: {self.ref_v1} v2: {self.ref_v2} v3: {self.ref_v3}")

        # Set flag that we've received the goal velocities
        self.received_ref_vel = True
        print("We're ready to start our controller!")

        # Reset the timer iterator and graphing flag
        self.curr_time_iterator = 0
        self.graph_generated = False
        # Reset joint data arrays
        self.joint1_data_array = np.array([])
        self.joint2_data_array = np.array([])
        self.joint3_data_array = np.array([])
    
    def activate_effort_controller(self):
        # Set the controller we want to activate
        self.switch_controller_req.activate_controllers = ["forward_effort_controller"]
        # Set the controllers we want to deactivate
        self.switch_controller_req.deactivate_controllers = ["forward_position_controller", "forward_velocity_controller"]
        # Start an asynchronous call and then block until it is done
        future = self.switch_controller_client.call_async(self.switch_controller_req)
        rclpy.spin_until_future_complete(self, future)
        # Report the result 
        print(f"The result of the attempt to activate the effort controller is ok: {future.result().ok}")

def main():
    rclpy.init()

    scara_velocity_controller = ScaraVelocityController()

    rclpy.spin(scara_velocity_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()