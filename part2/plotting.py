# Some pseudo code to assist with the graphing elements of the assignment

from matplotlib import pyplot as plt
import numpy as np
import random
import time

Joint1_ref = 5
Joint2_ref = 2
Joint3_ref = 1
Joint1 = np.array([])
Joint2 = np.array([])
Joint3 = np.array([])

Time = np.arange(0, 10, .01)
starttime=time.time()
end_time = starttime + 10

for t in Time:
	Joint1ValFromGazebo = random.randrange(0,10) #This will actually be for inserting the real joint value from Gazebo
	Joint2ValFromGazebo = random.randrange(0,10) #This will actually be for inserting the real joint value from Gazebogit
	Joint3ValFromGazebo = random.randrange(0,10) #This will actually be for inserting the real joint value from Gazebo
	Joint1 = np.append(Joint1, Joint1ValFromGazebo) #Append Joint1 array with Gazebo joint value
	Joint2 = np.append(Joint2, Joint2ValFromGazebo) #Append Joint1 array with Gazebo joint value
	Joint3 = np.append(Joint3, Joint3ValFromGazebo) #Append Joint1 array with Gazebo joint value
	# time.sleep(0.01)


Joint1 = np.sort(Joint1) #This just puts the random values in ascending order for sake of this example. Not needed when using real Gazebo values
Joint2 = np.sort(Joint2) #This just puts the random values in ascending order for sake of this example. Not needed when using real Gazebo values
Joint3 = np.sort(Joint3) #This just puts the random values in ascending order for sake of this example. Not needed when using real Gazebo values

# Time1 = np.append(Time1, [10])
# Joint1 = np.append(Joint1, Joint1[len(Joint1)-1])
# Time2 = np.append(Time2, [10])
# Joint2 = np.append(Joint2, Joint2[len(Joint2)-1])
# Time3 = np.append(Time3, [10])
# Joint3 = np.append(Joint3, Joint3[len(Joint3)-1])


plt.subplot(3, 1, 1)
plt.plot(Time, Joint1)
plt.title("Joint 1 Position vs Time")
plt.xlabel("Time (seconds)")
plt.ylabel("Position (Radians)")
plt.subplot(3, 1, 2)
plt.plot(Time, Joint2)
plt.title("Joint 2 Position vs Time")
plt.xlabel("Time (seconds)")
plt.ylabel("Position (Radians)")
plt.subplot(3, 1, 3)
plt.plot(Time, Joint3)
plt.title("Joint 3 Position vs Time")
plt.xlabel("Time (seconds)")
plt.ylabel("Position (meters)")
plt.subplots_adjust(bottom=0.05,
                    top=.95,
                    wspace=0.6,
                    hspace=0.6)
plt.show()
