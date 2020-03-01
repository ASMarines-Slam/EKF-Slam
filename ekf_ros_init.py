import rospy
from std_msgs.msg import String

def get_thrusters_readings():
    #this will be replaced by a ROS subscriber script that will recieve
    #the thruster readings from the control node
    tx=5
    ty=5
    return(tx,ty)


#will read the sensor data from the mission planner node
def get_observations():


def get_imu_velocities():
    #will integrate the IMU readings to get the velocity

    return velocity_in_x,velocity_in_y
    