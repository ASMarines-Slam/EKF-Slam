from ekf_ros_init import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *


class ExtendedKalmanFilterSLAM:
    def __init__(self, state, covariance,
                robot_width, scanner_displacement,
                control_motion_factor, control_turn_factor,
                measurement_distance_stddev, measurement_angle_stddev,):
                self.state = state
                self.covariance = covariance
                self.scanner_displacement =scanner_displacement
                self.control_motion_factor = control_motion_factor
                self.control_turn_factor = control_turn_factor
                self.measurement_distance_stddev = measurement_distance_stddev
                self.measurement_angle_stddev=measurement_angle_stddev
                self.measurement_IMU_Error=measurement_IMU_Error
                #for initig
                self.number_of_landmarks=0

    def predict(self,control):
        """The prediction step of the Kalman filter."""
    
    @staticmethod
    def g(self):

        
    def dg_dstate(self):

    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))

        #the r will be given to us by the camera directly
        r = sqrt(dx * dx + dy * dy)

        #same
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        #to be discussed later in the meeting
        velocity_in_x = self.state[3]
        velocity_in_y = self.state[4]

        return array([r, alpha,velocity_in_x,velocity_in_y])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):
        theta = state[2]
        cost, sint = cos(theta), sin(theta)
        dx = landmark[0] - (state[0] + scanner_displacement * cost)
        dy = landmark[1] - (state[1] + scanner_displacement * sint)
        q = dx * dx + dy * dy
        sqrtq = sqrt(q)
        drdx = -dx / sqrtq
        drdy = -dy / sqrtq
        drdtheta = (dx * sint - dy * cost) * scanner_displacement / sqrtq

        #Extra to be discussed in the meeting
        drdxdot=
        drdydot=


        dalphadx =  dy / q
        dalphady = -dx / q
        dalphadtheta = -1 - scanner_displacement / q * (dx * cost + dy * sint)

        #Extra to be discussed in the meeting
        dalphadxdot=
        dalphadydot=


        dvelocityx_dx=
        dvelocityx_dy=0
        dvelocityx_dtheta=
        dvelocityx_dvelocityx=1
        dvelocityx_dvelocityy=0

        dvelocityy_dx=0
        dvelocityy_dy=
        dvelocityy_dtheta=
        dvelocityy_dvelocityx=0
        dvelocityy_dvelocityy=1



        return array([[drdx, drdy, drdtheta, drdxdot , drdydot ],
                     [dalphadx, dalphady, dalphadtheta,dalphadxdot,dalphadydot]
                     [dvelocityx_dx,dvelocityx_dy,dvelocityx_dtheta,dvelocityx_dvelocityx,dvelocityx_dvelocityy]   
                     [dvelocityy_dx,dvelocityy_dy,dvelocityy_dtheta,dvelocityy_dvelocityx,dvelocityy_dvelocityy]
                     ])
        #return array([[drdx, drdy, drdtheta],
                      #[dalphadx, dalphady, dalphadtheta]])

    def correct(self, measurement, landmark_index):
        """The correction step of the Kalman filter."""
        # Get (x_m, y_m) of the landmark from the state vector.
        landmark = self.state[5+2*landmark_index : 5+2*landmark_index+2]
        H5 = self.dh_dstate(self.state, landmark, self.scanner_displacement)

        # --->>> Add your code here to set up the full H matrix.
        N = self.number_of_landmarks
        
        #because we have 4 measurements, the r, the theta, velocity from x , velocity from y
        new_H = zeros((4,5+2*N))
        #new_H = zeros((2,5+2*N))

        new_H[0:4,0:5] = H5

        # the added drevatives for that landmark
        new_H[0:4, 5+2*landmark_index : 5+2*landmark_index+2] = new_H[0:4,0:2]*-1

        H = new_H  

        # This is the old code from the EKF - no modification necessary!
        Q = diag([self.measurement_distance_stddev**2,
                  self.measurement_angle_stddev**2,
                  self.measurement_IMU_Error**2,
                  self.measurement_IMU_Error**2])
        K = dot(self.covariance,
                dot(H.T,    linalg.inv(dot(H, dot(self.covariance, H.T)) + Q)  ))
        innovation = array(measurement) -\
                     self.h(self.state, landmark, self.scanner_displacement)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        self.state = self.state + dot(K, innovation)
        self.covariance = dot(eye(size(self.state)) - dot(K, H),
                              self.covariance)

    

    





Camera_Displacment = 50
Sonar_Displacement = 50

#The sensor node will choose wheter the readings comes from the Sonar or the camera
scanner_displacement=Camera_Displacment


#Landmark extraction and constraints
Distance_between_each_landmark = 60
# Filter constants.
control_motion_factor = 0.35  # Error in motor control.
control_turn_factor = 0.6  # Additional error due to slip when turning.
measurement_distance_stddev = 600.0  # Distance measurement error of cylinders.
measurement_angle_stddev = 45. / 180.0 * pi  # Angle measurement error.
measurement_IMU_Error=5

# Arbitrary start position.
# the initial state will be
#       x , y , theta , x_dot and y_dot
x_init = 0
y_init = 0
theta  = 0
x_dot  = 0
y_dot  = 0

initial_state = array([x_init,y_init,theta,x_dot,y_dot])
# Covariance at start position.
initial_covariance = zeros((5,5))

# Setup filter.
kf = ExtendedKalmanFilterSLAM(initial_state, initial_covariance,
                            scanner_displacement,
                            control_motion_factor, control_turn_factor,
                            measurement_distance_stddev,
                            measurement_angle_stddev,measurement_IMU_Error)

while not rospy.is_shutdown():

    #prediction step
    control = get_thrusters_readings()
    kf.predict(control)
    #correction step

    #we have to update the position and the velocities
    #the velocities will be updated from the IMU readings

    IMU_Velocity_in_X , IMU_Velocity_in_Y = get_imu_velocities()
    measurements_from_sensor_node = get_observations()

    #we here need to choose if we will update the velocities only if there are no 
    #measurments from the sensor node, or update them all together







    

