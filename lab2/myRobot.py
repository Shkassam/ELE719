from numpy import array, dot
from math import sin, cos, exp, sqrt, pi

from Robot3WD import Robot

class myRobot(Robot):
    def __init__(self, sampling_period):
        Robot.__init__(self, sampling_period)

    # --------------------------------------------------------------------------------------#
    # Pre-Lab work for Experiment 2                                                         #
    # --------------------------------------------------------------------------------------#
    def inverse_kinematics(self, p_dot, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        m_theta = array([(sin(theta),(-1)*cos(theta),(-1)*L),\
        (cos(pi/6 + theta), sin(pi/6 + theta), (-1)*L),\
        ((-1)*cos(pi/6 - theta), sin(pi/6 - theta),(-1)*L)])
        wheel_angular_velocities = (dot(m_theta,p_dot))/wheel_radius
        return wheel_angular_velocities

    def move_left(self, vx, theta):
        p_dot = array([-vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def move_forward(self, vy, theta):
        p_dot = array([0.0, vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def move_backward(self, vy, theta):
        p_dot = array([0.0, -vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)  
        
    def move_right(self, vx, theta):
        p_dot = array([vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def rotate_CCW(self, w, theta):
        p_dot = array([0.0, 0.0, -w]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def rotate_CW(self, w, theta):
        p_dot = array([0.0, 0.0, w]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
