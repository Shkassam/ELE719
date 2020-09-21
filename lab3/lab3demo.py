import os
from numpy import array
from numpy.linalg import norm
from math import sqrt, pi
from time import time

from myRobot import myRobot
from Robot3WD.PID import PIDControllers

# Set up sampling period and stopping time
T_s =
T_f =

# Set up initial posture
x_0 =
y_0 =
theta_0 =

# Set up final posture
x_f =
y_f = 
theta_f = 

# set up p_0, p_f, delta_p
p_0 = array([x_0, y_0, theta_0]).T
p_f = 
delta_p = 

# Set up stopping criterion
epsilon = sqrt( 2.0*(0.5/100.0)**2 + (2.0*pi/180.0)**2 )

# Initialize PID controllers for the wheels
Kp = array([1.0, 1.0, 1.0]).T
Ki = 
Kd = 
pid = PIDControllers(Kp, Ki, Kd, T_s)

# Initialize robot and other variables

robot = myRobot(T_s)
robot.initialize(theta_0)

goal_reached = False

current_time = 0.0

# Set current posture p to p_0
p = p_0

# Calculate error norm
error_norm = norm(p_f - p)

# open a file to save data for plotting with MATLAB later
file = open('p.csv', 'w')
os.chmod('p.csv', 0666)

file.write('%10.3e %10.3e %10.3e %10.3e %10.3e\n' % (current_time, p[0], p[1], p[2], error_norm) ) 

# Set start time
start_time = time()

# Control Loop
while ( (not goal_reached) and (current_time < T_f) ): 
   robot.get_readings_update()

   # get current robot orientation
   theta = 

   # Determine p_dot using forward kinematics
   p_dot = 

   # Determine value of reference path p_r at current time
   p_r =

   # Determine pd_dot (desired p_dot) using PID controllers
   pd_dot = 

   # Determine desired wheel velocities using inverse kinematics
   w = 

   # Execute the motion
   robot.set_angular_velocities(...)

   # odometry update
   p =

   # check to see if goal is reached
   error_norm = norm(p_f - p)
   goal_reached = (error_norm <= epsilon)

   # time update
   current_time = time() - start_time

   # Write data to file
   file.write('%10.3e %10.3e %10.3e %10.3e %10.3e\n' % (current_time, p[0], p[1], p[2], error_norm) ) 

# Either goal is reached or current_time > T_f
if goal_reached:
   print 'Goal is reached, error norm is', error_norm
else:
   print 'Failed to reach goal, error norm is', error_norm

robot.stop()
robot.close()

# written data to file 'p.csv'
file.close()
