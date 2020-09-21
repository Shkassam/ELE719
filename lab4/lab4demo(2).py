import os
from numpy import array, dot, zeros
from numpy.linalg import norm
from math import sqrt, pi, sin, cos, atan2
from time import time

from myRobot import *

# Set up sampling period T_s and stopping time T_f
T_s = 0.02      # Sampling Period is 0.02 seconds
T_f = 8.0       # Run Time is 8 seconds

# Set up initial posture
x_0 = 0.0
y_0 = 0.0
theta_0 = 0.0

# Set up goal position
x_f = 0.7
y_f = 1.0
theta_f = 0.0

# Set up p_0 and p_f
p_0 = array([x_0, y_0, theta_0]).T
p_f = array([x_f, y_f, theta_f]).T

# Initialize d_min and d_free (See Eq. 4.4 and 4.5)
d_min = 0.08
d_free = 1.25*d_min

# Set up error tolerance to be within 1cm of target in both x and y directions
epsilon = sqrt( 2.0*(1.0/100.0)**2 )

# Set up controller gains (required in Eq. 4.11 and 4.12)
k_rho = 1.0
k_beta = -1.0
k_alpha = (2.0/pi)*k_rho - (5.0/3.0)*k_beta + 0.5

# Initialize vector pr_dot to be used for inverse kinematics
pr_dot = array([0.0, 0.0, 0.0]).T

# Initialize vector d. The sensor measurement d_i will be assigned to d[i]
d = array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]).T

# Initialize a 3x6 matrix u_iR. The vector delta_i^R will be assigned to
# the i-th column of this matrix
u_iR = zeros(shape=(3,6))

# Initialize a 6x3 matrix sensor_loc to store the sensor frame
# locations as per Table 4.1. The i-th row of this matrix corresponds
# to the i-th ROW of Table 4.1.
R = 0.13
halfR = R/2.0
root3Rover2 = sqrt(3)*R/2.0
sensor_loc = array([
   [-halfR, -root3Rover2, -(2.0/3.0)*pi],
   [-root3Rover2, halfR, (5.0/6.0)*pi],
   [0, R, (1.0/2.0)*pi],
   [halfR, root3Rover2, (1.0/3.0)*pi],
   [root3Rover2, halfR, (1.0/6.0)*pi],
   [root3Rover2, -halfR, -(1.0/6.0)*pi]
])

# open a file to store data for plotting later
f = open('p.csv', 'w')
os.chmod('p.csv', 0666)

# Initial robot
robot = myRobot(T_s)

robot.initialize(theta_0)

# Compute distance to goal, rho, using Eq. 4.1
dp = p_f - p_0
rho = norm(dp[0:2])  # Note: In Python, the range 0:2 means [0,1] NOT [0,1,2]

# Initialize additional variables
p = p_0
goal_reached = False
elapsed_time = 0.0

# Set start time
start_time = time()

# Control loop
while ( (not goal_reached) and (elapsed_time < T_f) ):

    robot.get_readings_update()

    # save data to file for plotting
    f.write('%10.3e %10.3e % 10.3e %10.3e %10.3e\n' % (elapsed_time, p[0], p[1], p[2], rho))

    theta = p[2]

    # Use forward kinematics to get p_dot
    p_dot = robot.forward_kinematics(robot.angular_velocities, theta)

    # Determine angle alpha (remember to used atan2 instead of atan)
    alpha = atan2(dp[1], dp[0]) - theta

    # Get sensor measurements w.r.t. sensor frames
    for i in range(0, 6):
        d[i] = Vraw_to_distance(robot.ir_sensors_raw_values[i])

    # Check if there is a risk of collision with obstacle(s).
    # If true, determine the temporary goal vectors delta_goal^0 and delta_goal^R
    # and use them to find the temporary alpha and rho.
    # See Eqs. 4.4 to 4.11.
    if min(d) <= d_min:
        x_bar = 0.0
        y_bar = 0.0
        u_iR[0, :] = d
        u_iR[2, :] = 1 # The delta_i^Si matrix (creating the new path avoiding the obstacle)
        for i in range(0, 6):
            if d[i] >= d_free:
                u_iR[:, i] = dot(HMatrix((sensor_loc[i, :]).T), u_iR[:, i])
                x_bar = x_bar + u_iR[0, i]
                y_bar = y_bar + u_iR[1, i]

        delta_goalR = array([x_bar, y_bar, 1]).T    # delta_goal^R = dot(H_Si^R, delta_i^Si)
        alpha = atan2(delta_goalR[i, 1], delta_goalR[i, 0]) - theta

        delta_goal0 = dot(HMatrix(p), delta_goalR)  # delta_goal^0 = dot(H_R^0, delta_goal^R)
        dp = delta_goalR - p_0
        rho = norm(dp[0:2])

    # Determine the angle beta
    beta = -(alpha + theta)

    # Determine linear and angular velocities of the robot (v and w) using the control law
    # given in Eqs. 4.12 and 4.13
    v = k_rho*rho
    w = k_alpha*alpha + k_beta*beta

    pr_dot[0] = v*cos(theta)
    pr_dot[1] = v*sin(theta)
    pr_dot[2] = w

    # Now use Inverse Kinematics to determine wheel ref velocities
    wheel_ref_vel = robot.inverse_kinematics(pr_dot, theta)

    # Execute motion
    robot.set_angular_velocities(wheel_ref_vel)

    # Odometry update
    p = p + (p_dot*T_s)

    # Replace calculated update for theta with measured value from IMU
    p[2]= robot.orientation

    # Check to see if goal is reached
    dp = p_f - p
    rho = norm(dp[0:2])
    goal_reached = ( rho <= epsilon)

    # time update
    elapsed_time = time() - start_time


print 'ELPASED TIME = %s rho = %s' % (elapsed_time, rho)

# Either goal is reached or current_time > T_f
if goal_reached:
   print 'Goal is reached, error norm is', rho
else:
   print 'Failed to reach goal, error norm is', rho

robot.stop()
robot.close()

f.close()
