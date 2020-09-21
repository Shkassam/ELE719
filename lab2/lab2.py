from myrobot import Robot
import math
import time

sampling_period = 0.02

run_time = 3.0

w = 8.0

myrobot = Robot(sampling_period)

theta_0 = 30.0*math.pi/180.0
myrobot.initialize(theta_0)

elapsed_time = 0.0
start_time = time.time()

while (elapsed_time < run_time):
    myrobot.get_readings_update()
    print 'angular velocities of wheels = ', myrobot.angular_velocities
    myrobot.set_angular_velocities([w, w, w])
    print 'ir sensors values = ', myrobot.ir_sensors_raw_values
    print 'robot orientation = ', myrobot.orientation*180.0/math.pi
    print ' '
    elapsed_time = time.time() - start_time

myrobot.stop()
myrobot.close()
