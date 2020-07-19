"""control_test controller."""

from controller import Robot, Motor, PositionSensor, GPS, Display
from webots_api_helper_funs import *
from forw_kinm import *
from inv_kinm import *
#from anim_func import *
import numpy as np
#import matplotlib.pyplot as plt 
#import mpl_toolkits.mplot3d.axes3d as p3
#from matplotlib.animation import FuncAnimation
#import matplotlib.colors as colr
import sys
import scipy.optimize
import random

# angle in radians ofc
def turnAngleToPlusMinusPiRange(angle):
    return np.arcsin(np.sin(angle))
# let's trace out a circle
# the circle formula is:
# x = radius * cos(t)
# y = radius * sin(t)
# z = height
# the direction you are to move in in order to trace a circle,
# or any parametric curve for that matter is
# in the direction of the derivative at the given point
# the derivative w.r.t. t is, by coordinate:
# x = radius * -1 * sin(t)
# y = radius * cos(t)
# z = 0
def goInACirleViaDerivative(radius, height, current_parameter):
    return np.array([radius * -1 * np.sin(curve_parameter), radius * np.cos(curve_parameter), 0])


# or just pass the next point to reach and use the vector to get there lel
def goInACirleViaPositionAroundZ(radius, height, current_parameter):
    return np.array([radius * np.cos(curve_parameter), radius * np.sin(curve_parameter), height])

# here height is distance from yz plane
# but also from xy plane (we want it to be floating around axis parallel to x axis actually)
def goInACirleViaPositionAroundLiftedX(radius, height, x_0, current_parameter):
    return np.array([x_0, radius * np.sin(curve_parameter), height
                + radius * np.cos(curve_parameter)])


def generateCircleForTesting(robot):
    """1. assume robot is fully extended when all motor positions are at 0
        2. assume robot's workspace is a torus
        3. now pick 3 points inside of the workspace

    :robot: TODO
    :returns: TODO

    """
    pass


# for validation purposes, we must plot this circle
def drawCircle(radius, height, robot):
    display = robot.getDisplay("display")
    display.setColor(0xFF0FF)
    display.drawOval(64,64,45,45)


# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

print("eto me")

#enable gps
gps = robot.getGPS("ee_gps")
gps.enable(10)





# Main loop:
# - perform simulation steps until Webots is stopping the controller
#t = np.array([0.07,0.7,0.75929])
#t = np.array([-0.60,0.07,0.75929])
t = np.array([0.29702256, -0.424394371, 0.64633786])
iter_num = 0
motors = getAllMotors(robot)
initializeMotorsForPosition(motors)
sensors = getAndInitAllSensors(robot)
r = Robot_raw(motors, sensors)


# get me a curve yo
curve_parameter = 1
curve_parameter_step = 0.01
radius = 0.35
height = 0.67
x_0 = -0.99
drawCircle(radius, height, robot)

for motor in motors:
    motor.setVelocity(float('inf'))
   
#for motor in motors:
#        motor.setPosition(0.0)

inited = 0

# initialize a file in which measurements are to be stored
# for later analysis and visualization
# manipulability measure is left column
# and the smallest eigenvalue is the right column
measurements_file = open("./data/sing_av_pinv_mem_data", "w")

while robot.step(timestep) != -1:
 #   print("r.p_e:", r.p_e)
#    print("target:", t)
    ee_pos_gps = gps.getValues()
    ee_pos_gps[0] = -1 * ee_pos_gps[0]
    z_cp = ee_pos_gps[1]
    ee_pos_gps[1] = ee_pos_gps[2]
    ee_pos_gps[2] = z_cp
#    print("gps position:", ee_pos_gps)


    current_joint_positions = readJointState(sensors)
    iter_num += 1

    #e = t - np.array(ee_pos_gps)
#    e = t - r.p_e

    # get me circle positions
#    t = goInACirleViaPositionAroundLiftedX(radius, height, x_0, curve_parameter)
#    t = goInACirleViaPositionAroundZ(radius, height, curve_parameter)
    e = t - np.array(ee_pos_gps)
    error = np.sqrt(np.dot(e,e))
    print("error = ", error)
    print("target =", t)
    print("position =", ee_pos_gps)


    # for ik, give a random spot
    if error < 0.02:
        t = np.array([random.uniform(-0.75, 0.75), random.uniform(-0.75, 0.75), random.uniform(0, 0.75)])
        #if np.abs(t[0]) + np.abs(t[1]) + np.abs(t[2]) > 1.3:
         #   t = t - 0.1
        if np.abs(t[0]) + np.abs(t[1]) + np.abs(t[2]) < 0.45:
            t = t + 0.2


# do not give me the next point before i got to the one you gave me
    if inited != 0: 
        curve_parameter += curve_parameter_step
        inited = 0
    else:
        if error < 0.01:
            inited = 1

    # right from the maric paper
    # you prolly want to update this jacobian via forwKinm function (by first
    # reading the sensors and then calculating the jacobian (but it could be unnecessary too)
#    r.calcJacobian()
    #M = r.jacobian @ r.jacobian.T
    M = r.jac_tri @ r.jac_tri.T
    manip_index = np.sqrt(np.linalg.det(M))
    eigenvals, eigvecs = np.linalg.eig(M)

    # now write this to the measurements file
    measurements_file.write(str(manip_index) + ";" + str(eigenvals[eigenvals.argmin()]) + "\n")
    # and stop after you have finished going around the shape
    if curve_parameter > 16.0:
        print("WE DONE")
        sys.exit(0)



    # here you choose which ik method you want
    # just pass the robot_raw instance and the target position
    # they use the calculated position of ee
    # of course that can be modified
#    del_thet = invKinm_Jac_T(r, t)
#    del_thet = invKinm_PseudoInv(r, t)
#    del_thet = invKinm_dampedSquares(r, t)
#    del_thet = invKinmGradDesc(r, t)
#    del_thet = invKinmSingAvoidance_PseudoInv(r, t)
#    del_thet = invKinmSingAvoidanceWithQP(r, t)



#    del_thet = np.array(invKinm_Jac_T(r, t)) / 3
#    del_thet = np.array(invKinm_PseudoInv(r, t)) / 3
#    del_thet = np.array(invKinm_dampedSquares(r, t)) / 3
#    del_thet = np.array(invKinmGradDesc(r, t)) / 3
#    del_thet = np.array(invKinmSingAvoidance_PseudoInv(r, t)) / 3
    del_thet = np.array(invKinmSingAvoidanceWithQP(r, t)) / 3

# clamping for joint rotation limits
#    print("del_thet")
#    print(del_thet)
#    print("current_joint_positions")
#    print(current_joint_positions)
#    r.printJacobianForSomeAngles(current_joint_positions)
  #  setMotorSpeeds(motors, del_thet)
    r.forwardKinmViaPositions(del_thet, motors, sensors)

#    motor_positions = readJointState(sensors)
#    print(motor_positions)

