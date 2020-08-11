"""control_test controller."""

from forw_kinm import *
from inv_kinm import *
from follow_curve import *
#from anim_func import *
import numpy as np
#import matplotlib.pyplot as plt 
#import mpl_toolkits.mplot3d.axes3d as p3
#from matplotlib.animation import FuncAnimation
#import matplotlib.colors as colr
import sys
import subprocess
import scipy.optimize
import random

try:
    sim = sys.argv[1]
except IndexError:
    sim = "sim"
    pass

if  sim == "sim":
    from controller import Robot, Motor, PositionSensor, GPS, Display
    from webots_api_helper_funs import *

# angle in radians ofc
def turnAngleToPlusMinusPiRange(angle):
    return np.arcsin(np.sin(angle))
# let's trace out a circle
# th circle formula is:
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



def error_test(robot, t):
    e = abs(t - r.p_e)
    if e[0] < 0.001 and e[1] < 0.001 and e[2] < 0.001:
        return True
    else:
        return False


print("eto me")


# Main loop:
# - perform simulation steps until Webots is stopping the controller
#t = np.array([0.07,0.7,0.75929])
#t = np.array([-0.60,0.07,0.75929])
#t = np.array([-0.59702256, -0.424394371, 0.64633786])
#t = np.array([0.59702256, 0.424394371, 0.64633786])
#t = np.array([-0.1617, -0.1901, 1.0250])
#t = np.array([1.017, 1.1901, 1.0250])
#t = np.array([-0.1617, -0.1901, 1.0250])
#t = np.array([1.017, 1.1901, 1.0250])
t = np.array([-0.165797, 0.571876, 0.618752])
iter_num = 0

if sim == "no_sim":
    r = Robot_raw(robot_name="no_sim")
    damping = 5
else:
    damping = 1
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    print("hey operating on:", robot.getName())
    robot_name =robot.getName()
    gps = robot.getGPS("ee_gps")
    gps.enable(10)
    if robot_name == "UR10e":
        motors = getAllMotors(robot)
        sensors = getAndInitAllSensors(robot)
    if robot_name == "base_link":
        motors = getAllMotorsKuka(robot)
        sensors = getAndInitAllSensorsKuka(robot)
    if robot_name == "j2n6s300":
        motors = getAllMotorsJaco(robot)
        sensors = getAndInitAllSensorsJaco(robot)
    r = Robot_raw(motors=motors, sensors=sensors, robot_name=robot_name)
    initializeMotorsForPosition(motors)
    radius = 0.35
    height = 0.67
#    drawCircle(radius, height, robot)
    for motor in motors:
        motor.setVelocity(float('inf'))




# get me a curve yo
curve_parameter = 1
curve_parameter_step = 0.1
#curve_parameter_step = 0.01
radius = 0.4
height = 0.50
x_0 = -0.59
   
#iter_max = 1000

# initialize a file in which measurements are to be stored
# for later analysis and visualization
# manipulability measure is left column
# and the smallest eigenvalue is the right column
#measurements_file = open("./data/sing_av_pinv_mem_data", "w")
measurements_file_no_sing_avoid = open("./data/no_sing_avoid_200_inv_kinms", "w")
measurements_file_E_kI = open("./data/E_kI_200_inv_kinms", "w")
measurements_file_E_kM = open("./data/E_kM_200_inv_kinms", "w")
measurements_file_manip_max = open("./data/ManipMax_200_inv_kinms", "w")
iter_num = 0

if sim == "no_sim":
    max_tries = 300
    total_number_of_points = 150
else:
    max_tries = 250
    total_number_of_points = 10
    

for broj in range(5):
    number_of_points = 0

    if broj == 0:
        measurements_file = open("./data/no_sing_avoid_200_inv_kinms", "w")

    if broj == 1:
        measurements_file.close()
        measurements_file = open("./data/E_kI_200_inv_kinms", "w")

    if broj == 2:
        measurements_file.close()
        measurements_file = open("./data/E_kM_200_inv_kinms", "w")

    if broj == 3:
        measurements_file.close()
        measurements_file = open("./data/ManipMax_200_inv_kinms", "w")

    if broj == 4:
        measurements_file.close()
        print("we are finished!")
        print("the generated datapoints are in the data folder and the relevant file are:")
        print("1. no_sing_avoid_200_inv_kinms")
        print("2. E_kI_200_inv_kinms")
        print("3. E_kM_200_inv_kinms")
        print("")
        print("Each file is a csv, with \";\" as a separator.")
        print("The entries are: manipulability_measure, smallest eigenval of M, largest eigenval of M")
        print("check out the graphs for results")
        proc = subprocess.run(["python3", "data/turn_data_into_info.py"])

        sys.exit(0)
# 200 for the maximum number of point to be reached by the ik algorithms
    n_of_tries_for_point = 0
    while number_of_points < total_number_of_points:
        if sim == "sim":
            robot.step(timestep)
#        sensors_data = readJointState(r.sensors)
#        print("sensors_data")
#        print(sensors_data)
#        exit()

        iter_num += 1

        if sim == "no_sim":
            n_of_tries_for_point += 1
            e = t - r.p_e
        else:
            ee_pos_gps = gps.getValues()
            ee_pos_gps[0] = -1 * ee_pos_gps[0]
            z_cp = ee_pos_gps[1]
            ee_pos_gps[1] = ee_pos_gps[2]
            ee_pos_gps[2] = z_cp
            current_joint_positions = readJointState(sensors)
            e = t - np.array(ee_pos_gps)
         #   print("ee_pos_gps")
         #   print(ee_pos_gps)
         #   print("r.p_e")
         #   print(r.p_e)

        error = np.sqrt(np.dot(e,e))
        #print("error:", error)
        #print("r.p_e:", r.p_e)
        #print("t:", t)




        # for ik, give a random spot
        if error_test(r, t) or n_of_tries_for_point > max_tries:
            # in no_sim we do tests, in sim we do trajectory following
            if sim == "no_sim":
                # error_vec is used if you want trajectory following via jacobian
                error_vec = None
                if(n_of_tries_for_point > max_tries):
                    print("FAILED TO CONVERGE in", n_of_tries_for_point, "steps!!!")
                    print("i got to", r.p_e, "and the error is:", error)
                else:
                    print("i did it in,", n_of_tries_for_point, "steps")


            if sim == "sim":
                if curve_parameter > 8.0:
                    broj +=1
                    curve_parameter = 1.0
                    break
                curve_parameter += curve_parameter_step
                t = goInACirleViaPositionAroundLiftedX(radius, height, x_0, curve_parameter)
                # error_vec is used if you want trajectory following via jacobian
                # TODO FIX!! (rn does not move at all)
                #error_vec = goInACircleViaJacobian(curve_parameter)
                print(curve_parameter)

# alternative with eigenvalues
#            eigenvals, eigvecs = np.linalg.eig(M)
#            measurements_file.write(str(manip_index) + ";" + str(eigenvals[eigenvals.argmin()]) + ";" + str(eigenvals[eigenvals.argmax()]) + "\n")
            if sim == "no_sim":
                M = r.jac_tri @ r.jac_tri.T
                manip_index = np.sqrt(np.linalg.det(M))
                diagonal_of_svd_of_M = np.linalg.svd(M)[1]
                measurements_file.write(str(manip_index) + ";" + str(diagonal_of_svd_of_M[diagonal_of_svd_of_M.argmin()]) \
                        + ";" + str(diagonal_of_svd_of_M[diagonal_of_svd_of_M.argmax()]) + "\n")

            #t = np.array([random.uniform(-0.75, 0.75), random.uniform(-0.75, 0.75), random.uniform(-0.75, 0.75)])
                t = np.array([random.uniform(-0.70, 0.70), random.uniform(-0.70, 0.70), random.uniform(-0.70, 0.70)])
                number_of_points += 1
    #            if np.abs(t[0]) + np.abs(t[1]) + np.abs(t[2]) < 0.45:
    #                t = t + 0.3
                print("point number:", number_of_points)
                print("target =", t)
                n_of_tries_for_point = 0


            # for trajectory following
            # and TODO stop after you have finished going around the shape



        # here you choose which ik method you want
        # just pass the robot_raw instance and the target position
       # they use the calculated position of ee
        # of course that can be modified

        

        if broj == 0:
            #del_thet = invKinm_Jac_T(r, t)
            del_thet = invKinmQP(r, t) / damping
        if broj == 1:
            del_thet = invKinmQPSingAvoidE_kI(r, t, error_vec) / damping
        if broj == 2:
            del_thet = invKinmQPSingAvoidE_kM(r, t, error_vec) / damping
        if broj == 3:
            del_thet = invKinmQPSingAvoidManipMax(r, t, error_vec) / damping



        # move by calculated amount
        r.forwardKinmViaPositions(del_thet)



        ############ some debugging ############
#        q0 = np.array([1.1778, -1.5286,  2.0600,  2.9207, -1.0143, -0.2353])
        if 1 == 0:
            print("r.ndof")
            print(r.ndof)
            q0 = np.array([np.pi / 2]*6)
#            q0 = np.array([0]*r.ndof)
            r.forwardKinmNumericsOnlyDebug2(q0)
            print("r.p_e")
            print(r.p_e)
            for motor in range(r.ndof):
                r.motors[motor].setPosition(q0[motor])


            ee_pos_gps = gps.getValues()
            ee_pos_gps[0] = -1 * ee_pos_gps[0]
            z_cp = ee_pos_gps[1]
            ee_pos_gps[1] = ee_pos_gps[2]
            ee_pos_gps[2] = z_cp
            current_joint_positions = readJointState(sensors)
            e = t - np.array(ee_pos_gps)
            print("ee_pos_gps")
            print(ee_pos_gps)
    #        print(r.calcMToEGradient_kM())
    ##        for joint in r.joints:
    ##            print(joint.theta)
    ##        r.calcJacobian()
    #        print("the resulting coefficients!!!")
    #        print(invKinmQPSingAvoidManipMax(r, t))
    #        invKinmQPSingAvoidManipMax(r, t)
    #        print("")
    #        print("")
    #        print("")
    ##        print("ik solution")
    ##        print(invKinmQPSingAvoidE_kI(r, t))
            test = True 
            current_joint_positions = np.array([0,0,0, 0,0,0])
            while test:
                robot.step(timestep)
                r.forwardKinmNumericsOnlyDebug2(q0)
                print("r.p_e")
                print(r.p_e)
                ee_pos_gps = gps.getValues()
                ee_pos_gps[0] = -1 * ee_pos_gps[0]
                z_cp = ee_pos_gps[1]
                ee_pos_gps[1] = ee_pos_gps[2]
                ee_pos_gps[2] = z_cp
                print("ee_pos_gps")
                print(ee_pos_gps)
                last_it = current_joint_positions
                current_joint_positions = np.array(readJointState(sensors))
                raz = last_it - current_joint_positions
                summ = 0.0
                for bbb in range(r.ndof):
                    summ += raz[bbb]
                if summ == 0.0:
                    test = False
                print("sensored positions")
                print(current_joint_positions)
            for jointt in r.joints:
                print(jointt.theta)
            exit()
    #

