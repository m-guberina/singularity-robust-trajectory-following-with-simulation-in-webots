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


def initForVideo(robot, r, gps):
    timestep = int(robot.getBasicTimeStep())
    test = True 
#    q0 = np.array([-0.62721, -1.72245, 1.67299, 4.33949, -1.8109014, 1.4324919])
    #q0 = np.array([-0.617609, -1.23382, 1.405701, 1.3009651, -1.86564, -0.00337386])
#    q0 = np.array([-0.210952, 0.4707195, -0.65795456, -1.321194, 2.504538, 1.887749, -0.06052169])
#    q0 = np.array([3.375480, 4.43982, -1.2783410, 3.8671280, 1.9059, 1.82677, -0.03385])
    q0 = np.array([3.5209543, 4.350698, -1.012536, 3.6862029, 2.007738, 1.934826, -0.0306429])
    current_joint_positions = np.array([1.0] * r.ndof)
    for motor in range(r.ndof):
        r.motors[motor].setPosition(q0[motor])
    while test:
        #print("i tried init")
        robot.step(timestep)
        ee_pos_gps = gps.getValues()
        ee_pos_gps[0] = -1 * ee_pos_gps[0]
        z_cp = ee_pos_gps[1]
        ee_pos_gps[1] = ee_pos_gps[2]
        ee_pos_gps[2] = z_cp
        last_it = current_joint_positions
        current_joint_positions = np.array(readJointState(sensors))
        raz = last_it - current_joint_positions
        summ = 0.0
        for bbb in range(r.ndof):
            summ += raz[bbb]
        if summ == 0.0:
            test = False
            print("inited to position for video")
            r.forwardKinmNumericsOnlyDebug(q0)



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
#t = np.array([0.060,0.47,0.75929])
#t = np.array([-0.59702256, -0.424394371, 0.64633786])
#t = np.array([0.64702256, 0.64394371, 0.94633786])
#t = np.array([-0.1617, -0.1901, 1.0250])
#t = np.array([1.017, 1.1901, 1.0250])
#t = np.array([-0.1617, -0.1901, 1.0250])
#t = np.array([0.0, 0.1, 1.10])
t = np.array([0.2, -0.5, -0.5])
#t = np.array([0.2, 0.7, 0.7])
#t = np.array([0.09333098,  0.66210875,  0.63463856])
#iter_num = 0

inited = 0

if sim == "no_sim":
    r = Robot_raw(robot_name="no_sim")
    damping = 5
    error_vec = None
else:
    damping = 5
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
    if robot_name == "j2s7s300_link_base":
        motors = getAllMotorsJaco7(robot)
        sensors = getAndInitAllSensorsJaco7(robot)
    r = Robot_raw(motors=motors, sensors=sensors, robot_name=robot_name)
    initializeMotorsForPosition(motors)
    radius = 0.25
    height = 0.67
    error_vec = None
    drawCircle(radius, height, robot)
#    motors[6].setPosition(float('inf'))
#    motors[6].setVelocity(1.0)
#    exit()
    for motor in motors:
        motor.setVelocity(float('inf'))
# setting it right for the kuka lbr iiwa
    rot_mat_to_try1 = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    rot_mat_to_try2 = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])



# get me a curve yo
curve_parameter = 1
curve_parameter_step = 0.005
#curve_parameter_step = 0.01
radius = 0.3
height = 0.60
x_0 = -0.6
   
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

if sim == "sim":
    initForVideo(robot, r, gps)
    inited = 1


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

# first we need to initialize everything on the same position
    

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
            if r.robot_name == "base_link":
                ee_pos_gps = rot_mat_to_try1 @ ee_pos_gps
            if r.robot_name == "j2n6s300":
                ee_pos_gps = rot_mat_to_try1 @ ee_pos_gps
            if r.robot_name == "j2n6s300":
                ee_pos_gps = rot_mat_to_try1 @ ee_pos_gps
            if r.robot_name == "j2s7s300_link_base":
                ee_pos_gps = rot_mat_to_try2 @ ee_pos_gps
            current_joint_positions = readJointState(sensors)
            e = t - np.array(ee_pos_gps)


        error = np.sqrt(np.dot(e,e))
#        print("error:", error)
        #print("r.p_e:", r.p_e)
        #print("ee_pos_gps", ee_pos_gps)
        #print("t:", t)


        if sim == "sim" and inited == 1:
            if curve_parameter > 8.0:
                broj +=1
                curve_parameter = 1.0
                break
            curve_parameter += curve_parameter_step
            t = goInACirleViaPositionAroundLiftedX(radius, height, x_0, curve_parameter)
            t = rot_mat_to_try2 @ t
            e = t - r.p_e
            # error_vec is used if you want trajectory following via jacobian
            # TODO FIX!! (rn does not move at all)
#                e = goInACircleViaJacobian(curve_parameter)
#                print("e", e)
            print(curve_parameter)


        # for ik, give a random spot
        if (error_test(r, t) or n_of_tries_for_point > max_tries) and inited == 0:
            #print("we did it reddit")
            #exit()
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
                t = rot_mat_to_try2 @ t
                if curve_parameter >= 1.01:
                    current_joint_positions = readJointState(sensors)
                    print("current_joint_positions")
                    print(current_joint_positions)
                    inited = 1
#                t = np.array([random.uniform(-0.70, 0.70), random.uniform(-0.70, 0.70), random.uniform(-0.70, 0.70)])
                e = t - r.p_e
                # error_vec is used if you want trajectory following via jacobian
                # TODO FIX!! (rn does not move at all)
#                e = goInACircleViaJacobian(curve_parameter)
#                print("e", e)
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
#            del_thet = invKinm_Jac_T(r, t)
#            print(del_thet)
#            del_thet = del_thet * -1
#            del_thet[2] = del_thet[2] * -1
#            del_thet = invKinmQPSingAvoidE_kI(r, t)
#            del_thet = invKinmQP(r, t) / damping
#            del_thet = invKinmQPSingAvoidE_kI(r, t) / damping
            #print(del_thet)
#            del_thet = invKinmQPSingAvoidE_kM(r, t, error_vec) / damping
#            del_thet[0] = -1 * del_thet[0]
#            del_thet[1] = -1 * del_thet[1]
#            del_thet[2] = -1 * del_thet[2]
#            del_thet[3] = -1 * del_thet[3]
#            del_thet[4] = -1 * del_thet[4]
#            del_thet[5] = -1 * del_thet[5]
            del_thet = invKinmQPSingAvoidE_kM(r, t) / damping
#            del_thet = invKinm_dampedSquares(r, t)
        if broj == 1:
            del_thet = invKinmQPSingAvoidE_kI(r, t) / damping
        if broj == 2:
            del_thet = invKinmQPSingAvoidE_kM(r, t) / damping
        if broj == 3:
            del_thet = invKinmQPSingAvoidManipMax(r, t) / damping



        # move by calculated amount
        r.forwardKinmViaPositions(del_thet)


#
        ############ some debugging ############
#        q0 = np.array([1.1778, -1.5286,  2.0600,  2.9207, -1.0143, -0.2353])
        if 0 == 1:
            print("r.ndof")
            print(r.ndof)
            #q0 = np.array([np.pi / 2]*6)
            #q0 = np.array([np.pi] * 6)
            q0 = np.array([np.pi, np.pi, np.pi , np.pi,  0.0, np.pi])
           # q0 = np.array([0.0]*r.ndof)
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
            test = True 
            current_joint_positions = np.array([0.0] * r.ndof)
            while test:
                robot.step(timestep)
                r.forwardKinmNumericsOnlyDebug(q0)
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
           #     print("sensored positions")
           #     print(current_joint_positions)
           # for jointt in r.joints:
            #    print(jointt.theta)

            #q1 = np.array([0.0] * r.ndof)
#            q1= np.array([np.pi,  -np.pi /2,  np.pi/2,  \
#                    0.0,  0.0,  1.57079633])

#            q1 = np.array([-0.01497, 1.570796, -1.422393, -1.570507, -3.141536, 0.0])
#            q1 = np.array([-0.01497, 1.5708, -1.5708, -1.5708, -3.14159, 0.0])
            q1 = np.array([0.0, 1.5708, -1.5708, -1.5708, -3.14159, 0.0])
#            q1 = np.array([-0.01497, 1.5708, 1.5708, -1.5708, -3.14159, 0.0])
#            q1 = np.array([-0.014970197622824383, 1.57079632670909 ,-1.4223930882803404, \
#                            -1.570794433564159, -3.141592653581663 , 0.0])

#            q1 = np.array([0.0, 1.57079632670909 ,1.57079632670909, \
#                            -1.570794433564159, -3.141592653581663 , 0.0])

 #           q1 = np.array([-0.1611594433484704, 1.2889602934430824, -1.4243159418532632, -1.7331188642117612, -3.14159265318733, 0.0])

#            q1 = np.array([-0.20106614204974613, 1.2084377717951746, -1.4226039819277718, -1.604066203071168, -3.1270166104653656, 0.0])
            r.forwardKinmNumericsOnlyDebug2(q1)
            print("r.p_e")
            print(r.p_e)
            exit()
    #

