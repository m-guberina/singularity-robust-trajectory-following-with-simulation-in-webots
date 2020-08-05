"c""control_test controller."""

from forw_kinm import *
from inv_kinm import *
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



print("eto me")


# Main loop:
# - perform simulation steps until Webots is stopping the controller
#t = np.array([0.07,0.7,0.75929])
#t = np.array([-0.60,0.07,0.75929])
t = np.array([-0.59702256, -0.424394371, 0.64633786])
iter_num = 0
r = Robot_raw()


# get me a curve yo
curve_parameter = 1
curve_parameter_step = 0.1
radius = 0.35
height = 0.67
x_0 = -0.99
   
#iter_max = 1000

# initialize a file in which measurements are to be stored
# for later analysis and visualization
# manipulability measure is left column
# and the smallest eigenvalue is the right column
#measurements_file = open("./data/sing_av_pinv_mem_data", "w")
measurements_file_no_sing_avoid = open("./data/no_sing_avoid_200_inv_kinms", "w")
measurements_file_E_kI = open("./data/E_kI_200_inv_kinms", "w")
measurements_file_E_kM = open("./data/E_kM_200_inv_kinms", "w")


for broj in range(4):
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
    while number_of_points < 50:
        n_of_tries_for_point += 1


        e = t - r.p_e
        error = np.sqrt(np.dot(e,e))
        #print("error:", error)
        #print("r.p_e:", r.p_e)
        #print("t:", t)




        # for ik, give a random spot
        if error < 0.01 or n_of_tries_for_point > 100:
            if(n_of_tries_for_point > 50):
                print("FAILED TO CONVERGE in", n_of_tries_for_point, "steps!!!")
                print("i got to", r.p_e, "and the error is:", error)
            else:
                print("i did it in,", n_of_tries_for_point, "steps")

            # write final configuration
            # right from the paper
            M = r.jac_tri @ r.jac_tri.T
            manip_index = np.sqrt(np.linalg.det(M))
            # let's try with diagonal entries of the diag. mat. in SVD decomposition
#            eigenvals, eigvecs = np.linalg.eig(M)
#            measurements_file.write(str(manip_index) + ";" + str(eigenvals[eigenvals.argmin()]) + ";" + str(eigenvals[eigenvals.argmax()]) + "\n")
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
        #curve_parameter += curve_parameter_step

        # now write this to the measurements file
        # and stop after you have finished going around the shape
    #    if curve_parameter > 16.0:
    #        print("WE DONE")
    #        sys.exit(0)


        # here you choose which ik method you want
        # just pass the robot_raw instance and the target position
        # they use the calculated position of ee
        # of course that can be modified
        if broj == 0:
            #del_thet = invKinm_Jac_T(r, t)
            del_thet = invKinmQP(r, t)
        if broj == 1:
            del_thet = invKinmQPSingAvoidE_kM(r, t) / 3
        if broj == 2:
            del_thet = invKinmQPSingAvoidE_kI(r, t) / 5

        # move by calculated amount
        r.forwardKinmNumericsOnly(del_thet)


