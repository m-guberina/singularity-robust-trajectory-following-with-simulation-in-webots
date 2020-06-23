import numpy as np
#import matplotlib.pyplot as plt 
#import mpl_toolkits.mplot3d.axes3d as p3
#from matplotlib.animation import FuncAnimation
#import matplotlib.colors as colr
import math
from webots_api_helper_funs import *

#from drawing import *
from joint_as_hom_mat import *


# we need an end effector (it's just a point as of now)
# however i wont add it yet
# it should have the 3 rotational degrees of freedom
# but that requires further considerations
# and it is not super-relevant at the moment


##########################################################3
# NOTES regarding mapping to webots
# i'll just comment drawing out for now
# however, it would be nice if i had both later on
# #crossvalidation
# #multiple_output_options

##########################################################3

# WHAT WEBOTS HAS AND THIS DID NOT
# - physics
# - many processes sorounding the controler
# - it's own API for everything 
# - remembers state 
# - which is then to be read from sensors (ex. positions of ee or motors)
##########################################################3


# add a drawing flag so that you can turn it on and off
class Robot_raw:
    #def __init__(self, clamp):
    def __init__(self, simulated_robot_motors, sensors):
        #self.clamp = clamp
        self.motors = simulated_robot_motors
        self.sensors = sensors
        self.clamp = 0
        self.joints = []
        fil = open('ur10e_dh_parameters_from_the_ur_site', 'r')
        params = fil.read().split('\n')
        params.pop()
        for p in params:
            p = p.split(';')
            self.joints.append(Joint(float(p[0]), float(p[1]), float(p[2]), float(p[3]), self.clamp))

        fil.close()
        self.jacobian = 0
        self.calcJacobian()

# drawing
#        self.anim_data = []
#        self.lines = []
        #initialize the plot for each line in order to animate 'em
        # each joint equates to four lines: 3 for orientation and 1 for the link
        # and each line has its x,y and z coordinate
        # thus 1 joint is: [x_hat, y_hat, z_hat, p] = [[x_hat1_x, x_hat_y, x_hat_z], [...]
        for j in range(len(self.joints)):
            x_hat = self.joints[j].HomMat[0:3,0]
            y_hat = self.joints[j].HomMat[0:3,1]
            z_hat = self.joints[j].HomMat[0:3,2]
            p = self.joints[j].HomMat[0:3,3]
# drawing
#            self.anim_data += [[x_hat, y_hat, z_hat, p]]
#            line_x, = plt.plot([],[]) 
#            line_y, = plt.plot([],[])
#            line_z, = plt.plot([],[])
#            line_p, = plt.plot([],[], 'g')
#            self.lines += [[line_x, line_y, line_z, line_p]]

# needed only for drawing, and even that is optional (remains to be seen)
    def saveConfToFile(self):
        fil = open('robot_parameters', 'r')
        params = fil.read().split('\n')
        fil.close()
        fil = open('robot_parameters', 'w')
        params.pop()
        back_to_string = ''
        for i in range(len(params)):
            params[i] = params[i].split(';')
            params[i][1] = self.joints[i].theta
            for j in range(4): #n of d-h params
                back_to_string += str(params[i][j])
                if j != 3:
                    back_to_string += ';'
                else:
                    back_to_string += '\n'
        fil.write(back_to_string) 
        fil.close()


# the jacobian is calulated for the end effector in base frame coordinates
#   TODO: the base frame ain't aligned with world coordinate frame!!!
#       ==> the only offset is on the z axis tho
#       ==> resolved by putting first joint's d parameter to height, there is no x nor y offset
# the simulation is controlled via motors
# we do calculations by reading from the sensors 
# and we send the results to the motors
    def calcJacobian(self):
        z_is = [np.array([0,0,1])]
        p_is = [np.array([0,0,0])]
        toBase = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        self.p_e = np.array([0.,0.,0.])
        
        for j in range(len(self.joints)):
            toBase = toBase @ self.joints[j].HomMat
            z_is.append(toBase[0:3, 2])
            p_is.append(toBase[0:3, 3])
        p_e = p_is[-1]
        self.p_e = p_is[-1]
        jac = np.array([0,0,0,0,0,1]).reshape(6,1)
# we doin only revolute joints still 

        for j in range(len(self.joints)):
            j_p = np.cross(z_is[j], p_e - p_is[j]).reshape(3,1)
            j_o = z_is[j].reshape(3,1)
            j_i = np.vstack((j_p, j_o))
            jac = np.hstack((jac, j_i))
        self.jacobian = jac[0:6,1:]
#        print("jacobian incoming")
#        print(self.jacobian)
        self.jac_tri = self.jacobian[0:3,:]
            

    def drawState(self, ax, color_link):
        toBase = [np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])]
        drawOrientation(ax, toBase[-1][0:3,0:3], np.zeros((3,)))
        for j in range(len(self.joints)):
            toBase.append(toBase[-1] @ self.joints[j].HomMat)
            drawOrientation(ax, toBase[-1][0:3,0:3], toBase[-1][0:3,3])
            drawVector(ax, -1* ( toBase[-2][0:3,3] - toBase[-1][0:3,3] ), toBase[-2][0:3,3],color_link)

   # the thetas are to be read from position sensors 
    def updateJointsAndJacobian(self):
        thetas = np.array(readJointState(self.sensors))
# potential clamping for joint rotation limits
        for i in range(len(thetas)):
            if self.clamp == 1:
                self.joints[i].rotate_numerically(clampTheta(thetas[i]), self.clamp)
            else:
                self.joints[i].rotate_numerically(thetas[i], self.clamp)
        self.calcJacobian()


    def forwardKinmViaPositions(self, thetas, simulated_robot_motors, sensors):
# potential clamping for joint rotation limits
#   ==> ur10e does not have joint limits, but other robots might
# the horribly arcsin(sin(x)) is here because it motors take positions from -pi to pi
# it is highly advisable that this be calculated by a modulus or something along those lines
# because calculating transcendental functions is way more expensive
        for i in range(len(thetas)):
            if self.clamp == 1:
                #TODO write this out
                self.joints[i].rotate(clampTheta(self.joints[i].theta + thetas[i]), self.clamp)
            else:
                self.motors[i].setPosition(np.arcsin(np.sin(np.array(self.joints[i].theta + thetas[i]))))
            # now we update the joints and calculate the jacobian
            # perhaps that should be done before motor update, idk, TODO try it out
        self.updateJointsAndJacobian()


    def forwardKinmViaVelocities(self, thetas):
# potential clamping for joint velocity limits
#   ==> done in the ik method!
        for i in range(len(thetas)):
            if self.clamp == 1:
                # TODO
                self.joints[i].rotate(clampTheta(self.joints[i].theta + thetas[i]), self.clamp)
            else:
                self.motors[i].setVelocity(thetas[i])
        # this should probably be done in a smarter way
        self.updateJointsAndJacobian()




# in webots, i get this for free with a position sensor
# but since such a device is not available in reality.
# thus, the gps device in webots shall be used to measure accuracy.
    def eePositionAfterForwKinm(self, thetas):
        joints2 = self.joints
        for i in range(len(thetas)):
            joints2[i].rotate(joints2[i].theta + thetas[i])

        toBase = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        
        for j in range(len(joints2)):
            toBase = toBase @ joints2[j].HomMat
        p_e = toBase[0:3,3]
        return p_e
        
    
    def bruteAnimForwKinm(self, ax, thetas):
        shots = np.linspace(0,1,20)
        for shot in shots:
            for i in range(len(thetas)):
                self.joints[i].rotate(self.joints[i].theta + thetas[i] * shot)
            self.drawState(ax, 'bisque')
        self.calcJacobian()
        self.drawState(ax, 'b')

    
    # remake this into the FuncAnimation update function
    # divide each angle from the current to desired angle in the same number of steps
    # then for each joint and each of its steps calculate the new position,
    # and append it in the appropriate "line" list
# there is no set_data() for 3dim, so we use set_3_properties() for the z axis
def forwardKinAnim(frame, r, thetas, n_of_frames, ax):
    thetas_sliced = []
#    print("frame")
 #   print(frame)
    for j in range(len(r.joints)):
        # i have no idea why but the sum of frame(i)/n_of_frames adds up to 0.5
        # we need it to add to to 1 so that in total the manipulator rotates by the specified angles
        thetas_sliced.append(thetas[j] * (2 * frame / n_of_frames))
    r.forwardKinm(thetas_sliced)
    toBase = [np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])]
    x_hat_dat =[[0],[0],[0]]
    y_hat_dat =[[0],[0],[0]]
    z_hat_dat =[[0],[0],[0]]
    p_dat = [[0],[0],[0]]
    for j in range(len(r.joints)):
        toBase.append(toBase[-1] @ r.joints[j].HomMat)
        orientation = toBase[-1][0:3,0:3] 
        x_hat = orientation[0:3,0]  + toBase[-1][0:3,3]
        y_hat = orientation[0:3,1] + toBase[-1][0:3,3]
        z_hat = orientation[0:3,2] + toBase[-1][0:3,3]
        p = (-1* ( toBase[-2][0:3,3] - toBase[-1][0:3,3] ) + toBase[-2][0:3,3])
        for i in range(3):
            x_hat_dat[i].append(x_hat[i])
            y_hat_dat[i].append(y_hat[i])
            z_hat_dat[i].append(z_hat[i])
            p_dat[i].append(p[i])
#            r.lines[j][0].set_data(x_hat_dat[0], x_hat_dat[1])
#            r.lines[j][0].set_3d_properties(x_hat_dat[2])
#            r.lines[j][1].set_data(y_hat_dat[0], y_hat_dat[1])
#            r.lines[j][1].set_3d_properties(y_hat_dat[2])
#            r.lines[j][2].set_data(z_hat_dat[0], z_hat_dat[1])
#            r.lines[j][2].set_3d_properties(z_hat_dat[2])
            r.lines[j][3].set_data(p_dat[0], p_dat[1])
            r.lines[j][3].set_3d_properties(p_dat[2])
#    print(frame / frame_length)
    if frame == 1:
        r.drawState(ax, 'r')

