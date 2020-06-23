import numpy as np

# deriving the homogeneus transformation matrix
# as a series of transformations between two coordinate frames
# detailed description: Wikipedia article on Denavit-Hartenberg parameters

# this function is wrong!!!!!!!!!!!!!1
def DHtoHomMat(d, theta, r, alpha):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    # the d: translation along i^th z-axis until the common normal
    trans_z_n_prev = np.array([[1,0,0], [0,1,0], [0,0,1]])
    trans_z_n_prev = np.hstack((trans_z_n_prev, np.array([0,0,d]).reshape(3,1)))
    trans_z_n_prev = np.vstack((trans_z_n_prev, np.array([0,0,0,1])))

    # the theta: rotation about i^th z-axis until i^th x-axis coincides with the common normal
    # this is the parameter we get to control
    rot_z_n_prev = np.array([[ct,-1*st,0], [st,ct,0], [0,0,1]])
    rot_z_n_prev= np.hstack((rot_z_n_prev, np.array([0,0,1]).reshape(3,1)))
    rot_z_n_prev= np.vstack((rot_z_n_prev, np.array([0,0,0,1])))
    
    # the r: traslation along the i^th x-axis, now common normal, until the origin of i+1^th frame
    trans_x_n = np.array([[1,0,0], [0,1,0], [0,0,1]])
    trans_x_n = np.hstack((trans_x_n, np.array([r,0,0]).reshape(3,1)))
    trans_x_n = np.vstack((trans_x_n, np.array([0,0,0,1])))
    
    # the a: rotation about common normal so that z-axis aling 
    rot_x_n = np.array([[1,0,0], [0,ca,-1*sa], [0,sa,ca]])
    rot_x_n = np.hstack((rot_x_n, np.array([0,0,0]).reshape(3,1)))
    rot_x_n = np.vstack((rot_x_n, np.array([0,0,0,1])))

    return trans_z_n_prev @ rot_z_n_prev @ trans_x_n @ rot_x_n

# the above, but multiplied into the final form (less calculation needed, hence better)
# also correct
def createDHMat(d, theta, r, alpha):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    DHMat = np.array([ [ct, -1 * st * ca, st * sa,      r * ct], \
                       [st,      ct * ca, -1 * ct * sa, r * st], \
                       [0,        sa,        ca,         d], \
                       [0,        0,       0,           1]  ])
    return DHMat

# clamping of all joints to rotate only by 7/8 of the cirle
# of course this can be changed to other limits as needed
# however, it is hardcoded here because it's good enough for demonstrative purposes
# the ur arms all do not have joint limits!
def clampTheta(theta):
    if theta > np.pi * 7/4:
        theta = np.pi * 7/4
    if theta < -1 * (np.pi * 7/4):
        theta = -1 * (np.pi * 7/4)
    return theta

# if clamp is == 0 then don't clamp and if it is 1 then do clamp the angles
# this class does not reprezent a geometric entity, but rather the coordinate system
# in which kinematics is defined for a particular joint
# and that coordinate system is defined via the D-H parameters
class Joint:
    def __init__(self, d, theta, r, alpha, clamp):
        self.d = d
        self.clamp = clamp
# potential clamping for joint rotation limits
        if clamp == 1:
            self.theta = clampTheta(theta)
        else:
            self.theta = theta
        self.r = r
        self.alpha = alpha
        self.HomMat = createDHMat(self.d, self.theta, self.r, self.alpha)
   

# we simply recalculate the homogenous transformation matrix for the given joint
# and that equates to it being rotated
# the timely rotation is simply the rotation broken into discete parts
########################################################################
# this is fundamentaly different from how the physics simulation works!
# there rotation equates to setting positions/velocities to individual motors,
# which then do the moving.
# there you can only SENSE via SENSORS what the result of the rotation is
# DO CONTROL, not numerics!
########################################################################
# the solution is simple:
# do the rotation periodically and "rotate" by the angles given by reading PositionSensors,
# i.e. read the current state in the simulation and transport it here where we can do math

# bottom line
#############
# the class joint reprezents the abstract coordinate systems we need to 
# calculate the jacobian
# thus the collection of all joints stores the state (the current angles)
# and allows for that state to be mapped to the jacobian
# thus, the simulation control is not done here, but in the Robot class
# via the forward kinematics method


    def rotate_numerically(self,theta, clamp):
# potential clamping for joint rotation limits
        if self.clamp == 1:
            self.theta = clampTheta(theta)
        else:
            self.theta = theta
        self.HomMat = createDHMat(self.d, self.theta, self.r, self.alpha)

    def __repr__(self):
        return str(self.HomMat)
