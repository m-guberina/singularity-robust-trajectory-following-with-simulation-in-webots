#########################################################3
# for now only for ur10e, later write out for other robots
#########################################################3


def getAllMotors(robot):
    motors = []
    motors.append(robot.getMotor('shoulder_pan_joint'))
    motors.append(robot.getMotor('shoulder_lift_joint'))
    motors.append(robot.getMotor('elbow_joint'))
    motors.append(robot.getMotor('wrist_1_joint'))
    motors.append(robot.getMotor('wrist_2_joint'))
    motors.append(robot.getMotor('wrist_3_joint'))
    return motors



def setMotorSpeeds(motors, speeds):
    for i in range(len(motors)):
        motors[i].setVelocity(speeds[i])



def initializeMotorsForVelocity(motors):
    speeds = []
    for motor in motors:
        motor.setPosition(float('inf'))
        speeds.append(0)

    
    setMotorSpeeds(motors, speeds)



def initializeMotorsForPosition(motors):
    speeds = []
    for motor in motors:
#        motor.setVelocity(float('inf'))
        speeds.append(0)

    
    setMotorSpeeds(motors, speeds)


def getAndInitAllSensors(robot):
    sensors = []
    sensors.append(robot.getPositionSensor('shoulder_pan_joint_sensor'))
    sensors.append(robot.getPositionSensor('shoulder_lift_joint_sensor'))
    sensors.append(robot.getPositionSensor('elbow_joint_sensor'))
    sensors.append(robot.getPositionSensor('wrist_1_joint_sensor'))
    sensors.append(robot.getPositionSensor('wrist_2_joint_sensor'))
    sensors.append(robot.getPositionSensor('wrist_3_joint_sensor'))
   
   # now we must specify how often do sensors get read in miliseconds
   # i've put 10ms since this seems like it should work smoothly,
   # but ofc you should play with this value later on
    for sensor in sensors:
        sensor.enable(10)

    return sensors

def readJointState(sensors):
    joint_positions = []
    for sensor in sensors:
        joint_positions.append(sensor.getValue())
    return joint_positions


