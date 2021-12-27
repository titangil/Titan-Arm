import pybullet as sim
import time
import pybullet_data
import math
physicsclient = sim.connect(sim.GUI)
sim.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
sim.setGravity(0, 0, -9.8)
planeId = sim.loadURDF("plane.urdf")
path = "/home/titan/Robot/Titan_Arm/planar_robot.urdf" # Loading the URDF file of robot
armID = sim.loadURDF(path, useFixedBase=True, basePosition=[0, 0,0])
# Get joint number
jointNum = sim.getNumJoints(armID)
print ("joint number is", jointNum)
# Get jointInfo
for jointIndex in range(0,jointNum):

    jointInfo = sim.getJointInfo(armID, jointIndex)
    print ("joint information", jointInfo)

# Set Control parameters
joint1 = 0
#joint2 = 1
#joint3 = 2
#joint4 = 5  #####

mode = sim.POSITION_CONTROL

# User Interface for joint's angle adjustment
maxForceId = sim.addUserDebugParameter("maxForce", 0, 100, 1)
joint1Id = sim.addUserDebugParameter("Joint1", -90.0, 90.0, 0.0)
#joint2Id = sim.addUserDebugParameter("Joint2", -90.0, 90.0, 0.0)
#joint4Id = sim.addUserDebugParameter("Joint4", -90.0, 90.0, 0.0)########


while physicsclient == 0:
    # Update values from slide bar

    j1Pos = sim.readUserDebugParameter(joint1Id)
    #j2Pos = sim.readUserDebugParameter(joint2Id)
    #j4Pos = sim.readUserDebugParameter(joint4Id)#########
    maxForce = sim.readUserDebugParameter(maxForceId)

    # Set target position

    sim.setJointMotorControl2(armID, joint1, controlMode=mode, targetPosition=math.radians(j1Pos),

    force=maxForce)

    #sim.setJointMotorControl2(armID, joint2, controlMode=mode, targetPosition=math.radians(j2Pos),

    #
    #sim.setJointMotorControl2(armID, joint4, controlMode=mode, targetPosition=math.radians(j2Pos),########

    #force=maxForce)##########


    sim.stepSimulation()
    time.sleep(0.01)