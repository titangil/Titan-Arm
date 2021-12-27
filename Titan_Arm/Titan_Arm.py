import pybullet as sim
import time
import pybullet_data
import math as m
import numpy as np
import keyboard

x= 0 #max 0.96
y= 0 #max 0.96
z= 0.3  #max 0.57


physicsclient = sim.connect(sim.GUI)
sim.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
sim.setGravity(0, 0, -9.8)
planeId = sim.loadURDF("plane.urdf")
path = "/home/titan/Robot/Titan_Arm/Titan_Arm.urdf"
# Get joint number


# Get jointInfo



# Set Control parameters
joint1 = 2
joint2 = 5
joint3 = 7
joint4 = 8
joint5 = 11
joint6 = 12
mode = sim.POSITION_CONTROL
# Loading the URDF file of robot
armID = sim.loadURDF(path, useFixedBase=True, basePosition=[0, 0,0])
jointNum = sim.getNumJoints(armID)
for jointIndex in range(0,jointNum):

    jointInfo = sim.getJointInfo(armID, jointIndex)
    print( "joint information", jointInfo)

#maxForceId = sim.addUserDebugParameter("maxForce", 0, 100, 1)
'''joint1Id = sim.addUserDebugParameter("revjoint1", -135.0, 135.0, 0.0)
joint2Id = sim.addUserDebugParameter("revjoint2", -135.0, 135.0, 0.0)
joint3Id = sim.addUserDebugParameter("prisjoint1", -0.33, 0.0, 0.0)
joint4Id = sim.addUserDebugParameter("revjoint3", -135, 135, 0.0)'''

#print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
if True:
    print(""" ───────▄▀▀▀▀▀▀▀▀▀▀▄▄
 ────▄▀▀░░░░░░░░░░░░░▀▄
 ──▄▀░░░░░░░░░░░░░░░░░░▀▄
 ──█░░░░░░░░░░░░░░░░░░░░░▀▄
 ─▐▌░░░░░░░░▄▄▄▄▄▄▄░░░░░░░▐▌
 ─█░░░░░░░░░░░▄▄▄▄░░▀▀▀▀▀░░█
 ▐▌░░░░░░░▀▀▀▀░░░░░▀▀▀▀▀░░░▐▌
 █░░░░░░░░░▄▄▀▀▀▀▀░░░░▀▀▀▀▄░█
 █░░░░░░░░░░░░░░░░▀░░░▐░░░░░▐▌
 ▐▌░░░░░░░░░▐▀▀██▄░░░░░░▄▄▄░▐▌
 ─█░░░░░░░░░░░▀▀▀░░░░░░▀▀██░░█
 ─▐▌░░░░▄░░░░░░░░░░░░░▌░░░░░░█                   Hello there
 ──▐▌░░▐░░░░░░░░░░░░░░▀▄░░░░░█
 ───█░░░▌░░░░░░░░▐▀░░░░▄▀░░░▐▌
 ───▐▌░░▀▄░░░░░░░░▀░▀░▀▀░░░▄▀
 ───▐▌░░▐▀▄░░░░░░░░░░░░░░░░█
 ───▐▌░░░▌░▀▄░░░░▀▀▀▀▀▀░░░█
 ───█░░░▀░░░░▀▄░░░░░░░░░░▄▀
 ──▐▌░░░░░░░░░░▀▄░░░░░░▄▀
 ─▄▀░░░▄▀░░░░░░░░▀▀▀▀█▀
 ▀░░░▄▀░░░░░░░░░░▀░░░▀▀▀▀▄▄▄▄▄
 ==============================================================================
    """)
#print(sim.getJointInfo(armID,7))
count = 0
force = 100000
while True:
    
    #print()
    x =float(input("Enter X: "))
    #print("Enter Y:")
    y =float(input("Enter Y: "))
    #print()
    z =float(input("Enter Z: "))
    alpha =float(input("Enter aplha: "))
    grip = str(input("Enter gripper state: "))
    print('\n')
    e = 0
    if y < 0:
        e = 1
        y = -1*y

    theata2 = np.degrees(np.arccos((np.power(x,2)+np.power(y,2)-0.4608)/0.4608))
    t1 = np.degrees(np.arctan2(y,x))
    t2 = np.degrees(np.arctan2((0.48*np.sin(np.radians(theata2))), (0.48 + (0.48*np.cos(np.radians(theata2))))))   
    theata1  = t1-t2
    d= z-0.27

    D= np.sqrt( (np.power(x,2)) + (np.power(y,2)) )

    print("Theata 1: ",theata1)
    print("Theata 2: ",theata2)
    if d < -0.26:
        print("d: ","-0.26","\n")
    elif d > 0.2:
        print("d: ","0.2","\n")
    else:
        print("d: ",d,"\n")
    print("Please wait....\n")
    while physicsclient == 0:
        if D > 0.98 or x <0 or theata2 > 135:
            print("[ERROR : out of range]\n")
            print("==============================================================================\n")
            break
            break
        '''j1Pos = sim.readUserDebugParameter(joint1Id)
        j2Pos = sim.readUserDebugParameter(joint2Id)
        j3Pos = sim.readUserDebugParameter(joint3Id)
        j4Pos = sim.readUserDebugParameter(joint4Id)'''
        #maxForce = sim.readUserDebugParameter(maxForceId)
        
        # Set target position
        '''if e == 1:
            sim.setJointMotorControl2(armID, joint1, controlMode=mode, targetPosition=m.radians(-theata1),targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint2, controlMode=mode, targetPosition=m.radians(-theata2),targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint4, controlMode=mode, targetPosition=m.radians(theata1+theata2+alpha),targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
        else:
            sim.setJointMotorControl2(armID, joint1, controlMode=mode, targetPosition=m.radians(theata1),targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint2, controlMode=mode, targetPosition=m.radians(theata2),targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint4, controlMode=mode, targetPosition=m.radians(-theata1-theata2+alpha),targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
        sim.setJointMotorControl2(armID, joint3, controlMode=mode, targetPosition=d,targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
        if grip == "close" or grip == "":
            sim.setJointMotorControl2(armID, joint5, controlMode=mode, targetPosition=-0.04,targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint6, controlMode=mode, targetPosition= 0.04,targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
        elif grip == "open":
            sim.setJointMotorControl2(armID, joint5, controlMode=mode, targetPosition=-0.00,targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint6, controlMode=mode, targetPosition= 0.00,targetVelocity=0,force=10000, positionGain=50, velocityGain=50, maxVelocity=1)'''

        if e == 1:
            sim.setJointMotorControl2(armID, joint1, controlMode=mode, targetPosition=m.radians(-theata1),force=10000, maxVelocity=0.5)
            sim.setJointMotorControl2(armID, joint2, controlMode=mode, targetPosition=m.radians(-theata2),force=10000, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint4, controlMode=mode, targetPosition=m.radians(theata1+theata2+alpha),force=10000, maxVelocity=1)
        else:
            sim.setJointMotorControl2(armID, joint1, controlMode=mode, targetPosition=m.radians(theata1),force=10000,  maxVelocity=0.5)
            sim.setJointMotorControl2(armID, joint2, controlMode=mode, targetPosition=m.radians(theata2),force=10000,  maxVelocity=1)
            sim.setJointMotorControl2(armID, joint4, controlMode=mode, targetPosition=m.radians(-theata1-theata2+alpha),force=10000,  maxVelocity=1)
        sim.setJointMotorControl2(armID, joint3, controlMode=mode, targetPosition=d,force=10000, maxVelocity=1)
        if grip == "close" or grip == "":
            sim.setJointMotorControl2(armID, joint5, controlMode=mode, targetPosition=-0.04,force=10000, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint6, controlMode=mode, targetPosition= 0.04,force=10000, maxVelocity=1)
        elif grip == "open":
            sim.setJointMotorControl2(armID, joint5, controlMode=mode, targetPosition=-0.00,force=10000, maxVelocity=1)
            sim.setJointMotorControl2(armID, joint6, controlMode=mode, targetPosition= 0.00,force=10000, maxVelocity=1)

        

        count = count+1
        #countdeci = count/100
        #countdown = np.round((3-countdeci),0)
        
        #print("Please wait ", np.round(countdown,1) , " seconds")
        if  keyboard.is_pressed('space') :
            count = 0
            print("==============================================================================\n")
            break
        #pos,ore= sim.getJointState(armID,)
        #rint(pos,ore)
        #pos = sim.getLinkState(armID,7)
        #print(pos)
        #pos,aa,bb,cc,dd,ee  = sim.getLinkState(armID,6)
        #print(pos)
        sim.stepSimulation()
        time.sleep(0.01)