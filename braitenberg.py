import math
from ntpath import join
import sys
import time

import sim
from objectWrappers import Sensor

# simRemoteApi.start(19999)

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print("Connected to remote API server")
else:
    sys.exit("Connection unsuccessful")

sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_oneshot_wait)

# Get handles
# Motors
returnCode, leftMotor = sim.simxGetObjectHandle(
                            clientID=clientID, 
                            objectName='Pioneer_p3dx_leftMotor', 
                            operationMode=sim.simx_opmode_oneshot_wait
                        )

returnCode, rightMotor = sim.simxGetObjectHandle(
                            clientID=clientID, 
                            objectName='Pioneer_p3dx_rightMotor', 
                            operationMode=sim.simx_opmode_oneshot_wait
                        )

# Sonar Array
sensorArray = []

for i in range(1, 17):
    returnCode, sensorHandle = sim.simxGetObjectHandle(
        clientID=clientID,
        objectName=f"Pioneer_p3dx_ultrasonicSensor{i}",
        operationMode=sim.simx_opmode_oneshot_wait
        )
    
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(
        clientID=clientID,
        sensorHandle=sensorHandle,
        operationMode=sim.simx_opmode_streaming
        )
    
    sensor = Sensor(sensorHandle=sensorHandle, detectionState=detectionState, detectedPoint=detectedPoint, detectedObjectHandle=detectedObjectHandle, detectedSurfaceNormalVector=detectedSurfaceNormalVector, sensorNumber=i)
    sensorArray.append(sensor)

NODETECTIONDIST = 0.2
MAXDETECTIONDIST = 0.1
detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
v0=2

try:
    while True:
        #Update Sonar Array
        for i in range(16):        
            returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(
                clientID=clientID,
                sensorHandle=sensorArray[i].sensorHandle,
                operationMode=sim.simx_opmode_buffer
                )
            
            sensorArray[i].updateValues(detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector)

        for i in range(len(sensorArray)):
            if sensorArray[i].distance < NODETECTIONDIST:
                if sensorArray[i].distance < MAXDETECTIONDIST:
                    sensorArray[i].distance = MAXDETECTIONDIST
                
                detect[i] = 1 - ((sensorArray[i].distance - MAXDETECTIONDIST) / (NODETECTIONDIST - MAXDETECTIONDIST))
            else:
                detect[i] = 0

        vLeft = v0
        vRight = v0

        for i in range(16):
            vLeft = vLeft + braitenbergL[i] * detect[i]
            vRight = vRight + braitenbergR[i] * detect[i]
        
        print(vLeft, vRight)
        
        sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=vLeft, operationMode=sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=vRight, operationMode=sim.simx_opmode_oneshot_wait)

except KeyboardInterrupt:
    sim.simxStopSimulation(clientID, operationMode=sim.simx_opmode_oneshot_wait)