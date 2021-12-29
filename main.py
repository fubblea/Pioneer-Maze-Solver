import math
import sys

import numpy as np

import sim
from objectWrappers import Sensor

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print("Connected to remote API server")
else:
    sys.exit("Connection unsuccessful")

sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_oneshot)

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
PI = math.pi

sensorArrayFront = []
sensorPositions = [-0.5*PI, -(5*PI)/18, -PI/6, -PI/18, PI/18, PI/6, (5*PI)/18, 0.5*PI]

for i in range(1, 9):
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
    
    sensor = Sensor(sensorHandle=sensorHandle, detectionState=detectionState, detectedPoint=detectedPoint, detectedObjectHandle=detectedObjectHandle, detectedSurfaceNormalVector=detectedSurfaceNormalVector, position=sensorPositions[i-1], sensorNumber=i)
    sensorArrayFront.append(sensor)

sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=0.6, operationMode=sim.simx_opmode_oneshot_wait)
sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=0.6, operationMode=sim.simx_opmode_oneshot_wait)

while True:
    for i in range(8):        
        returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(
            clientID=clientID,
            sensorHandle=sensorArrayFront[i].sensorHandle,
            operationMode=sim.simx_opmode_buffer
            )
        
        sensorArrayFront[i].updateValues(detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector)
    
    print(sensorArrayFront[4].detectedObjectHandle)
    