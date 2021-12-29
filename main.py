import math
import sys

import numpy as np

import sim

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
class Sensor:
    def __init__(self, sensorNumber, sensorHandle, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, position) -> None:
        self.sensorNumber = sensorNumber
        self.sensorHandle = sensorHandle
        self.detectionState = detectionState
        self.detectedPoint = detectedPoint
        self.distance = np.linalg.norm(detectedPoint)
        self.detectedObjectHandle = detectedObjectHandle
        self.detectedSurfaceNormalVector = detectedSurfaceNormalVector
        self.position = position
    
    def __str__(self) -> str:
        return f"Sensor Number: {self.sensorNumber}, Detected Object: {self.detectionState}, Distance: {self.distance}, Position: {self.position} rad"

sensorArrayFront = []
sensorArrayBack = []

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

for i in range(9, 17):
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
    
    sensor = Sensor(sensorHandle=sensorHandle, detectionState=detectionState, detectedPoint=detectedPoint, detectedObjectHandle=detectedObjectHandle, detectedSurfaceNormalVector=detectedSurfaceNormalVector, position=sensorPositions[8-(i-1)], sensorNumber=i)
    sensorArrayBack.append(sensor)