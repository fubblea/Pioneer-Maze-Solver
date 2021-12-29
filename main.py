import math
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

try:
    while True:
        #Update Sonar Array
        for i in range(8):        
            returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(
                clientID=clientID,
                sensorHandle=sensorArrayFront[i].sensorHandle,
                operationMode=sim.simx_opmode_buffer
                )
            
            sensorArrayFront[i].updateValues(detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector)

        
        if sensorArrayFront[3].distance <= 0.2 or sensorArrayFront[4].distance <= 0.2 or sensorArrayFront[5].distance <= 0.2:
            if sensorArrayFront[0].distance > sensorArrayFront[7].distance:
                sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=0, operationMode=sim.simx_opmode_oneshot_wait)
                sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=0.6, operationMode=sim.simx_opmode_oneshot_wait)
                sim.simxAddStatusbarMessage(clientID, message="1a", operationMode=sim.simx_opmode_oneshot_wait)
            else:
                sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=1, operationMode=sim.simx_opmode_oneshot_wait)
                sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=0.3, operationMode=sim.simx_opmode_oneshot_wait)
                sim.simxAddStatusbarMessage(clientID, message="1b", operationMode=sim.simx_opmode_oneshot_wait)
        
        elif sensorArrayFront[7].distance >= 0.2:
            sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=1, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=0.3, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, message="2", operationMode=sim.simx_opmode_oneshot_wait)
        
        elif sensorArrayFront[0].distance <= 0.2:
            sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=0.6, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, message="3", operationMode=sim.simx_opmode_oneshot_wait)
        
        elif sensorArrayFront[7].distance <= 0.2:
            sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=0.6, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, message="4", operationMode=sim.simx_opmode_oneshot_wait)
        
        else:
            sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=0.6, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=0.6, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, message="5", operationMode=sim.simx_opmode_oneshot_wait)

except KeyboardInterrupt:
    sim.simxStopSimulation(clientID, operationMode=sim.simx_opmode_oneshot_wait)