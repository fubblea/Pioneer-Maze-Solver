import sys

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
returnCode, robot = sim.simxGetObjectHandle(
                            clientID=clientID, 
                            objectName='Pioneer_p3dx', 
                            operationMode=sim.simx_opmode_oneshot_wait
                        )

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

returnCode, finishLine = sim.simxGetObjectHandle(
                            clientID=clientID, 
                            objectName='finish_line', 
                            operationMode=sim.simx_opmode_oneshot_wait
                        )
returnCode, collisionState = sim.simxCheckCollision(clientID, entity1=robot, entity2=finishLine, operationMode=sim.simx_opmode_streaming)

# Key Robot Parameters
NODETECTIONDIST = 0.15
V0 = 1

try:
    while collisionState == False:
        #Update Sonar Array
        for i in range(16):        
            returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(
                clientID=clientID,
                sensorHandle=sensorArray[i].sensorHandle,
                operationMode=sim.simx_opmode_buffer
                )
            
            sensorArray[i].updateValues(detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector)

        right = sensorArray[7].distance <= NODETECTIONDIST
        front = (sensorArray[2].distance <= NODETECTIONDIST) or (sensorArray[3].distance <= NODETECTIONDIST) or (sensorArray[4].distance <= NODETECTIONDIST) or (sensorArray[5].distance <= NODETECTIONDIST)

        if front == False and right == False:
            sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=V0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=V0/3, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, message="1", operationMode=sim.simx_opmode_oneshot_wait)
        
        elif front == True and right == False:
            sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=V0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, message="2", operationMode=sim.simx_opmode_oneshot_wait)
        
        elif front == False and right == True:
            sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=V0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=V0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, message="3", operationMode=sim.simx_opmode_oneshot_wait)
        
        elif front == True and right == True:
            sim.simxSetJointTargetVelocity(clientID, jointHandle=leftMotor, targetVelocity=0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, jointHandle=rightMotor, targetVelocity=V0, operationMode=sim.simx_opmode_oneshot_wait)
            sim.simxAddStatusbarMessage(clientID, message="4", operationMode=sim.simx_opmode_oneshot_wait)
        
        else:
            sim.simxAddStatusbarMessage(clientID, message="No Case", operationMode=sim.simx_opmode_oneshot_wait)
        
        returnCode, collisionState = sim.simxCheckCollision(clientID, entity1=robot, entity2=finishLine, operationMode=sim.simx_opmode_buffer)
                

except KeyboardInterrupt:
    sim.simxStopSimulation(clientID, operationMode=sim.simx_opmode_oneshot_wait)