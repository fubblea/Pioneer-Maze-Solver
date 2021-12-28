import sim
import sys
import numpy as np

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print("Connected to remote API server")
else:
    sys.exit("Connection unsuccessful")

sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_oneshot)

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

returnCode, ultrasonicSensor12 = sim.simxGetObjectHandle(
                                    clientID=clientID, 
                                    objectName='Pioneer_p3dx_ultrasonicSensor12', 
                                    operationMode=sim.simx_opmode_oneshot_wait
                                )

returnCode = sim.simxSetJointTargetVelocity(
                clientID=clientID,
                jointHandle=leftMotor,
                targetVelocity=0.3,
                operationMode=sim.simx_opmode_oneshot
            )

returnCode = sim.simxSetJointTargetVelocity(
                clientID=clientID,
                jointHandle=rightMotor,
                targetVelocity=0.3,
                operationMode=sim.simx_opmode_oneshot_wait
            )

returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(
    clientID=clientID,
    sensorHandle=ultrasonicSensor12,
    operationMode=sim.simx_opmode_streaming
    )

while True:
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(
    clientID=clientID,
    sensorHandle=ultrasonicSensor12,
    operationMode=sim.simx_opmode_buffer
    )

    print(np.linalg.norm(detectedPoint))