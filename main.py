import sim
import sys

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print("Connected to remote API server")
else:
    sys.exit("Connection unsuccessful")

sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_oneshot)

errorCode, motorLeft = sim.simxGetObjectHandle(
                            clientID=clientID, 
                            objectName='Pioneer_p3dx_leftMotor', 
                            operationMode=sim.simx_opmode_oneshot_wait
                        )

errorCode, motorRight = sim.simxGetObjectHandle(
                            clientID=clientID, 
                            objectName='Pioneer_p3dx_rightMotor', 
                            operationMode=sim.simx_opmode_oneshot_wait
                        )

errorCode = sim.simxSetJointTargetVelocity(
                clientID=clientID,
                jointHandle=motorLeft,
                targetVelocity=0.2,
                operationMode=sim.simx_opmode_oneshot
            )

errorCode = sim.simxSetJointTargetVelocity(
                clientID=clientID,
                jointHandle=motorRight,
                targetVelocity=0.4,
                operationMode=sim.simx_opmode_oneshot_wait
            )
