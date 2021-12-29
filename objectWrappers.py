import numpy as np

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
    
    def updateValues(self, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector) -> None:
        self.detectionState = detectionState
        self.detectedPoint = detectedPoint
        self.distance = np.linalg.norm(detectedPoint)
        self.detectedObjectHandle = detectedObjectHandle
        self.detectedSurfaceNormalVector = detectedSurfaceNormalVector