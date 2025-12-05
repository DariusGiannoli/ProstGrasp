import time
import sys
import pybullet as p
import pybullet_data

print(
    f"Starting PyBullet simulation in GUI mode. Python version: {sys.version} (recommended: Python 3.8)"
)

# Connect to PyBullet
p.connect(p.GUI_SERVER)
time.sleep(1)  # Wait for the GUI to initialize

p.resetDebugVisualizerCamera(
    cameraDistance=0.5,  # Distance of the camera from the target.
    cameraYaw=150,  # Rotation around the target (left/right).
    cameraPitch=-30,  # Rotation around the target (up/down).
    cameraTargetPosition=[0, 0, 0],  # The target position the camera looks at.
)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)


# p.setRealTimeSimulation(1)


while True:
    p.stepSimulation()
    # time.sleep(1.0 / 240.0)
    # time.sleep(1e-9)
