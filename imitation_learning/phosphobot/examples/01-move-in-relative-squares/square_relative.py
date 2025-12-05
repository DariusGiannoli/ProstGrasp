import time
import requests

# Configurations
PI_IP: str = "127.0.0.1"
PI_PORT: int = 80
NUMBER_OF_SQUARES: int = 100


# Function to call the API
def call_to_api(endpoint: str, data: dict = {}):
    response = requests.post(f"http://{PI_IP}:{PI_PORT}/move/{endpoint}", json=data)
    return response.json()


# Example code to move the robot in a square of 4 cm x 4 cm
# 1 - Initialize the robot
call_to_api("init")
print("Initializing robot")
time.sleep(2)

# We move it to the top left corner of the square
call_to_api("relative", {"x": 0, "y": -3, "z": 3, "rx": 0, "ry": 0, "rz": 0, "open": 0})
print("Moving to top left corner")
time.sleep(0.2)

# With the move relative endpoint, we can move relative to its current position
# 2 - We make the robot follow a 3 cm x 3 cm square
for _ in range(NUMBER_OF_SQUARES):
    # Move to the top right corner
    call_to_api(
        "relative", {"x": 0, "y": 3, "z": 0, "rx": 0, "ry": 0, "rz": 0, "open": 0}
    )
    print("Moving to top right corner")
    time.sleep(0.2)

    # Move to the bottom right corner
    call_to_api(
        "relative", {"x": 0, "y": 0, "z": -3, "rx": 0, "ry": 0, "rz": 0, "open": 0}
    )
    print("Moving to bottom right corner")
    time.sleep(0.2)

    # Move to the bottom left corner
    call_to_api(
        "relative", {"x": 0, "y": -3, "z": 0, "rx": 0, "ry": 0, "rz": 0, "open": 0}
    )
    print("Moving to bottom left corner")
    time.sleep(0.2)

    # Move to the top left corner
    call_to_api(
        "relative", {"x": 0, "y": 0, "z": 3, "rx": 0, "ry": 0, "rz": 0, "open": 0}
    )
    print("Moving to top left corner")
    time.sleep(0.2)
