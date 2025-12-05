import cv2
import sys
import time
import signal
import requests
import mediapipe as mp  # type: ignore

# Configurations
PI_IP: str = "127.0.0.1"
PI_PORT: int = 80

# Initialize MediaPipe Hand tracking
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7
)


# Handle Ctrl+C to exit the program gracefully
def signal_handler(sig, frame):
    print("\nExiting gracefully...")
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# Function to call the API
def call_to_api(endpoint: str, data: dict = {}):
    response = requests.post(f"http://{PI_IP}:{PI_PORT}/move/{endpoint}", json=data)
    return response.json()


def wave_motion():
    points = 5
    for _ in range(2):
        for i in range(points):
            call_to_api(
                "absolute",
                {
                    "x": 0,
                    "y": 2 * (-1) ** i,
                    "z": 0,
                    "rx": -90,
                    "ry": 0,
                    "rz": 0,
                    "open": i % 2 == 0,
                },
            )
            time.sleep(0.2)


call_to_api("init")
cap = cv2.VideoCapture(0)
last_wave_time: float = 0
WAVE_COOLDOWN = 3

try:
    while True:
        success, image = cap.read()
        if not success:
            continue

        results = hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        current_time = time.time()
        if (
            results.multi_hand_landmarks
            and current_time - last_wave_time > WAVE_COOLDOWN
        ):
            wave_motion()
            last_wave_time = current_time

        cv2.imshow("Hand Detection", image)
        cv2.waitKey(1)

except KeyboardInterrupt:
    print("\nExiting gracefully...")
finally:
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
