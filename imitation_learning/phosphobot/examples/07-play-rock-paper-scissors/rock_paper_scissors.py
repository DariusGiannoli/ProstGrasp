import cv2
import time
import random
import requests
import numpy as np
import mediapipe as mp  # type: ignore

# Robot API Configuration
PI_IP = "127.0.0.1"
PI_PORT = 80


class RockPaperScissorsGame:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
        )
        self.cap = cv2.VideoCapture(0)
        self.gestures = {
            "rock": self.make_rock_gesture,
            "paper": self.make_paper_gesture,
            "scissors": self.make_scissors_gesture,
        }

    def call_to_api(self, endpoint: str, data: dict = {}):
        response = requests.post(f"http://{PI_IP}:{PI_PORT}/move/{endpoint}", json=data)
        return response.json()

    def detect_gesture(self, hand_landmarks):
        # Get relevant finger landmarks
        thumb_tip = hand_landmarks.landmark[4]
        index_tip = hand_landmarks.landmark[8]
        middle_tip = hand_landmarks.landmark[12]
        ring_tip = hand_landmarks.landmark[16]
        pinky_tip = hand_landmarks.landmark[20]

        # Get wrist position for reference
        wrist = hand_landmarks.landmark[0]

        # Calculate distances from wrist
        fingers_extended = []
        for tip in [thumb_tip, index_tip, middle_tip, ring_tip, pinky_tip]:
            distance = np.sqrt((tip.x - wrist.x) ** 2 + (tip.y - wrist.y) ** 2)
            fingers_extended.append(distance > 0.2)  # Threshold for extended fingers

        # Determine gesture
        if not any(fingers_extended[1:]):  # All fingers closed
            return "rock"
        elif all(fingers_extended):  # All fingers open
            return "paper"
        elif (
            fingers_extended[1]
            and fingers_extended[2]
            and not fingers_extended[3]
            and not fingers_extended[4]
        ):  # Only index and middle extended
            return "scissors"
        return None

    def make_rock_gesture(self):
        # Move to closed fist position
        self.call_to_api(
            "absolute",
            {"x": 0, "y": 0, "z": 5, "rx": 0, "ry": 0, "rz": 0, "open": 0},
        )

    def make_paper_gesture(self):
        # Move to open hand position
        self.call_to_api(
            "absolute",
            {"x": 0, "y": 0, "z": 5, "rx": 0, "ry": 0, "rz": 0, "open": 1},
        )

    def make_scissors_gesture(self):
        # Move to scissors position
        self.call_to_api(
            "absolute",
            {"x": 0, "y": 0, "z": 5, "rx": 0, "ry": -45, "rz": 0, "open": 0.5},
        )

    def move_up_down(self, times=3):
        for step in range(times + 1):
            self.call_to_api(
                "absolute",
                {"x": 0, "y": 0, "z": 4, "rx": 0, "ry": 0, "rz": 0, "open": 0},
            )
            time.sleep(0.25)
            self.call_to_api(
                "absolute",
                {"x": 0, "y": 0, "z": -4, "rx": 0, "ry": 0, "rz": 0, "open": 0},
            )
            time.sleep(0.25)
            print(times - step)

    def determine_winner(self, player_gesture, robot_gesture):
        if player_gesture == robot_gesture:
            return "Tie!"
        winners = {"rock": "scissors", "paper": "rock", "scissors": "paper"}
        return (
            "Player wins!"
            if winners[player_gesture] == robot_gesture
            else "Robot wins!"
        )

    def play_game(self):
        print("Initializing robot...")
        self.call_to_api("init")
        time.sleep(1)

        print("Robot performing countdown...")
        self.move_up_down(times=3)

        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture image.")
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            player_gesture = self.detect_gesture(results.multi_hand_landmarks[0])

            if player_gesture:
                robot_gesture = random.choice(["rock", "paper", "scissors"])
                print(f"\nPlayer chose: {player_gesture}")
                print(f"Robot chose: {robot_gesture}")

                self.gestures[robot_gesture]()  # Robot makes its gesture
                result = self.determine_winner(player_gesture, robot_gesture)
                print(result)
                time.sleep(2)
            else:
                print("Gesture not detected. Please try again.")
        else:
            print("No hand detected. Please try again.")

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    game = RockPaperScissorsGame()
    game.play_game()
