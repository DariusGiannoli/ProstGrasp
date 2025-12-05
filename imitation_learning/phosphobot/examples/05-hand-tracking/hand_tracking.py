import cv2
import requests  # type: ignore
import numpy as np
import mediapipe as mp  # type: ignore

# TODO: estimate depth using the stero camera


class HandTracker:
    def __init__(self):
        # MediaPipe hand tracking setup
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
        )
        self.left_hand_z = 0

    def calculate_hand_open_state(self, hand_landmarks):
        """Calculate hand open state based on thumb and finger tip distances."""
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

        openess = 1 - max(
            0,
            min(
                1,
                1
                - np.sqrt(
                    (thumb_tip.x - index_tip.x) ** 2
                    + (thumb_tip.y - index_tip.y) ** 2
                    + (thumb_tip.z - index_tip.z) ** 2
                )
                * 3,
            ),
        )

        return 0 if openess < 0.25 else openess

    def add_hand_data_overlay(self, image, x, y, z, open):
        """Add overlay with hand tracking data."""
        # Prepare text
        overlay_text = f"X: {x:.3f} " f"Y: {y:.3f} " f"Z: {z:.3f} " f"Open: {open:.2f}"

        # Add background rectangle
        cv2.rectangle(image, (10, 30), (550, 70), (255, 255, 255), -1)

        # Add text
        cv2.putText(
            image, overlay_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2
        )

        return image

    def track_hand(self):
        """Open video capture and continuously track hand position."""
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                continue

            # Flip the image horizontally
            image = cv2.flip(image, 1)

            # Convert the BGR image to RGB
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Process the image and find hands
            results = self.hands.process(image_rgb)

            # Reset left hand z
            self.left_hand_z = 0

            # Draw hand landmarks and send position
            if results.multi_hand_landmarks:
                hand_data = {"rx": 0, "ry": 0, "rz": 0}

                for hand_index, hand_landmarks in enumerate(
                    results.multi_hand_landmarks
                ):
                    # Draw landmarks on the image
                    self.mp_drawing.draw_landmarks(
                        image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                    )

                    # Extract hand position (using wrist as reference)
                    wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

                    # Identify hand side (uses handedness if available)
                    if results.multi_handedness:
                        handedness = results.multi_handedness[hand_index]
                        if handedness.classification[0].label == "Left":
                            hand_data["x"] = -wrist.y + 0.65
                        else:
                            hand_data["y"] = 0 - wrist.x + 0.525
                            hand_data["z"] = 0 - wrist.y + 0.65
                            hand_data["open"] = self.calculate_hand_open_state(
                                hand_landmarks
                            )

                # Send data to endpoint
                if (
                    "x" in hand_data
                    and "y" in hand_data
                    and "z" in hand_data
                    and "open" in hand_data
                ):
                    try:
                        self.add_hand_data_overlay(
                            image,
                            hand_data["x"],
                            hand_data["y"],
                            hand_data["z"],
                            hand_data["open"],
                        )

                        requests.post(
                            "http://localhost:80/move/absolute",
                            json={
                                **hand_data,
                                "x": hand_data["x"] * 100,
                                "y": hand_data["y"] * 100,
                                "z": hand_data["z"] * 100,
                            },
                            timeout=0.2,  # Short timeout to prevent blocking
                        )
                    except requests.RequestException as e:
                        print(f"Failed to send data: {e}")
                else:
                    print("Missing hand data")

            # Display the image
            cv2.imshow("Hand Tracking", image)

            # Break loop on 'q' key press
            if cv2.waitKey(5) & 0xFF == ord("q"):
                break

        # Clean up
        cap.release()
        cv2.destroyAllWindows()


def main():
    # Initialize hand tracker
    try:
        requests.post("http://localhost:80/move/init")
    except requests.RequestException as e:
        print(f"Failed to connect to server: {e}")

    tracker = HandTracker()
    tracker.track_hand()


if __name__ == "__main__":
    main()
