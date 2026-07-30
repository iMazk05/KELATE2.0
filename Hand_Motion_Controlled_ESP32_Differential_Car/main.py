import argparse
import json
import math
import time

import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt

# ------------------------- CONFIG -------------------------
MQTT_TOPIC = "robot/cmd"
PUBLISH_HZ = 20 
SMOOTHING = 0.35 
DEADZONE_STEER = 0.08
DEADZONE_THROTTLE = 0.08
# -----------------------------------------------------------


class ExponentialSmoother:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * self.value + (1 - self.alpha) * new_value
        return self.value


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def apply_deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def compute_rotation(hand_landmarks):
    wrist = hand_landmarks.landmark[0]
    mid_mcp = hand_landmarks.landmark[9]

    dx = mid_mcp.x - wrist.x
    dy = mid_mcp.y - wrist.y

    angle = math.degrees(math.atan2(dx, -dy))
    return clamp(angle / 90.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--broker", default="localhost",
                         help="MQTT broker IP (default: localhost, since the broker runs on this same laptop)")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--camera", type=int, default=0)
    args = parser.parse_args()

    client = mqtt.Client(client_id="gesture_controller", protocol=mqtt.MQTTv311)
    client.connect(args.broker, args.port, keepalive=30)
    client.loop_start()

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.5,
    )

    cap = cv2.VideoCapture(args.camera)

    throttle_smoother = ExponentialSmoother(SMOOTHING)
    steer_smoother = ExponentialSmoother(SMOOTHING)

    last_publish = 0.0
    publish_interval = 1.0 / PUBLISH_HZ

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)

            throttle_raw = 0.0
            steer_raw = 0.0

            if results.multi_hand_landmarks and results.multi_handedness:
                for lm, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    label = handedness.classification[0].label
                    score = handedness.classification[0].score
                    mp_draw.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)

                    wrist_px = (int(lm.landmark[0].x * frame.shape[1]),
                                int(lm.landmark[0].y * frame.shape[0]))
                    cv2.putText(frame, f"{label} ({score:.2f})", wrist_px,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

                    if label == "Left":
                        throttle_raw = compute_rotation(lm)
                    elif label == "Right":
                        steer_raw = compute_rotation(lm)

            throttle = throttle_smoother.update(throttle_raw)
            steer = steer_smoother.update(steer_raw)

            throttle = apply_deadzone(throttle, DEADZONE_THROTTLE)
            steer = apply_deadzone(steer, DEADZONE_STEER)

            now = time.time()
            if now - last_publish >= publish_interval:
                payload = json.dumps({"throttle": round(throttle, 3), "steering": round(steer, 3)})
                client.publish(MQTT_TOPIC, payload, qos=0)
                last_publish = now

            cv2.putText(frame, f"Throttle: {throttle:+.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"Steering: {steer:+.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Gesture Control", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        client.publish(MQTT_TOPIC, json.dumps({"throttle": 0.0, "steering": 0.0}), qos=0)
        cap.release()
        cv2.destroyAllWindows()
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()