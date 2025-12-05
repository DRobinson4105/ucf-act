import os
import sys
import argparse
import time

import cv2

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from utils.video_capture import VideoCapture

def main(config):
    cap = VideoCapture(config.camera_port, frame_shape=(720, 1280))

    try:
        last_frame = time.time()
        while True:
            ret, frame = cap.read()
            if not ret: continue

            print(f"FPS: {1 / (time.time() - last_frame)}")
            last_frame = time.time()

            cv2.imshow("window", frame)

            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--camera-port", type=int, default=0)

    config = parser.parse_args()

    main(config)
