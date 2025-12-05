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

    count = 0
    os.makedirs(config.out_dir, exist_ok=True)

    try:
        last_frame = time.time()
        while True:
            if config.fps != 0:
                while 1 / (time.time() - last_frame) < config.fps:
                    continue

            last_frame = time.time()

            ret, frame = cap.read()
            if not ret: continue

            path = f"{config.out_dir}/{count:06}.png"
            cv2.imwrite(path, frame)
            print(f"Saved to {path}")
            count += 1

            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--camera-port", type=int, default=0)
    parser.add_argument("--out-dir", type=str)
    parser.add_argument("--fps", type=int, default=0)

    config = parser.parse_args()

    main(config)
