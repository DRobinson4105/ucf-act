import argparse
import time

import cv2

import _init_paths
from lib.utils import VideoCapture

def main(args):
    w, h = args.resolution.lower().split('x')
    w, h = int(w), int(h)

    cap = VideoCapture(args.camera_port, frame_shape=(h, w))

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
    parser.add_argument("--resolution", type=str, default="1280x720")

    args = parser.parse_args()

    main(args)
