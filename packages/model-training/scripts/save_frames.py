import os
import argparse
import time

import cv2

import _init_paths
from lib.utils import VideoCapture

def main(args):
    w, h = args.resolution.lower().split('x')
    w, h = int(w), int(h)

    cap = VideoCapture(args.camera_port, frame_shape=(h, w))

    count = 0
    os.makedirs(args.out_dir, exist_ok=True)

    try:
        last_frame = time.time()
        while True:
            if args.fps != 0:
                while (time.time() - last_frame) < (1.0 / args.fps):
                    continue

            last_frame = time.time()

            ret, frame = cap.read()
            if not ret: continue

            path = f"{args.out_dir}/{count:06}.png"
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
    parser.add_argument("--resolution", type=str, default="1280x720")

    args = parser.parse_args()

    main(args)
