import argparse
import time

import torch
if torch.cuda.is_available():
    torch.cuda.init()

import cv2
from ultralytics import YOLO

import _init_paths
from lib.utils import VideoCapture

def main(args):
    if torch.cuda.is_available():
        device = "cuda"
    elif torch.backends.mps.is_available():
        device = "mps"
    else:
        device = "cpu"

    model = YOLO(f"weights/{args.checkpoint}").to(device)

    w, h = args.resolution.lower().split('x')
    w, h = int(w), int(h)
    resolution = (w, h)
    
    if args.test:
        frame = cv2.imread(args.test)
        frame = cv2.resize(frame, resolution)

        results = model(frame, verbose=False)
        annotated = results[0].plot()

        cv2.imshow("Test", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cap = VideoCapture(args.camera_port, frame_shape=(h, w))
        last_time = time.time()

        try:
            while True:
                ret, frame = cap.read()
                if not ret: break

                print(f"FPS: {1 / (time.time() - last_time)}")
                last_time = time.time()

                results = model(frame, verbose=False)
                annotated = results[0].plot()

                cv2.imshow("window", annotated)

                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break

        finally:
            cap.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--checkpoint", type=str, default="yolo11n.pt")
    parser.add_argument("--camera-port", type=int, default=0)
    parser.add_argument("--resolution", type=str, default="1280x720")
    parser.add_argument("--test", type=str, default="")

    args = parser.parse_args()

    main(args)
