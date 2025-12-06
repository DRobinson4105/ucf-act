import os
import sys
import argparse

import cv2
import torch
torch.cuda.init()

from ultralytics import YOLO

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from utils.video_capture import VideoCapture

def main(config):
    if torch.cuda.is_available():
        device = "cuda"
    elif torch.backends.mps.is_available():
        device = "mps"
    else:
        device = "cpu"

    model = YOLO("models/yolo11n.pt").to(device)

    w, h = config.resolution.lower().split('x')
    w, h = int(w), int(h)
    resolution = (w, h)
    
    if config.test:
        frame = cv2.imread(config.test)
        frame = cv2.resize(frame, resolution)

        results = model(frame, verbose=False)
        annotated = results[0].plot()

        cv2.imshow("Test", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cap = VideoCapture(config.camera_port, frame_shape=(h, w))
    
        try:
            while True:
                ret, frame = cap.read()
                if not ret: break

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

    parser.add_argument("--camera-port", type=int, default=0)
    parser.add_argument("--resolution", type=str, default="1280x720")
    parser.add_argument("--test", type=str, default="")

    config = parser.parse_args()

    main(config)
