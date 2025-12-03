import argparse

from ultralytics import YOLO
import cv2
import torch

def main(config):
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = YOLO("models/yolo11n.pt").to(device)

    camera_port = int(config.camera_port)
    cap = cv2.VideoCapture(camera_port)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

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

    config = parser.parse_args()

    main(config)