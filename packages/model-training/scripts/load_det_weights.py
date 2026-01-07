import os
import argparse

import torch
if torch.cuda.is_available():
    torch.cuda.init()

from ultralytics import YOLO

def main(args):
    print(f"Downloading YOLOv11")

    w, h = args.resolution.lower().split('x')
    w, h = int(w), int(h)

    os.makedirs("weights", exist_ok=True)
    os.chdir("weights")

    model = YOLO("yolo11n.pt")

    model.export(
        format="onnx",
        half=False,
        imgsz=(h, w),
        dynamic=False,
        nms=False,
        opset=17,
        simplify=True,
    )

    if torch.cuda.is_available():
        model.export(
            format="engine",
            imgsz=(h, w),
            half=True,
            nms=True,
            dynamic=False,
        )

    print(f"Downloaded YOLOv11")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--resolution", type=str, default="1280x720")

    args = parser.parse_args()

    main(args)
