import os
import argparse
import shutil

import torch
torch.cuda.init()

from ultralytics import YOLO

def main(config):
    print(f"Downloading YOLOv11")

    w, h = config.resolution.lower().split('x')
    w, h = int(w), int(h)

    os.makedirs("models", exist_ok=True)
    os.chdir("models")

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

    if config.output_path != "models/yolo11n.pt":
        os.chdir("..")
        shutil.move()

    print(f"Downloaded YOLOv11")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--output-path", type=str, default="models/yolo11n.pt")
    parser.add_argument("--resolution", type=str, default="1280x720")

    config = parser.parse_args()

    main(config)
