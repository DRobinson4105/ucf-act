import os

from ultralytics import YOLO
import torch

print(f"Downloading YOLOv11")

os.makedirs("models", exist_ok=True)
os.chdir("models")

print("Downloading YOLOv11")

model = YOLO("yolo11n.pt")

model.export(
    format="onnx",
    half=False,
    imgsz=640,
    dynamic=False,
    nms=False,
    opset=17,
    simplify=True,
)

if torch.cuda.is_available():
    model.export(
        format="engine",
        imgsz=640,
        half=True,
        nms=True,
        dynamic=False,
    )
