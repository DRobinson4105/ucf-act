import os

from ultralytics import YOLO
import torch

if torch.cuda.is_available():
    device = "0"
elif torch.backends.mps.is_available():
    device = "mps"
else:
    device = "cpu"

print(f"Downloading YOLOv11 on device {device}")

os.makedirs("models", exist_ok=True)
os.chdir("models")

print("Downloading YOLOv11")

model = YOLO("yolo11n.pt")

model.export(
    format="onnx",
    half=False,
    device=device,
    imgsz=640,
    dynamic=False,
    nms=False,
    opset=17,
    simplify=True,
)

if device == "0":
    model.export(
        format="engine",
        device=device,
        imgsz=640,
        half=True,
        nms=True,
        dynamic=False,
    )
