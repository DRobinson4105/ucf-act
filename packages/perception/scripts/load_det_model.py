import os

from ultralytics import YOLO

os.makedirs("models", exist_ok=True)
os.chdir("models")

model = YOLO("yolo11n.pt")

model.export(
    format="onnx",
    half=False,
    device=0,
    imgsz=640,
    dynamic=False,
    nms=False,
    opset=17,
    simplify=True,
)

model.export(
    format="engine",
    device=0,
    imgsz=640,
    half=True,
    nms=True,
    dynamic=False,
)