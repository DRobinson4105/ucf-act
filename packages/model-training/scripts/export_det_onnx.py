import argparse

from ultralytics import YOLO

def main(args):

    model = YOLO(f"weights/{args.checkpoint}")

    w, h = args.resolution.lower().split('x')
    w, h = int(w), int(h)
    
    model.export(
        format="onnx",
        half=True,
        imgsz=(h, w),
        dynamic=False,
        nms=True,
        opset=17,
        simplify=True,
    )

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--checkpoint", type=str, default="yolo11n.pt")
    parser.add_argument("--resolution", type=str, default="1280x720")

    args = parser.parse_args()

    main(args)
