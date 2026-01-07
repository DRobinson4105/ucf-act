import os
import argparse
import time

import cv2
import numpy as np
import onnxruntime as ort
import torch

import _init_paths
from lib.utils import VideoCapture
from lib.models import model_generators

LABEL_MAP = np.array([0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], dtype=np.uint8)
PALETTE = np.array([
    [50, 209, 50], # drivable
    [255, 0, 0], # non-drivable
], dtype=np.uint8)

def preprocess(frame: np.typing.NDArray[np.uint8], resolution: tuple[int, int]) -> np.typing.NDArray[np.float32]:
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img = (img - mean) / std
    
    w, h = resolution
    inp = img.transpose(2, 0, 1).reshape(1, 3, h, w)

    return inp

def postprocess(frame: np.typing.NDArray[np.uint8], outputs: list[np.typing.NDArray[np.float32]]) -> np.typing.NDArray[np.uint8]:
    preds = np.squeeze(outputs.argmax(axis=1), axis=0)

    # preds = LABEL_MAP[preds]
    mask = PALETTE[preds]

    annotated = cv2.addWeighted(frame, 0.5, mask, 0.5, 0)
    annotated = annotated[..., ::-1]

    return annotated

@torch.inference_mode
def main(args):
    if torch.cuda.is_available():
        device = torch.device("cuda")
    elif torch.backends.mps.is_available():
        device = torch.device("mps")
    else:
        device = torch.device("cpu")

    # model_path = f"weights/{args.model}.onnx"
    # ort_session = ort.InferenceSession(model_path, providers=[
    #     "CUDAExecutionProvider", "CoreMLExecutionProvider", "CPUExecutionProvider"
    # ])
    # input_name = ort_session.get_inputs()[0].name
    # output_names = [o.name for o in ort_session.get_outputs()]
    if args.model in model_generators:
        model = model_generators[args.model](num_classes=2)
        checkpoint_path = f"weights/{args.model}{args.checkpoint_suffix}.pth"
        checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=True)
        model.load_state_dict(checkpoint, strict=True)
    else:
        raise RuntimeError(f"Unsupported model: {args.model}, options are {model_generators}")

    model.eval()
    model = model.to(device)

    w, h = args.resolution.lower().split('x')
    w, h = int(w), int(h)
    resolution = (w, h)

    if args.test:
        frame = cv2.imread(args.test)
        frame = cv2.resize(frame, resolution)

        inp = preprocess(frame, resolution)

        # outputs = ort_session.run(output_names, {input_name: inp})
        outputs = model(torch.from_numpy(inp).to(device)).cpu().numpy()
        annotated = postprocess(frame, outputs)

        cv2.imshow("Test", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cap = VideoCapture(args.camera_port, frame_shape=(h, w))

        try:
            last_frame = time.time()
            while True:
                ret, frame = cap.read()
                if not ret: continue

                print(f"FPS: {1 / (time.time() - last_frame)}")
                last_frame = time.time()

                inp = preprocess(frame, resolution)
                # outputs = ort_session.run(output_names, {input_name: inp})
                outputs = model(torch.from_numpy(inp).to(device)).cpu().numpy()
                annotated = postprocess(frame, outputs)

                cv2.imshow("window", annotated)

                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break

        finally:
            cap.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--camera-port", type=int, default=0)
    parser.add_argument("--model", type=str, default="ddrnet", choices=model_generators)
    parser.add_argument("--checkpoint-suffix", type=str, default="")
    parser.add_argument("--resolution", type=str, default="1280x720")
    parser.add_argument("--test", type=str, default="")

    args = parser.parse_args()

    main(args)
