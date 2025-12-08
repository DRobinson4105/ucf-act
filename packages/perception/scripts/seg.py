import os
import sys
import argparse
import time

import cv2
import numpy as np
import onnxruntime as ort

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from utils.video_capture import VideoCapture

label_map = np.array([0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 3, 3, 3, 3], dtype=np.uint8)

palette = np.array([
    [50, 209, 50], # drivable
    [255, 0, 0], # non-drivable
    [244, 35, 232], # person
    [220, 220, 0], # vehicle
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
    preds = np.squeeze(outputs[0].argmax(axis=1), axis=0)
    preds = label_map[preds]

    mask = palette[preds]

    annotated = cv2.addWeighted(frame, 0.5, mask, 0.5, 0)

    return annotated

def main(config):
    model_path = f"models/{config.model}.onnx"
    ort_session = ort.InferenceSession(model_path, providers=[
        "CUDAExecutionProvider", "CoreMLExecutionProvider", "CPUExecutionProvider"
    ])
    input_name = ort_session.get_inputs()[0].name
    output_names = [o.name for o in ort_session.get_outputs()]
    w, h = config.resolution.lower().split('x')
    w, h = int(w), int(h)
    resolution = (w, h)

    if config.test:
        frame = cv2.imread(config.test)
        frame = cv2.resize(frame, resolution)

        inp = preprocess(frame, resolution)

        outputs = ort_session.run(output_names, {input_name: inp})
        annotated = postprocess(frame, outputs)

        cv2.imshow("Test", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cap = VideoCapture(config.camera_port, frame_shape=(h, w))

        try:
            last_frame = time.time()
            while True:
                ret, frame = cap.read()
                if not ret: continue

                print(f"FPS: {1 / (time.time() - last_frame)}")
                last_frame = time.time()

                inp = preprocess(frame, resolution)
                outputs = ort_session.run(output_names, {input_name: inp})
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
    parser.add_argument("--model", type=str, default="ddrnet")
    parser.add_argument("--resolution", type=str, default="1280x720")
    parser.add_argument("--test", type=str, default="")

    config = parser.parse_args()

    main(config)
