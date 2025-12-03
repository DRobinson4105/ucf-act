import argparse

from mmseg.apis import MMSegInferencer
import cv2
import torch
import numpy as np

def main(config):
    if config.model == "gcnet":
        infer = MMSegInferencer(
            model='configs/gcnet-s_4xb3-120k_cityscapes-1024x1024.py',
            weights='models/gcnet-s_mIoU-76.9.pth',
            device='cuda:0'
        )
    elif config.model == "ddrnet":
        infer = MMSegInferencer(
            model='configs/ddrnet.py',
            weights='models/ddrnet_23-slim_in1k-pre_2xb6-120k_cityscapes-1024x1024_20230426_145312-6a5e5174.pth',
            device='cuda:0'
        )
    else:
        raise Exception("Unsupported model")

    # warm up
    sample = cv2.cvtColor(cv2.imread('examples/image.png'), cv2.COLOR_BGR2RGB)

    for _ in range(5):
        with torch.inference_mode(), torch.amp.autocast('cuda'):
            _ = infer(sample, return_datasamples=False)

    camera_port = int(config.camera_port)
    cap = cv2.VideoCapture(camera_port, cv2.CAP_DSHOW)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            res = infer(frame, return_datasamples=False)

            pred = res["predictions"]

            palette = np.array(infer.visualizer.dataset_meta["palette"], dtype=np.uint8)
            color_mask = palette[pred]

            annotated = cv2.addWeighted(frame, 0.5, color_mask[..., ::-1], 0.5, 0)

            cv2.imshow("window", annotated)

            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", type=str, default="gcnet", choices=["gcnet", "ddrnet"])
    parser.add_argument("--camera-port", type=int, default=0)

    config = parser.parse_args()

    main(config)
