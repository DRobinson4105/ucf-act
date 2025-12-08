import os
import sys
import argparse
from collections import OrderedDict

import gdown
import torch
import onnx

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from utils.ddrnet import load_ddrnet_slim
from utils.gcnet import load_gcnet

def main(config):
    checkpoint_path = f"models/{config.model}.pth"
    output_path = f"models/{config.model}.onnx"
    print(f"Downloading {config.model}")
    
    os.makedirs("models", exist_ok=True)

    if config.model == "ddrnet":
        model = load_ddrnet_slim()

        checkpoint_id = "1d_K3Af5fKHYwxSo8HkxpnhiekhwovmiP"

        gdown.download(id=checkpoint_id, output=checkpoint_path)
        checkpoint = torch.load(checkpoint_path, map_location='cpu') 

        state_dict = OrderedDict()

        for k, v in checkpoint.items():
            if k.startswith("model") and not k.startswith("model.seghead_extra"):
                state_dict[k[6:]] = v

        torch.save(state_dict, checkpoint_path)

        model.load_state_dict(state_dict, strict=True)
        model.eval()
    elif config.model == "gcnet":
        model = load_gcnet()

        checkpoint_id = "1KersBP95k3b0AELiYlQ1rk4PKUmN-ueu"

        gdown.download(id=checkpoint_id, output=checkpoint_path)
        checkpoint = torch.load(checkpoint_path, map_location='cpu')

        state_dict = checkpoint["state_dict"]

        torch.save(state_dict, checkpoint_path)

        model.load_state_dict(state_dict, strict=True)
        model.eval()

    w, h = config.resolution.lower().split('x')
    w, h = int(w), int(h)

    example_inputs = torch.randn(1, 3, h, w)
    mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
    example_inputs = (example_inputs - mean) / std
    example_inputs = (example_inputs,)

    torch.onnx.export(
        model,
        example_inputs,
        output_path,
        export_params=True,
        input_names=["input"],
        output_names=["output"]
    )

    onnx_model = onnx.load(output_path)
    onnx.checker.check_model(onnx_model)

    print(f"Downloaded {config.model}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", type=str, default="ddrnet")
    parser.add_argument("--resolution", type=str, default="1280x720")

    config = parser.parse_args()

    main(config)
