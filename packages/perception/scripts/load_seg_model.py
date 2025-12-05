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

from utils.ddrnet import load_ddrnet, load_ddrnet_slim

def main(config):
    os.makedirs("models", exist_ok=True)

    if config.model == "ddrnet_slim":
        model = load_ddrnet_slim()
        checkpoint_id = "1d_K3Af5fKHYwxSo8HkxpnhiekhwovmiP"
    elif config.model == "ddrnet":
        model = load_ddrnet()
        checkpoint_id = "16viDZhbmuc3y7OSsUo2vhA7V6kYO0KX6"
    else:
        raise RuntimeError(f"Unsupported model {config.model}")
    
    checkpoint_path = config.output_path.replace(".onnx", ".pth")

    print(f"Downloading {config.model}")

    gdown.download(id=checkpoint_id, output=checkpoint_path)
    checkpoint = torch.load(checkpoint_path, map_location='cpu') 

    state_dict = OrderedDict()

    for k, v in checkpoint.items():
        if k.startswith("model") and not k.startswith("model.seghead_extra"):
            state_dict[k[6:]] = v

    torch.save(state_dict, checkpoint_path)

    model.load_state_dict(state_dict, strict=True)
    model.eval()

    example_inputs = torch.randn(1, 3, 720, 1280)
    mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
    example_inputs = (example_inputs - mean) / std
    example_inputs = (example_inputs,)

    torch.onnx.export(
        model,
        example_inputs,
        config.output_path,
        export_params=True,
        input_names=["input"],
        output_names=["output"]
    )

    onnx_model = onnx.load(config.output_path)
    onnx.checker.check_model(onnx_model)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", type=str, choices=["ddrnet", "ddrnet_slim"], default="ddrnet_slim")
    parser.add_argument("--output-path", type=str, default="models/ddrnet_slim.onnx")

    config = parser.parse_args()

    main(config)
