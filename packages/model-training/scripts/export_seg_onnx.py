import os
import argparse
from collections import OrderedDict

import gdown
import torch
import torch.nn as nn
import onnx
import yaml

import _init_paths
from lib.models import model_generators

def main(args):
    with open(args.config, 'r') as fp:
        config = yaml.safe_load(fp)

    torch.manual_seed(config["seed"])

    checkpoint_path = f"weights/{args.checkpoint}.pth"
    output_path = f"weights/{args.checkpoint}.onnx"
    print(f"Loading {checkpoint_path}")
    
    os.makedirs("weights", exist_ok=True)

    model = model_generators[config["model"]](num_classes=2)

    state_dict = torch.load(checkpoint_path, map_location="cpu", weights_only=True)

    model.load_state_dict(state_dict, strict=True)

    model.eval()

    w, h = config["resolution"].lower().split('x')
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
    onnx.checker.check_model(onnx_model, full_check=True)

    print(f"Exported to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--config", type=str, default="config/ddrnet.yaml")
    parser.add_argument("--checkpoint", type=str, default="ddrnet")

    args = parser.parse_args()

    main(args)
