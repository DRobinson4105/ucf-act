import os
import argparse
from collections import OrderedDict

import gdown
import torch
import onnx

import _init_paths
from lib.models import load_ddrnet_slim, load_gcnet

MODEL_OPTIONS = ["ddrnet_slim", "gcnet"]

def main(args):
    checkpoint_path = f"weights/{args.model}.pth"
    output_path = f"weights/{args.model}.onnx"
    print(f"Downloading {args.model}")
    
    os.makedirs("weights", exist_ok=True)

    if args.model == "ddrnet_slim":
        model = load_ddrnet_slim()

        checkpoint_id = "1d_K3Af5fKHYwxSo8HkxpnhiekhwovmiP"

        gdown.download(id=checkpoint_id, output=checkpoint_path)
        checkpoint = torch.load(checkpoint_path, map_location='cpu') 

        state_dict = OrderedDict()

        for k, v in checkpoint.items():
            if k.startswith("model") and (args.with_aux_head or not k.startswith("model.seghead_extra")):
                state_dict[k[6:]] = v

        torch.save(state_dict, checkpoint_path)

        model.load_state_dict(state_dict, strict=True)
        model.eval()
    elif args.model == "gcnet":
        model = load_gcnet()

        checkpoint_id = "1KersBP95k3b0AELiYlQ1rk4PKUmN-ueu"

        gdown.download(id=checkpoint_id, output=checkpoint_path)
        checkpoint = torch.load(checkpoint_path, map_location='cpu')

        state_dict = checkpoint["state_dict"]

        torch.save(state_dict, checkpoint_path)

        model.load_state_dict(state_dict, strict=True)
        model.eval()
    else:
        raise RuntimeError(f"Unsupported model: {args.model}, options are {MODEL_OPTIONS}")

    w, h = args.resolution.lower().split('x')
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

    print(f"Downloaded {args.model}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", type=str, default="ddrnet_slim", choices=["ddrnet_slim", "gcnet"])
    parser.add_argument("--resolution", type=str, default="1280x720")
    parser.add_argument("--with-aux-head", action="store_true")

    args = parser.parse_args()

    main(args)
