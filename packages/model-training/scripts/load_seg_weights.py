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

MODEL_OPTIONS=["ddrnet", "gcnet"]

def collapse_head(old_conv):
    new_conv = nn.Conv2d(
        in_channels=old_conv.in_channels,
        out_channels=2,
        kernel_size=old_conv.kernel_size,
        stride=old_conv.stride,
        padding=old_conv.padding,
        dilation=old_conv.dilation,
        groups=old_conv.groups,
        bias=(old_conv.bias is not None),
        padding_mode=old_conv.padding_mode,
    )

    with torch.no_grad():
        new_conv.weight[0] = old_conv.weight[0] + old_conv.weight[1]
        old_conv.weight[1] = old_conv.weight[2:].sum(dim=0)

        if old_conv.bias is not None:
            new_conv.bias[0] = old_conv.bias[0] + old_conv.bias[1]
            new_conv.bias[1] = old_conv.bias[2:].sum()

    return new_conv

def main(args):
    with open(args.config, 'r') as fp:
        config = yaml.safe_load(fp)

    torch.manual_seed(config["seed"])

    checkpoint_path = f"weights/{config['model']}.pth"
    output_path = f"weights/{config['model']}.onnx"
    print(f"Downloading {config['model']}")
    
    os.makedirs("weights", exist_ok=True)

    model = model_generators[config["model"]](num_classes=19)

    if config["model"] == "ddrnet":
        checkpoint_id = "1d_K3Af5fKHYwxSo8HkxpnhiekhwovmiP"

        gdown.download(id=checkpoint_id, output=checkpoint_path)
        checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=True) 

        state_dict = OrderedDict()

        for k, v in checkpoint.items():
            if k.startswith("model"):
                state_dict[k[6:]] = v

        model.load_state_dict(state_dict, strict=True)

        model.final_layer.conv2 = collapse_head(model.final_layer.conv2)
        model.seghead_extra.conv2 = collapse_head(model.seghead_extra.conv2)

        torch.save(model.state_dict(), checkpoint_path)
        model.eval()
    elif config["model"] == "gcnet":
        checkpoint_id = "1KersBP95k3b0AELiYlQ1rk4PKUmN-ueu"

        gdown.download(id=checkpoint_id, output=checkpoint_path)
        checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=True)

        state_dict = checkpoint["state_dict"]

        model.load_state_dict(state_dict, strict=True)

        model.decode_head.conv_seg = collapse_head(model.decode_head.conv_seg)
        model.decode_head.aux_cls_seg_c4 = collapse_head(model.decode_head.aux_cls_seg_c4)
        
        torch.save(model.state_dict(), checkpoint_path)
        model.eval()
    else:
        raise RuntimeError(f"Unsupported model: {config['model']}, options are {MODEL_OPTIONS}")

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
    onnx.checker.check_model(onnx_model)

    print(f"Downloaded {config['model']}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--config", type=str, default="config/ddrnet.yaml")

    args = parser.parse_args()

    main(args)
