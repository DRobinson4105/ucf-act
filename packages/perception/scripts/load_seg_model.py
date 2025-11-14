import os
import argparse

import gdown
from mim.commands.download import download

parser = argparse.ArgumentParser()

parser.add_argument("--model", type=str, default="gcnet", choices=["gcnet", "ddrnet"])

args = parser.parse_args()

os.makedirs("models", exist_ok=True)

if args.model == "gcnet":
    print("Downloading GCNet")
    gdown.download(id='1KersBP95k3b0AELiYlQ1rk4PKUmN-ueu', output='models/gcnet-s_mIoU-76.9.pth')
elif args.model == "ddrnet":
    print("Downloading DDRNet")
    download(
        package="mmsegmentation",
        configs=["ddrnet_23-slim_in1k-pre_2xb6-120k_cityscapes-1024x1024"],
        dest_root="models"
    )

    config_file = "ddrnet_23-slim_in1k-pre_2xb6-120k_cityscapes-1024x1024.py"
    os.replace(
        os.path.join("models", config_file),
        os.path.join("configs", config_file)
    )
else:
    raise Exception("Unsupported model")