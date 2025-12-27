import argparse
import shutil
import os.path as osp

import numpy as np
import torch
from torch.backends import cudnn
import yaml

if torch.cuda.is_available():
    torch.cuda.init()

from ultralytics import YOLO

def main(args):
    with open(args.config, 'r') as fp:
        config = yaml.safe_load(fp)

    train_config = config["train"]

    torch.manual_seed(config["seed"])
    np.random.seed(config["seed"])

    if torch.cuda.is_available():
        device = torch.device("cuda")
    elif torch.backends.mps.is_available():
        device = torch.device("mps")
    else:
        device = torch.device("cpu")

    print(f"Running on {device} device")

    cudnn.benchmark = config["cudnn"]["benchmark"]
    cudnn.deterministic = config["cudnn"]["deterministic"]
    cudnn.enabled = config["cudnn"]["enabled"]

    if "load_path" in train_config:
        model = YOLO(train_config["load_path"])
    else:
        model = YOLO(f"{config['model']}.pt")

    model.train(**train_config["ultralytics_parameters"], device=device, seed=config["seed"])

    if "save_path" in train_config:
        if "project" in train_config["ultralytics_parameters"] and "name" in train_config["ultralytics_parameters"]:
            shutil.copy(osp.join(train_config["ultralytics_parameters"]["project"], train_config["ultralytics_parameters"]["name"], "weights", "best.pt"), train_config["save_path"])
        else:
            raise RuntimeError("train/ultralytics_parameters/project and train/ultralytics_parameters/name must be defined in the config file to save model")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("--config", type=str, default="config/yolo.yaml")

    args = parser.parse_args()

    main(args)
