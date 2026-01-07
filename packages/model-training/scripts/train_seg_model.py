import time
import argparse
import copy

from tqdm import tqdm
import numpy as np
import torch
from torch.backends import cudnn
from torch.utils.data import DataLoader
import yaml

import _init_paths
from lib.datasets import dataset_generators
from lib.models import model_generators
from lib.core import (
    criterion_generators,
    optimizer_generators,
    lr_scheduler_generators,
    get_confusion_matrix
)

def main(args):
    with open(args.config, 'r') as fp:
        config = yaml.safe_load(fp)

    torch.manual_seed(config["seed"])
    np.random.seed(config["seed"])

    if args.stage in config["train_stages"]:
        train_config = config["train"] | config["train_stages"][args.stage]
        val_config = config["val"] | config["train_stages"][args.stage]
    else:
        raise RuntimeError(f"{args.stage} is not a valid train stage in {args.config}")

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

    model_name = config["model"]
    num_classes = config["num_classes"]

    if model_name in model_generators:
        model = model_generators[model_name](num_classes=num_classes)
    else:
        raise RuntimeError(f"Unsupported model: {model_name}, options are {model_generators}")
    
    if "load_path" in train_config and train_config["load_path"] is not None:
        checkpoint = torch.load(train_config["load_path"], map_location='cpu', weights_only=True)
        model.load_state_dict(checkpoint, strict=True)

    model = model.to(device)
    
    dataset_name = train_config["dataset"]

    if dataset_name in dataset_generators:
        train_dataset = dataset_generators[dataset_name](
            **train_config["dataset_parameters"],
            **train_config["train_dataset_parameters"]
        )
        val_dataset = dataset_generators[dataset_name](
            **val_config["dataset_parameters"],
            **val_config["val_dataset_parameters"]
        )
    else:
        raise RuntimeError(f"Unsupported dataset: {dataset_name}, options are {dataset_generators}")

    train_loader = DataLoader(
        train_dataset,
        batch_size=train_config["batch_size"],
        shuffle=True,
        drop_last=True
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=val_config["batch_size"],
        shuffle=False,
        drop_last=False
    )

    criterion_name = train_config["criterion"]
    
    if criterion_name in criterion_generators:
        criterion = criterion_generators[criterion_name](
            ignore_label=train_config["ignore_label"],
            **train_config["criterion_parameters"]
        )
    else:
        raise RuntimeError(f"Unsupported criterion: {criterion_name}, options are {criterion_generators}")
    
    criterion = criterion.to(device)
    
    optimizer_name = train_config["optimizer"]

    if optimizer_name in optimizer_generators:
        optimizer = optimizer_generators[optimizer_name](
            model.parameters(),
            **train_config["optimizer_parameters"]
        )
    else:
        raise RuntimeError(f"Unsupported optimizer: {optimizer_name}, options are {optimizer_generators}")
    
    if "lr_scheduler" in train_config:
        lr_scheduler_name = train_config["lr_scheduler"]

        if lr_scheduler_name in lr_scheduler_generators:
            lr_scheduler = lr_scheduler_generators[lr_scheduler_name](
                optimizer,
                max_iter=len(train_loader) * train_config["num_epochs"]
            )
        else:
            raise RuntimeError(f"Unsupported lr scheduler: {lr_scheduler_name}, options are {lr_scheduler_generators}")
    else:
        lr_scheduler = None

    best_epoch = -1
    best_miou = 0
    best_state_dict = None

    for epoch in range(train_config["num_epochs"]):
        start = time.time()
        model.train()

        losses = []

        for image, mask in tqdm(train_loader, leave=False):
            image, mask = image.to(device), mask.to(device)

            optimizer.zero_grad()

            logits = model(image)

            loss = criterion(logits, mask)

            loss.backward()

            optimizer.step()

            if lr_scheduler is not None:
                lr_scheduler.step()

            losses.append(loss.item())

        train_loss = float(np.mean(losses))

        end = time.time()
        elapsed = end - start
        hours = int(elapsed / 3600)
        minutes = int(elapsed / 60 % 60)
        seconds = int(elapsed % 60)
        timestamp = f"{hours}:{minutes:02d}:{seconds:02d}" if hours > 0 else f"{minutes:02d}:{seconds:02d}"

        print(f"Epoch {epoch+1} ({timestamp}) -> Loss: {train_loss}")
        
        if epoch % train_config["val_interval_epochs"] == 0:
            start = time.time()
            confusion_matrix = np.zeros((num_classes, num_classes))
            losses = []

            model.eval()

            with torch.no_grad():
                for image, mask in tqdm(val_loader, leave=False):
                    size = mask.size()
                    image, mask = image.to(device), mask.to(device)

                    logits = model(image)

                    loss = criterion(logits, mask)

                    losses.append(loss.item())
                    pred = logits.argmax(dim=1)
                    
                    confusion_matrix += get_confusion_matrix(
                        mask,
                        pred,
                        size,
                        num_classes,
                        val_config["ignore_label"]
                    )

            val_loss = float(np.mean(losses))

            pos = confusion_matrix.sum(1)
            res = confusion_matrix.sum(0)
            tp = np.diag(confusion_matrix)
            pixel_acc = tp.sum() / pos.sum()
            mean_acc = (tp / np.maximum(1.0, pos)).mean()
            IoU_array = (tp / np.maximum(1.0, pos + res - tp))
            mean_IoU = IoU_array.mean()

            end = time.time()
            elapsed = end - start
            hours = int(elapsed / 3600)
            minutes = int(elapsed / 60 % 60)
            seconds = int(elapsed % 60)
            timestamp = f"{hours}:{minutes:02d}:{seconds:02d}" if hours > 0 else f"{minutes:02d}:{seconds:02d}"

            print(f"Validation ({timestamp}) -> Loss: {val_loss}, MeanIoU: {mean_IoU:4.4f}, Pixel_Acc: {pixel_acc:4.4f}, Mean_Acc: {mean_acc:4.4f}, Class IoU: {IoU_array}")

            if mean_IoU > best_miou:
                best_state_dict = copy.deepcopy(model.state_dict())
                best_epoch = epoch
                best_miou = float(mean_IoU)
                if "save_path" in train_config:
                    torch.save(best_state_dict, train_config["save_path"])
                    print(f"Saved to {train_config['save_path']}")
            elif "patience" in train_config and epoch - best_epoch > train_config["patience"]:
                return

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("--config", type=str, default="config/ddrnet.yaml")
    parser.add_argument("--stage", type=str)

    args = parser.parse_args()

    main(args)
