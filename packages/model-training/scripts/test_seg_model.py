import argparse

from tqdm import tqdm
import numpy as np
import torch
import torch.backends.cudnn as cudnn
import yaml

import _init_paths
from lib.datasets import dataset_generators
from lib.models import model_generators
from lib.core import get_confusion_matrix

def main(args):
    with open(args.config, 'r') as fp:
        config = yaml.safe_load(fp)

    torch.manual_seed(config["seed"])
    np.random.seed(config["seed"])

    test_config = config["test"]

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

    num_classes = config["model"]["num_classes"]
    model_name = config["model"]["name"]

    if model_name in model_generators:
        model = model_generators[model_name](num_classes=num_classes)
    else:
        raise RuntimeError(f"Unsupported model: {model_name}, options are {model_generators}")
    
    if "load_path" in test_config:
        checkpoint = torch.load(test_config["load_path"], map_location='cpu')
        model.load_state_dict(checkpoint, strict=True)

    model = model.to(device)
    model.eval()
    
    dataset_name = test_config["dataset"]

    if dataset_name in dataset_generators:
        dataset = dataset_generators[dataset_name](
            **test_config["dataset_parameters"]
        )
    else:
        raise RuntimeError(f"Unsupported dataset: {dataset_name}, options are {dataset_generators}")

    loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=test_config["batch_size"],
        shuffle=test_config["shuffle"]
    )
        
    confusion_matrix = np.zeros((num_classes, num_classes))

    with torch.no_grad():
        for image, mask in tqdm(loader, leave=False):
            size = mask.size()
            image, mask = image.to(device), mask.to(device)

            logits = model(image)

            pred = logits.argmax(dim=1)
            
            confusion_matrix += get_confusion_matrix(
                mask,
                pred,
                size,
                num_classes,
                test_config["ignore_label"]
            )

    pos = confusion_matrix.sum(1)
    res = confusion_matrix.sum(0)
    tp = np.diag(confusion_matrix)
    pixel_acc = tp.sum() / pos.sum()
    mean_acc = (tp / np.maximum(1.0, pos)).mean()
    IoU_array = (tp / np.maximum(1.0, pos + res - tp))
    mean_IoU = IoU_array.mean()

    print(f"MeanIoU: {mean_IoU:4.4f}, Pixel_Acc: {pixel_acc: 4.4f}, Mean_Acc: {mean_acc: 4.4f}, Class IoU:")
    print(IoU_array)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("--config", type=str, default="config/ddrnet.yaml")

    args = parser.parse_args()

    main(args)
