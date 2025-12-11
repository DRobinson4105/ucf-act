import argparse

from tqdm import tqdm
import numpy as np
import torch
import torch.backends.cudnn as cudnn

import _init_paths
from lib.datasets import CityscapesDataset
from lib.models import load_ddrnet_slim
from lib.core import get_confusion_matrix

MODEL_OPTIONS = ["ddrnet_slim"]
DATASET_OPTIONS = ["cityscapes"]

def main(args):
    cudnn.benchmark = True
    cudnn.deterministic = False
    cudnn.enabled = True

    if args.model == "ddrnet_slim":
        model = load_ddrnet_slim()
    else:
        raise RuntimeError(f"Unsupported model: {args.model}, options are {MODEL_OPTIONS}")
    
    if args.checkpoint:
        checkpoint = torch.load(args.checkpoint, map_location='cpu')
        model.load_state_dict(checkpoint, strict=True)

    model = model.cuda()
    model.eval()
    
    if args.dataset == "cityscapes":
        dataset = CityscapesDataset(
            root="data/",
            list_path="list/cityscapes/val.lst"
        )
        num_classes = 19
    else:
        raise RuntimeError(f"Unsupported dataset: {args.dataset}, options are {DATASET_OPTIONS}")

    loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=1,
        shuffle=False
    )
        
    confusion_matrix = np.zeros((num_classes, num_classes))

    with torch.no_grad():
        for image, label in tqdm(loader):
            size = label.size()
            image, label = image.cuda(), label.cuda()

            pred = model(image)
            
            confusion_matrix += get_confusion_matrix(
                label,
                pred,
                size,
                num_classes,
                255)

    pos = confusion_matrix.sum(1)
    res = confusion_matrix.sum(0)
    tp = np.diag(confusion_matrix)
    pixel_acc = tp.sum()/pos.sum()
    mean_acc = (tp/np.maximum(1.0, pos)).mean()
    IoU_array = (tp / np.maximum(1.0, pos + res - tp))
    mean_IoU = IoU_array.mean()

    print(f"MeanIoU: {mean_IoU:4.4f}, Pixel_Acc: {pixel_acc: 4.4f}, \
        Mean_Acc: {mean_acc: 4.4f}, Class IoU:")
    print(IoU_array)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", type=str, choices=MODEL_OPTIONS)
    parser.add_argument("--checkpoint", type=str)
    parser.add_argument("--dataset", type=str, choices=DATASET_OPTIONS)

    args = parser.parse_args()

    main(args)
