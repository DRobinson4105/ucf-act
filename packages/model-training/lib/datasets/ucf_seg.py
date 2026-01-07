import random
from pathlib import Path

import torch
from torch.utils.data import Dataset
import torchvision.transforms as T
import torchvision.transforms.functional as TF
from PIL import Image

class UCFSeg(Dataset):
    def __init__(
        self,
        root: str,
        num_classes=2,
        random_flip=True,
        crop_size=None,
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    ):
        root_path = Path(root)
        
        self.images = sorted((root_path / "images").iterdir())
        self.masks = sorted((root_path / "labels").iterdir())

        self.crop_size = crop_size
        self.num_classes = num_classes
        self.flip = random_flip

        self.transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=mean, std=std)
        ])

        self.target_transform = T.Compose([
            T.ToTensor(),
            lambda x: x.squeeze(0).long()
        ])

    def __len__(self):
        return len(self.images)
    
    def __getitem__(self, idx: int):
        image = Image.open(self.images[idx]).convert("RGB")
        mask = Image.open(self.masks[idx]).convert("L")

        if self.crop_size is not None:
            i, j, h, w = T.RandomCrop.get_params(image, self.crop_size)

            image = TF.crop(image, i, j, h, w)
            mask = TF.crop(mask, i, j, h, w)

            if random.random() < 0.5:
                image = TF.hflip(image)
                mask = TF.hflip(mask)

        image = self.transform(image)
        mask = self.target_transform(mask)

        return image, mask
