import os

import cv2
import numpy as np
from PIL import Image
import torch
from torch.utils.data import Dataset

class Cityscapes(Dataset):
    def __init__(
        self, 
        root, 
        list_path, 
        num_classes=19,
        flip=True, 
        ignore_label=255,
        base_size=2048,
        crop_size=(512, 1024),
        mean=[0.485, 0.456, 0.406], 
        std=[0.229, 0.224, 0.225]
    ):
        super(Cityscapes, self).__init__()

        self.base_size = base_size
        self.crop_size = crop_size
        self.ignore_label = ignore_label

        self.mean = mean
        self.std = std

        self.root = root
        self.list_path = list_path
        self.num_classes = num_classes

        self.flip = flip
        
        self.img_list = [line.strip().split() for line in open(root+list_path)]

        self.files = self.read_files()

        self.label_mapping = {
            -1: ignore_label,
            0: ignore_label, 
            1: ignore_label,
            2: ignore_label,
            3: ignore_label,
            4: ignore_label,
            5: ignore_label,
            6: ignore_label,
            7: 0,
            8: 1,
            9: ignore_label,
            10: ignore_label,
            11: 2,
            12: 3,
            13: 4,
            14: ignore_label,
            15: ignore_label,
            16: ignore_label,
            17: 5,
            18: ignore_label,
            19: 6,
            20: 7,
            21: 8,
            22: 9,
            23: 10,
            24: 11,
            25: 12,
            26: 13,
            27: 14,
            28: 15,
            29: ignore_label,
            30: ignore_label,
            31: 16,
            32: 17,
            33: 18
        }
        self.class_weights = torch.FloatTensor(
            [0.8373, 0.918, 0.866, 1.0345, 
            1.0166, 0.9969, 0.9754, 1.0489,
            0.8786, 1.0023, 0.9539, 0.9843, 
            1.1116, 0.9037, 1.0865, 1.0955, 
            1.0865, 1.1529, 1.0507]
        )
    
    def __len__(self):
        return len(self.files)
    
    def input_transform(self, image):
        image = image.astype(np.float32)[:, :, ::-1]
        image = image / 255.0
        image -= self.mean
        image /= self.std
        return image
    
    def label_transform(self, label):
        return np.array(label).astype('int32')
    
    def gen_sample(self, image, label, is_flip=True):
        image = self.input_transform(image)
        label = self.label_transform(label)

        image = image.transpose((2, 0, 1))

        if is_flip:
            flip = np.random.choice(2) * 2 - 1
            image = image[:, :, ::flip]
            label = label[:, ::flip]

        return image, label
    
    def read_files(self):
        files = []
        if 'test' in self.list_path:
            for item in self.img_list:
                image_path = item
                name = os.path.splitext(os.path.basename(image_path[0]))[0]
                files.append({
                    "img": image_path[0],
                    "name": name,
                })
        else:
            for item in self.img_list:
                image_path, label_path = item
                name = os.path.splitext(os.path.basename(label_path))[0]
                files.append({
                    "img": image_path,
                    "label": label_path,
                    "name": name,
                    "weight": 1
                })
        return files
        
    def convert_label(self, label, inverse=False):
        temp = label.copy()
        if inverse:
            for v, k in self.label_mapping.items():
                label[temp == k] = v
        else:
            for k, v in self.label_mapping.items():
                label[temp == k] = v
        return label

    def __getitem__(self, index):
        item = self.files[index]
        image = cv2.imread(os.path.join(self.root,'cityscapes',item["img"]),
                           cv2.IMREAD_COLOR)

        if 'test' in self.list_path:
            image = self.input_transform(image)
            image = image.transpose((2, 0, 1))

            return image.copy()

        label = cv2.imread(os.path.join(self.root,'cityscapes',item["label"]),
                           cv2.IMREAD_GRAYSCALE)
        label = self.convert_label(label)

        image, label = self.gen_sample(image, label, self.flip)

        return image.copy(), label.copy()

    def get_palette(self, n):
        palette = [0] * (n * 3)
        for j in range(0, n):
            lab = j
            palette[j * 3 + 0] = 0
            palette[j * 3 + 1] = 0
            palette[j * 3 + 2] = 0
            i = 0
            while lab:
                palette[j * 3 + 0] |= (((lab >> 0) & 1) << (7 - i))
                palette[j * 3 + 1] |= (((lab >> 1) & 1) << (7 - i))
                palette[j * 3 + 2] |= (((lab >> 2) & 1) << (7 - i))
                i += 1
                lab >>= 3

        return palette

    def save_pred(self, preds, sv_path, name):
        palette = self.get_palette(256)
        preds = np.asarray(np.argmax(preds.cpu(), axis=1), dtype=np.uint8)
        for i in range(preds.shape[0]):
            pred = self.convert_label(preds[i], inverse=True)
            save_img = Image.fromarray(pred)
            save_img.putpalette(palette)
            save_img.save(os.path.join(sv_path, name[i]+'.png'))
