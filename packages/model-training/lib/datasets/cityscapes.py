import os
import random

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

class Cityscapes(Dataset):
    def __init__(
        self, 
        root, 
        list_path, 
        num_classes=2,
        random_flip=True,
        crop_size=None,
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    ):
        super(Cityscapes, self).__init__()

        self.crop_size = crop_size

        self.mean = mean
        self.std = std

        self.root = root
        self.list_path = list_path
        self.num_classes = num_classes

        self.flip = random_flip
        
        self.img_list = [line.strip().split() for line in open(os.path.join(root, list_path))]

        self.files = self.read_files()

        self.label_mapping = {
            -1: 1,
            0: 1, 
            1: 1,
            2: 1,
            3: 1,
            4: 1,
            5: 1,
            6: 1,
            7: 0,
            8: 0,
            9: 1,
            10: 1,
            11: 1,
            12: 1,
            13: 1,
            14: 1,
            15: 1,
            16: 1,
            17: 1,
            18: 1,
            19: 1,
            20: 1,
            21: 1,
            22: 1,
            23: 1,
            24: 1,
            25: 1,
            26: 1,
            27: 1,
            28: 1,
            29: 1,
            30: 1,
            31: 1,
            32: 1,
            33: 1
        }
        self.class_weights = torch.FloatTensor([0.8777, 1.0144])
    
    def __len__(self):
        return len(self.files)
    
    def input_transform(self, image):
        image = image.astype(np.float32)[:, :, ::-1]
        image = image / 255.0
        image -= self.mean
        image /= self.std
        return image
    
    def label_transform(self, label):
        return np.array(label).astype('int64')
    
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
        image = cv2.imread(os.path.join(self.root,item["img"]),
                           cv2.IMREAD_COLOR)

        if 'test' in self.list_path:
            image = self.input_transform(image)
            image = image.transpose((2, 0, 1))

            return image.copy()

        label = cv2.imread(os.path.join(self.root,item["label"]),
                           cv2.IMREAD_GRAYSCALE)
        label = self.convert_label(label)

        image, label = self.gen_sample(image, label)

        return image.copy(), label.copy()

    def gen_sample(self, image, label):
        image = self.input_transform(image)
        label = self.label_transform(label)

        image = image.transpose((2, 0, 1))

        if self.flip:
            flip = np.random.choice(2) * 2 - 1
            image = image[:, :, ::flip]
            label = label[:, ::flip]

        return image, label
