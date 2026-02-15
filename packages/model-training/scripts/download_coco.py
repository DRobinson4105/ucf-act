import json
import os
from pathlib import Path
import zipfile
import shutil
from urllib.request import urlopen

from tqdm import tqdm

train_url = "http://images.cocodataset.org/zips/train2017.zip"
val_url = "http://images.cocodataset.org/zips/val2017.zip"
annotation_url = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"

train_zip = "train2017.zip"
val_zip = "val2017.zip"
annotation_zip = "annotations_trainval2017.zip"

train_img_dir = Path("data/coco_filtered/train/images")
val_img_dir = Path("data/coco_filtered/val/images")

Path("data/coco_filtered/train/images").mkdir(parents=True, exist_ok=False)
Path("data/coco_filtered/val/images").mkdir(parents=True, exist_ok=False)
Path("data/coco_filtered/train/labels").mkdir(parents=True, exist_ok=False)
Path("data/coco_filtered/val/labels").mkdir(parents=True, exist_ok=False)

def extract_zip(url, zip_file):
    with urlopen(url) as response, open(zip_file, "wb") as out_file:
        shutil.copyfileobj(response, out_file)

    with zipfile.ZipFile(zip_file, "r") as fp:
        fp.extractall()

extract_zip(train_url, train_zip)
extract_zip(val_url, val_zip)
extract_zip(annotation_url, annotation_zip)

for img in Path("train2017/train2017").iterdir():
    shutil.move(str(img), train_img_dir / img.name)
    
for img in Path("val2017/val2017").iterdir():
    shutil.move(str(img), val_img_dir / img.name)

category_names = {
    1: 0,
    2: 1,
    3: 2,
    4: 3,
    6: 4,
    8: 5
}

for split in ["train", "val"]:
    for file in tqdm(list(Path(f"data/coco_filtered/{split}/images").iterdir())):
        open(f"data/coco_filtered/{split}/labels/{file.name.replace('.jpg', '.txt')}", "w").close()

    with open(f"annotations_trainval2017/annotations/instances_{split}2017.json") as fp:
        data = json.load(fp)

    image_sizes = dict()

    for image in data["images"]:
        image_sizes[image['id']] = (image['width'], image['height'])

    for annotation in tqdm(data['annotations']):
        if annotation['category_id'] in category_names.keys():
            image_path = f"{annotation['image_id']:012d}.txt"
            with open(f"data/coco_filtered/{split}/labels/{image_path}", "a") as fp:
                category = category_names[annotation['category_id']]
                image_width, image_height = image_sizes[annotation["image_id"]]
                x0, y0, w, h = annotation['bbox']
                x = (x0 + w / 2) / image_width
                y = (y0 + h / 2) / image_height
                w = w / image_width
                h = h / image_height
                fp.write(f"{category} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n")

shutil.rmtree("train2017")
shutil.rmtree("val2017")
shutil.rmtree("annotations_trainval2017")

Path("train2017.zip").unlink(missing_ok=True)
Path("val2017.zip").unlink(missing_ok=True)
Path("annotations_trainval2017.zip").unlink(missing_ok=True)
