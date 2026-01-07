# Model Training

## Dataset Download (if training from scratch)

**Cityscapes:** Download `leftImg8bit_trainvaltest.zip` and `gtFine_trainvaltest.zip` from https://www.cityscapes-dataset.com/downloads and move `leftImg8bit` and `gtFine` subfolders to `data/cityscapes`

```bash
mkdir data

# UCF Segmentation
git clone https://huggingface.co/datasets/DavidRobinson05/ucf-seg data/ucf-seg

# UCF Detection
git clone https://huggingface.co/datasets/DavidRobinson05/ucf-det data/ucf-det
```

## Environment Setup

```bash
conda create -n act_perception python=3.11 -y
conda activate act_perception

# install with cuda 12.1
conda install -c conda-forge cudnn=9
pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121 onnxruntime-gpu==1.23.2 tensorrt_cu12==10.14.1.48.post1

# install with cpu
pip install torch==2.5.1 torchvision==0.20.1 onnxruntime==1.23.2

# install other dependencies while pinning torch==2.5.1 and torchvision==0.20.1
pip install -r requirements.txt -c constraints.txt

# download detection weights (YOLOv11)
python scripts/load_det_weights.py

# train segmentation model
python scripts/load_seg_weights.py --config configs/ddrnet.yaml
python scripts/train_seg_model.py --config configs/ddrnet.yaml --stage pretrain
python scripts/train_seg_model.py --config configs/ddrnet.yaml --stage finetune

# test segmentation model
python scripts/test_seg_model.py --config configs/ddrnet.yaml
```
