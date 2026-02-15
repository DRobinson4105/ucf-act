# Model Training

## Dataset Download (if training from scratch)

**Cityscapes:** Download `leftImg8bit_trainvaltest.zip` and `gtFine_trainvaltest.zip` from https://www.cityscapes-dataset.com/downloads and move `leftImg8bit` and `gtFine` subfolders to `data/cityscapes`

```bash
mkdir data

# COCO Filtered
pip install tqdm
python scripts/download_coco.py

# UCF Segmentation
git clone https://huggingface.co/datasets/DavidRobinson05/ucf-seg data/ucf-seg

# UCF Detection
git clone https://huggingface.co/datasets/DavidRobinson05/ucf-det data/ucf-det

```

## Environment Setup

If `conda` is not installed, run the following commands:
```bash
# ARM64
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
bash Miniconda3-latest-Linux-aarch64.sh
rm Miniconda3-latest-Linux-aarch64.sh

# x86
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
rm Miniconda3-latest-Linux-x86_64.sh

# after installing
source ~/.bashrc
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
```

Create environment:
```bash
# create conda environment
conda create -n act python=3.10 -y
conda activate act

# install with cuda 12.1 (windows 11)
conda install -c conda-forge cudnn=9
pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121 onnxruntime-gpu==1.23.2 tensorrt_cu12==10.14.1.48.post1
pip install -r requirements.txt -c constraints_121.txt

# install with cuda 12.6 (jp6)
conda install -c conda-forge -y libcudss0
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
pip install torch==2.9.1 torchvision==0.24.1 onnxruntime-gpu==1.23.0 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
pip install -r requirements.txt -c constraints_126.txt

# install with cpu
pip install torch==2.9.1 torchvision==0.24.1 onnxruntime==1.23.2
pip install -r requirements.txt -c constraints_126.txt

# install other dependencies while pinning torch==2.5.1 and torchvision==0.20.1

# download detection weights (YOLOv11)
python scripts/load_det_weights.py

# train segmentation model
python scripts/load_seg_weights.py --config configs/ddrnet.yaml
python scripts/train_seg_model.py --config configs/ddrnet.yaml --stage pretrain
python scripts/train_seg_model.py --config configs/ddrnet.yaml --stage finetune

# test segmentation model
python scripts/test_seg_model.py --config configs/ddrnet.yaml
```
