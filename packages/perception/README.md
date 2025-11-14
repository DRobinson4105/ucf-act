# Environment Setup

```bash
conda create -n act_perception python=3.11 -y
conda activate act_perception

# install torch and torchvision with cuda 12.1
pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121

# build mmcv first
pip install -U openmim
mim install mmengine
mim install mmcv==2.2.0

# install mmseg
cd mmsegmentation
pip install -e .
cd ..

# install other dependencies while pinning torch==2.5.1 and torchvision==0.20.1
pip install -r requirements.txt -c constraints.txt

# download detection model (YOLOv11)
python scripts/load_det_model.py

# download segmentation model
python scripts/load_seg_model.py --model {gcnet, ddrnet}
```
