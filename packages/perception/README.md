# Environment Setup

```bash
conda create -n act_perception python=3.11 -y
conda activate act_perception

# install torch and torchvision with cuda 12.1
conda install pytorch==2.5.1 torchvision==0.20.1 pytorch-cuda=12.1 -c pytorch -c nvidia -y

# build mmcv first
pip install -U openmim
mim install mmengine
mim install mmcv==2.2.0

# install mmsegmentation without assertion against mmcv==2.2.0 
cd mmsegmentation
pip install -e .
cd ..

# install other dependencies while pinning torch==2.5.1 and torchvision==0.20.1
pip install -r requirements.txt -c constraints.txt
```