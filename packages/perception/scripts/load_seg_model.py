import os

from mim.commands.download import download

os.makedirs("models", exist_ok=True)

download(
    package="mmsegmentation",
    configs=["ddrnet_23-slim_in1k-pre_2xb6-120k_cityscapes-1024x1024"],
    dest_root="models"
)
