from .ddrnet_23_slim import DualResNet, BasicBlock
from .gcnet import GCNetSeg

def load_ddrnet_slim(augment=False) -> DualResNet:
    return DualResNet(
        block=BasicBlock,
        layers=[2, 2, 2, 2],
        num_classes=19,
        planes=32,
        spp_planes=128,
        head_planes=64,
        augment=False
    )

def load_gcnet():
    return GCNetSeg(num_classes=19)