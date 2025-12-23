from .ddrnet_23_slim import DualResNet, BasicBlock
from .gcnet import GCNetSeg

def load_ddrnet_23_slim(num_classes: int) -> DualResNet:
    return DualResNet(
        block=BasicBlock,
        layers=[2, 2, 2, 2],
        num_classes=num_classes,
        planes=32,
        spp_planes=128,
        head_planes=64
    )

def load_gcnet(num_classes: int):
    return GCNetSeg(num_classes=num_classes)

model_generators = {
    "ddrnet": load_ddrnet_23_slim,
    "gcnet": load_gcnet
}
