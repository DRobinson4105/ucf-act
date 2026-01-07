from .cityscapes import Cityscapes
from .ucf_seg import UCFSeg

dataset_generators = {
    "cityscapes": Cityscapes,
    "ucf_seg": UCFSeg,
}
