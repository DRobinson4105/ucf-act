from torch.optim import SGD

from .criterion import CrossEntropy, OhemCrossEntropy
from .evaluation_metrics import get_confusion_matrix
from .lr_scheduler import get_poly_lr_scheduler

criterion_generators = {
    "ohem": OhemCrossEntropy,
    "cross_entropy": CrossEntropy
}

optimizer_generators = {
    "sgd": SGD
}

lr_scheduler_generators = {
    "poly": get_poly_lr_scheduler
}
