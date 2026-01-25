import torch
import torch.nn as nn
from torch.nn import functional as F

class CrossEntropy(nn.Module):
    def __init__(self, ignore_label=-1, class_weight=None, weight=None, align_corners=False):
        super(CrossEntropy, self).__init__()
        self.ignore_label = ignore_label

        if class_weight is not None:
            class_weight = torch.Tensor(class_weight)
        self.criterion = nn.CrossEntropyLoss(
            weight=class_weight,
            ignore_index=ignore_label
        )
        self.weight = weight
        self.align_corners = align_corners

    def _forward(self, score, target):
        if score.dim() == 3:
            score = score.unsqueeze(0)
        ph, pw = score.size(2), score.size(3)
        h, w = target.size(1), target.size(2)
        if ph != h or pw != w:
            score = F.interpolate(input=score, size=(
                h, w), mode='bilinear', align_corners=self.align_corners)

        loss = self.criterion(score, target)

        return loss

    def forward(self, score, target):
        assert len(self.weight) >= len(score)

        return sum([w * self._forward(x, target) for (w, x) in zip(self.weight[:len(score)], score)])

class OhemCrossEntropy(nn.Module):
    def __init__(self, ignore_label=-1, ohem_threshold=0.7,
                 ohem_keep=100000, class_weight=None, weight=None, align_corners=False):
        super(OhemCrossEntropy, self).__init__()
        self.thresh = ohem_threshold
        self.min_kept = max(1, ohem_keep)
        self.ignore_label = ignore_label

        if class_weight is not None:
            class_weight = torch.Tensor(class_weight)
        self.criterion = nn.CrossEntropyLoss(
            weight=class_weight,
            ignore_index=ignore_label,
            reduction='none'
        )
        self.weight = weight
        self.align_corners = align_corners

    def _ce_forward(self, score, target):
        if score.dim() == 3:
            score = score.unsqueeze(0)
        ph, pw = score.size(2), score.size(3)
        h, w = target.size(1), target.size(2)
        if ph != h or pw != w:
            score = F.interpolate(input=score, size=(
                h, w), mode='bilinear', align_corners=self.align_corners)

        loss = self.criterion(score, target)

        return loss.mean()

    def _ohem_forward(self, score, target, **kwargs):
        if score.dim() == 3:
            score = score.unsqueeze(0)
        ph, pw = score.size(2), score.size(3)
        h, w = target.size(1), target.size(2)
        if ph != h or pw != w:
            score = F.interpolate(input=score, size=(
                h, w), mode='bilinear', align_corners=self.align_corners)
        pred = F.softmax(score, dim=1)
        pixel_losses = self.criterion(score, target).contiguous().view(-1)
        mask = target.contiguous().view(-1) != self.ignore_label

        tmp_target = target.clone()
        tmp_target[tmp_target == self.ignore_label] = 0
        pred = pred.gather(1, tmp_target.unsqueeze(1))
        pred, ind = pred.contiguous().view(-1,)[mask].contiguous().sort()
        k = min(self.min_kept, pred.numel())
        min_value = pred[k - 1]
        threshold = max(min_value, self.thresh)

        pixel_losses = pixel_losses[mask][ind]

        if threshold >= 1.0 - 1e-12:
            return pixel_losses.mean()

        hard = pixel_losses[pred < threshold]
        if hard.numel() < k:
            hard = pixel_losses[:k]

        return hard.mean()

    def forward(self, score, target):
        assert len(self.weight) >= len(score)

        functions = [self._ohem_forward] + [self._ce_forward] * \
            (len(score) - 1)
        return sum([
            w * func(x, target)
            for (w, x, func) in zip(self.weight[:len(score)], score, functions)
        ])
