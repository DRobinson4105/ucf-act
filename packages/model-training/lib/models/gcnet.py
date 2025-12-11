import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor

class ConvModule(nn.Module):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        kernel_size: int = 3,
        stride: int = 1,
        padding: int = 1,
        bias: bool = False,
        act: bool = True,
        order = ('conv', 'norm', 'act')
    ):
        super().__init__()
        self.conv = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size=kernel_size,
            stride=stride,
            padding=padding,
            bias=bias,
        )

        norm_channels = out_channels if order.index('norm') > order.index('conv') else in_channels
        self.bn = nn.SyncBatchNorm(norm_channels)
        self.relu = nn.ReLU(inplace=True)

        if act:
            self.op_map = {
                'conv': self.conv,
                'norm': self.bn,
                'act': self.relu
            }
            self.order = order
        else:
            self.op_map = {
                'conv': self.conv,
                'norm': self.bn,
            }
            self.order = tuple(x for x in order if x != 'act')

    def forward(self, x: Tensor):
        for op in self.order:
            x = self.op_map[op](x)

        return x

class ConvBlock(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, stride, padding):
        super().__init__()

        self.conv1 = ConvModule(in_channels, out_channels, kernel_size=kernel_size, stride=stride, padding=padding)
        self.conv2 = ConvModule(out_channels, out_channels, kernel_size=1, stride=1, padding=0)

    def forward(self, x):
        x = self.conv1(x)
        x = self.conv2(x)
        return x
    
class GCBlock(nn.Module):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        stride: int = 1,
        use_residual_bn: bool = True,
    ):
        super().__init__()

        self.path_3x3_1 = ConvBlock(in_channels, out_channels, kernel_size=3, stride=stride, padding=1)
        self.path_3x3_2 = ConvBlock(in_channels, out_channels, kernel_size=3, stride=stride, padding=1)
        self.path_1x1 = ConvBlock(in_channels, out_channels, kernel_size=1, stride=stride, padding=0)

        self.use_residual_bn = use_residual_bn
        if use_residual_bn:
            self.path_residual = nn.SyncBatchNorm(in_channels)

        self.relu = nn.ReLU(inplace=True)

    def forward(self, x: Tensor):
        if self.use_residual_bn:
            res = self.path_residual(x)
        else:
            res = 0

        out = (
            self.path_3x3_1(x)
            + self.path_3x3_2(x)
            + self.path_1x1(x)
            + res
        )
        return self.relu(out)


class SPP(nn.Module):
    def __init__(
        self,
        param1: int = 512,
        param2: int = 128
    ):
        super().__init__()

        self.scales = nn.ModuleList()
        self.scales.append(
            ConvModule(param1, param2, kernel_size=1, padding=0, order=('norm', 'act', 'conv'))
        )
        self.scales.append(
            nn.Sequential(
                nn.AvgPool2d(kernel_size=5, stride=2, padding=2),
                ConvModule(param1, param2, kernel_size=1, padding=0, order=('norm', 'act', 'conv')),
            )
        )
        self.scales.append(
            nn.Sequential(
                nn.AvgPool2d(kernel_size=9, stride=4, padding=4),
                ConvModule(param1, param2, kernel_size=1, padding=0, order=('norm', 'act', 'conv')),
            )
        )
        self.scales.append(
            nn.Sequential(
                nn.AvgPool2d(kernel_size=17, stride=8, padding=8),
                ConvModule(param1, param2, kernel_size=1, padding=0, order=('norm', 'act', 'conv')),
            )
        )
        self.scales.append(
            nn.Sequential(
                nn.AdaptiveAvgPool2d(1),
                ConvModule(param1, param2, kernel_size=1, padding=0, order=('norm', 'act', 'conv')),
            )
        )

        self.processes = nn.ModuleList(
            [
                ConvModule(
                    param2,
                    param2,
                    kernel_size=3,
                    padding=1,
                )
                for _ in range(4)
            ]
        )

        self.compression = ConvModule(
            param2 * 5,
            param2,
            kernel_size=1,
            padding=0,
            order=('norm', 'act', 'conv')
        )
        self.shortcut = ConvModule(
            param1,
            param2,
            kernel_size=1,
            padding=0,
            order=('norm', 'act', 'conv')
        )

    def forward(self, x: Tensor):
        h, w = x.shape[2:]

        x0 = self.scales[0](x)
        feats = [x0]

        for i in range(1, 5):
            s = self.scales[i](x)
            s = F.interpolate(
                s,
                size=(h, w),
                mode="bilinear",
                align_corners=False,
            )
            s = self.processes[i - 1](s + feats[-1])
            feats.append(s)

        x_cat = torch.cat(feats, dim=1)
        out = self.compression(x_cat) + self.shortcut(x)
        return out

class GCBackbone(nn.Module):
    def __init__(
        self,
        param1: int = 3,
        param2: int = 32,
        param3: int = 64,
        param4: int = 128,
        param5: int = 256,
        param6: int = 512
    ):
        super().__init__()

        self.stem = nn.Sequential(
            ConvModule(param1, param2, kernel_size=3, stride=2, padding=1),
            ConvModule(param2, param2, kernel_size=3, stride=2, padding=1),

            GCBlock(param2, param2, use_residual_bn=True),
            GCBlock(param2, param2, use_residual_bn=True),
            GCBlock(param2, param2, use_residual_bn=True),
            GCBlock(param2, param2, use_residual_bn=True),

            GCBlock(param2, param3, stride=2, use_residual_bn=False),
            GCBlock(param3, param3, use_residual_bn=True),
            GCBlock(param3, param3, use_residual_bn=True),
            GCBlock(param3, param3, use_residual_bn=True),
        )

        stage0 = nn.Sequential(
            GCBlock(param3, param4, stride=2, use_residual_bn=False),
            GCBlock(param4, param4, use_residual_bn=True),
            GCBlock(param4, param4, use_residual_bn=True),
            GCBlock(param4, param4, use_residual_bn=True),
            GCBlock(param4, param4, use_residual_bn=True),
        )
        stage1 = nn.Sequential(
            GCBlock(param4, param5, stride=2, use_residual_bn=False),
            GCBlock(param5, param5, use_residual_bn=True),
            GCBlock(param5, param5, use_residual_bn=True),
            GCBlock(param5, param5, use_residual_bn=True),
            GCBlock(param5, param5, use_residual_bn=True),
        )
        stage2 = nn.Sequential(
            GCBlock(param5, param6, stride=2, use_residual_bn=False),
            GCBlock(param6, param6, use_residual_bn=True),
        )
        self.semantic_branch_layers = nn.ModuleList([stage0, stage1, stage2])

        detail_stage0 = nn.Sequential(
            GCBlock(param3, param3, use_residual_bn=True),
            GCBlock(param3, param3, use_residual_bn=True),
            GCBlock(param3, param3, use_residual_bn=True),
            GCBlock(param3, param3, use_residual_bn=True),
        )
        detail_stage1 = nn.Sequential(
            GCBlock(param3, param3, use_residual_bn=True),
            GCBlock(param3, param3, use_residual_bn=True),
            GCBlock(param3, param3, use_residual_bn=True),
            GCBlock(param3, param3, use_residual_bn=True),
        )
        detail_stage2 = nn.Sequential(
            GCBlock(param3, param4, use_residual_bn=False),
            GCBlock(param4, param4, use_residual_bn=True),
        )
        self.detail_branch_layers = nn.ModuleList(
            [detail_stage0, detail_stage1, detail_stage2]
        )

        self.compression_1 = ConvModule(param4, param3, kernel_size=1, padding=0, act=False)
        self.down_1 = ConvModule(param3, param4, kernel_size=3, stride=2, padding=1, act=False)

        self.compression_2 = ConvModule(param5, param3, kernel_size=1, padding=0, act=False)
        self.down_2 = nn.Sequential(
            ConvModule(param3, param4, kernel_size=3, stride=2, padding=1),
            ConvModule(param4, param5, kernel_size=3, stride=2, padding=1, act=False),
        )

        self.spp = SPP(
            param1=param6,
            param2=param4
        )
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x: Tensor):
        x = self.stem(x)

        out_size = x.shape[-2:]

        x_s = self.semantic_branch_layers[0](x)
        x_d = self.detail_branch_layers[0](x)

        comp_c = self.compression_1(self.relu(x_s))
        x_s = x_s + self.down_1(self.relu(x_d))
        x_d = x_d + F.interpolate(
            comp_c,
            size=out_size,
            mode="bilinear",
            align_corners=False,
        )

        if self.training:
            c4_feat = x_d.clone()

        x_s = self.semantic_branch_layers[1](self.relu(x_s))
        x_d = self.detail_branch_layers[1](self.relu(x_d))

        comp_c = self.compression_2(self.relu(x_s))
        x_s = x_s + self.down_2(self.relu(x_d))
        x_d = x_d + F.interpolate(
            comp_c,
            size=out_size,
            mode="bilinear",
            align_corners=False,
        )

        x_d = self.detail_branch_layers[2](self.relu(x_d))
        x_s = self.semantic_branch_layers[2](self.relu(x_s))

        x_s = self.spp(x_s)
        x_s = F.interpolate(
            x_s,
            size=out_size,
            mode="bilinear",
            align_corners=False,
        )

        return (x_d + x_s, c4_feat) if self.training else x_d + x_s

class GCDecodeHead(nn.Module):
    def __init__(self, in_channels: int, channels: int, num_classes: int):
        super().__init__()

        self.in_channels = in_channels
        self.channels = channels
        self.out_channels = num_classes

        self.head = self._make_base_head(in_channels, channels)
        self.conv_seg = nn.Conv2d(channels, num_classes, kernel_size=1)

        self.aux_head_c4 = self._make_base_head(in_channels // 2, channels)
        self.aux_cls_seg_c4 = nn.Conv2d(channels, num_classes, kernel_size=1)

        self.init_weights()

    def _make_base_head(self, in_channels: int, channels: int):
        layers = [
            nn.SyncBatchNorm(in_channels),
            nn.ReLU(inplace=True),
            ConvModule(
                in_channels,
                channels,
                kernel_size=3,
                padding=1,
                order=('conv', 'norm', 'act'),
            ),
        ]
        return nn.Sequential(*layers)

    def init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, (nn.BatchNorm2d, nn.SyncBatchNorm)):
                nn.init.constant_(m.weight, 1.0)
                nn.init.constant_(m.bias, 0.0)

    def forward(self, inputs):
        if self.training:
            c6_feat, c4_feat = inputs

            aux_feat = self.aux_head_c4(c4_feat)
            aux_logits = self.aux_cls_seg_c4(aux_feat)

            main_feat = self.head(c6_feat)
            main_logits = self.conv_seg(main_feat)

            return main_logits, aux_logits
        else:
            c6_feat = inputs
            main_feat = self.head(c6_feat)
            main_logits = self.conv_seg(main_feat)
            return main_logits

class GCNetSeg(nn.Module):
    def __init__(self, num_classes: int = 19):
        super().__init__()

        self.backbone = GCBackbone(
            param1=3,
            param2=32,
            param3=64,
            param4=128,
            param5=256,
            param6=512,
        )

        self.decode_head = GCDecodeHead(
            in_channels=128,
            channels=64,
            num_classes=num_classes
        )

    def forward(self, x: Tensor):
        H, W = x.shape[-2], x.shape[-1]
        inputs = self.backbone(x)
        if self.training:
            main, aux = self.decode_head(inputs)
            main = F.interpolate(main, size=(H, W),
                                 mode="bilinear", align_corners=False)
            aux = F.interpolate(aux, size=(H, W),
                                mode="bilinear", align_corners=False)
            return main, aux
        else:
            main = self.decode_head(inputs)
            main = F.interpolate(main, size=(H, W),
                                 mode="bilinear", align_corners=False)
            return main
