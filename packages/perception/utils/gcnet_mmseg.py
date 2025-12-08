# import warnings
# from functools import partial
# from typing import Dict, Optional, Tuple, Union, Iterable, List, Sequence
# from abc import ABCMeta, abstractmethod
# import copy
# import math

# import torch
# import torch.nn as nn
# from torch import Tensor
# import torch.nn.functional as F
# from torch.nn.modules.batchnorm import _BatchNorm
# from torch.nn.modules.instancenorm import _InstanceNorm
# import numpy as np

# # from mmseg.registry import MODELS
# # from mmseg.utils import (ConfigType, OptConfigType, OptMultiConfig,
# #                          OptSampleList, SampleList, add_prefix)
# # from .base import BaseSegmentor
# # from mmengine.model import BaseModel
# # from mmengine.structures import PixelData
# # from mmseg.structures import SegDataSample
# # from mmseg.utils import (ForwardResults, OptConfigType, OptMultiConfig,
# #                          OptSampleList, SampleList)
# # from ..utils import resize
# # from mmengine.optim import OptimWrapper
# # from mmengine.registry import MODELS
# # from mmengine.utils import is_list_of
# # from ..base_module import BaseModule
# # from .data_preprocessor import BaseDataPreprocessor

# class_weight = [
#     0.8373, 0.918, 0.866, 1.0345, 1.0166, 0.9969, 0.9754, 1.0489, 0.8786,
#     1.0023, 0.9539, 0.9843, 1.1116, 0.9037, 1.0865, 1.0955, 1.0865, 1.1529,
#     1.0507
# ]

# class PixelData(BaseDataElement):
#     """Data structure for pixel-level annotations or predictions.

#     All data items in ``data_fields`` of ``PixelData`` meet the following
#     requirements:

#     - They all have 3 dimensions in orders of channel, height, and width.
#     - They should have the same height and width.

#     Examples:
#         >>> metainfo = dict(
#         ...     img_id=random.randint(0, 100),
#         ...     img_shape=(random.randint(400, 600), random.randint(400, 600)))
#         >>> image = np.random.randint(0, 255, (4, 20, 40))
#         >>> featmap = torch.randint(0, 255, (10, 20, 40))
#         >>> pixel_data = PixelData(metainfo=metainfo,
#         ...                        image=image,
#         ...                        featmap=featmap)
#         >>> print(pixel_data.shape)
#         (20, 40)

#         >>> # slice
#         >>> slice_data = pixel_data[10:20, 20:40]
#         >>> assert slice_data.shape == (10, 20)
#         >>> slice_data = pixel_data[10, 20]
#         >>> assert slice_data.shape == (1, 1)

#         >>> # set
#         >>> pixel_data.map3 = torch.randint(0, 255, (20, 40))
#         >>> assert tuple(pixel_data.map3.shape) == (1, 20, 40)
#         >>> with self.assertRaises(AssertionError):
#         ...     # The dimension must be 3 or 2
#         ...     pixel_data.map2 = torch.randint(0, 255, (1, 3, 20, 40))
#     """

#     def __setattr__(self, name: str, value: Union[torch.Tensor, np.ndarray]):
#         """Set attributes of ``PixelData``.

#         If the dimension of value is 2 and its shape meet the demand, it
#         will automatically expand its channel-dimension.

#         Args:
#             name (str): The key to access the value, stored in `PixelData`.
#             value (Union[torch.Tensor, np.ndarray]): The value to store in.
#                 The type of value must be `torch.Tensor` or `np.ndarray`,
#                 and its shape must meet the requirements of `PixelData`.
#         """
#         if name in ('_metainfo_fields', '_data_fields'):
#             if not hasattr(self, name):
#                 super().__setattr__(name, value)
#             else:
#                 raise AttributeError(f'{name} has been used as a '
#                                      'private attribute, which is immutable.')

#         else:
#             assert isinstance(value, (torch.Tensor, np.ndarray)), \
#                 f'Can not set {type(value)}, only support' \
#                 f' {(torch.Tensor, np.ndarray)}'

#             if self.shape:
#                 assert tuple(value.shape[-2:]) == self.shape, (
#                     'The height and width of '
#                     f'values {tuple(value.shape[-2:])} is '
#                     'not consistent with '
#                     'the shape of this '
#                     ':obj:`PixelData` '
#                     f'{self.shape}')
#             assert value.ndim in [
#                 2, 3
#             ], f'The dim of value must be 2 or 3, but got {value.ndim}'
#             if value.ndim == 2:
#                 value = value[None]
#                 warnings.warn('The shape of value will convert from '
#                               f'{value.shape[-2:]} to {value.shape}')
#             super().__setattr__(name, value)

#     # TODO torch.Long/bool
#     def __getitem__(self, item: Sequence[Union[int, slice]]) -> 'PixelData':
#         """
#         Args:
#             item (Sequence[Union[int, slice]]): Get the corresponding values
#                 according to item.

#         Returns:
#             :obj:`PixelData`: Corresponding values.
#         """

#         new_data = self.__class__(metainfo=self.metainfo)
#         if isinstance(item, tuple):

#             assert len(item) == 2, 'Only support to slice height and width'
#             tmp_item: List[slice] = list()
#             for index, single_item in enumerate(item[::-1]):
#                 if isinstance(single_item, int):
#                     tmp_item.insert(
#                         0, slice(single_item, None, self.shape[-index - 1]))
#                 elif isinstance(single_item, slice):
#                     tmp_item.insert(0, single_item)
#                 else:
#                     raise TypeError(
#                         'The type of element in input must be int or slice, '
#                         f'but got {type(single_item)}')
#             tmp_item.insert(0, slice(None, None, None))
#             item = tuple(tmp_item)
#             for k, v in self.items():
#                 setattr(new_data, k, v[item])
#         else:
#             raise TypeError(
#                 f'Unsupported type {type(item)} for slicing PixelData')
#         return new_data

#     @property
#     def shape(self):
#         """The shape of pixel data."""
#         if len(self._data_fields) > 0:
#             return tuple(self.values()[0].shape[-2:])
#         else:
#             return None

# def add_prefix(inputs, prefix):
#     """Add prefix for dict.

#     Args:
#         inputs (dict): The input dict with str keys.
#         prefix (str): The prefix to add.

#     Returns:

#         dict: The dict with keys updated with ``prefix``.
#     """

#     outputs = dict()
#     for name, value in inputs.items():
#         outputs[f'{prefix}.{name}'] = value

#     return outputs

# class _BatchNormXd(nn.modules.batchnorm._BatchNorm):
#     """A general BatchNorm layer without input dimension check.

#     Reproduced from @kapily's work:
#     (https://github.com/pytorch/pytorch/issues/41081#issuecomment-783961547)
#     The only difference between BatchNorm1d, BatchNorm2d, BatchNorm3d, etc
#     is `_check_input_dim` that is designed for tensor sanity checks.
#     The check has been bypassed in this class for the convenience of converting
#     SyncBatchNorm.
#     """

#     def _check_input_dim(self, input: torch.Tensor):
#         return

# def accuracy(pred, target, topk=1, thresh=None, ignore_index=None):
#     """Calculate accuracy according to the prediction and target.

#     Args:
#         pred (torch.Tensor): The model prediction, shape (N, num_class, ...)
#         target (torch.Tensor): The target of each prediction, shape (N, , ...)
#         ignore_index (int | None): The label index to be ignored. Default: None
#         topk (int | tuple[int], optional): If the predictions in ``topk``
#             matches the target, the predictions will be regarded as
#             correct ones. Defaults to 1.
#         thresh (float, optional): If not None, predictions with scores under
#             this threshold are considered incorrect. Default to None.

#     Returns:
#         float | tuple[float]: If the input ``topk`` is a single integer,
#             the function will return a single float as accuracy. If
#             ``topk`` is a tuple containing multiple integers, the
#             function will return a tuple containing accuracies of
#             each ``topk`` number.
#     """
#     assert isinstance(topk, (int, tuple))
#     if isinstance(topk, int):
#         topk = (topk, )
#         return_single = True
#     else:
#         return_single = False

#     maxk = max(topk)
#     if pred.size(0) == 0:
#         accu = [pred.new_tensor(0.) for i in range(len(topk))]
#         return accu[0] if return_single else accu
#     assert pred.ndim == target.ndim + 1
#     assert pred.size(0) == target.size(0)
#     assert maxk <= pred.size(1), \
#         f'maxk {maxk} exceeds pred dimension {pred.size(1)}'
#     pred_value, pred_label = pred.topk(maxk, dim=1)
#     # transpose to shape (maxk, N, ...)
#     pred_label = pred_label.transpose(0, 1)
#     correct = pred_label.eq(target.unsqueeze(0).expand_as(pred_label))
#     if thresh is not None:
#         # Only prediction values larger than thresh are counted as correct
#         correct = correct & (pred_value > thresh).t()
#     if ignore_index is not None:
#         correct = correct[:, target != ignore_index]
#     res = []
#     eps = torch.finfo(torch.float32).eps
#     for k in topk:
#         # Avoid causing ZeroDivisionError when all pixels
#         # of an image are ignored
#         correct_k = correct[:k].reshape(-1).float().sum(0, keepdim=True) + eps
#         if ignore_index is not None:
#             total_num = target[target != ignore_index].numel() + eps
#         else:
#             total_num = target.numel() + eps
#         res.append(correct_k.mul_(100.0 / total_num))
#     return res[0] if return_single else res

# class Accuracy(nn.Module):
#     """Accuracy calculation module."""

#     def __init__(self, topk=(1, ), thresh=None, ignore_index=None):
#         """Module to calculate the accuracy.

#         Args:
#             topk (tuple, optional): The criterion used to calculate the
#                 accuracy. Defaults to (1,).
#             thresh (float, optional): If not None, predictions with scores
#                 under this threshold are considered incorrect. Default to None.
#         """
#         super().__init__()
#         self.topk = topk
#         self.thresh = thresh
#         self.ignore_index = ignore_index

#     def forward(self, pred, target):
#         """Forward function to calculate accuracy.

#         Args:
#             pred (torch.Tensor): Prediction of models.
#             target (torch.Tensor): Target for each prediction.

#         Returns:
#             tuple[float]: The accuracies under different topk criterions.
#         """
#         return accuracy(pred, target, self.topk, self.thresh,
#                         self.ignore_index)

# class OhemCrossEntropy(nn.Module):
#     def __init__(self,
#                  ignore_label: int = 255,
#                  thres: float = 0.7,
#                  min_kept: int = 100000,
#                  loss_weight: float = 1.0,
#                  class_weight: Optional[Union[List[float], str]] = None,
#                  loss_name: str = 'loss_ohem'):
#         super().__init__()
#         self.thresh = thres
#         self.min_kept = max(1, min_kept)
#         self.ignore_label = ignore_label
#         self.loss_weight = loss_weight
#         self.loss_name_ = loss_name
#         self.class_weight = class_weight

#     def forward(self, score: Tensor, target: Tensor) -> Tensor:
#         """Forward function.
#         Args:
#             score (Tensor): Predictions of the segmentation head.
#             target (Tensor): Ground truth of the image.

#         Returns:
#             Tensor: Loss tensor.
#         """
#         # score: (N, C, H, W)
#         pred = F.softmax(score, dim=1)
#         if self.class_weight is not None:
#             class_weight = score.new_tensor(self.class_weight)
#         else:
#             class_weight = None

#         pixel_losses = F.cross_entropy(
#             score,
#             target,
#             weight=class_weight,
#             ignore_index=self.ignore_label,
#             reduction='none').contiguous().view(-1)  # (N*H*W)
#         mask = target.contiguous().view(-1) != self.ignore_label  # (N*H*W)

#         tmp_target = target.clone()  # (N, H, W)
#         tmp_target[tmp_target == self.ignore_label] = 0
#         # pred: (N, C, H, W) -> (N*H*W, C)
#         pred = pred.gather(1, tmp_target.unsqueeze(1))
#         # pred: (N*H*W, C) -> (N*H*W), ind: (N*H*W)
#         pred, ind = pred.contiguous().view(-1, )[mask].contiguous().sort()
#         if pred.numel() > 0:
#             min_value = pred[min(self.min_kept, pred.numel() - 1)]
#         else:
#             return score.new_tensor(0.0)
#         threshold = max(min_value, self.thresh)

#         pixel_losses = pixel_losses[mask][ind]
#         pixel_losses = pixel_losses[pred < threshold]
#         return self.loss_weight * pixel_losses.mean()

#     @property
#     def loss_name(self):
#         return self.loss_name_
    
# def resize(input,
#            size=None,
#            scale_factor=None,
#            mode='nearest',
#            align_corners=None,
#            warning=True):
#     if warning:
#         if size is not None and align_corners:
#             input_h, input_w = tuple(int(x) for x in input.shape[2:])
#             output_h, output_w = tuple(int(x) for x in size)
#             if output_h > input_h or output_w > output_h:
#                 if ((output_h > 1 and output_w > 1 and input_h > 1
#                      and input_w > 1) and (output_h - 1) % (input_h - 1)
#                         and (output_w - 1) % (input_w - 1)):
#                     warnings.warn(
#                         f'When align_corners={align_corners}, '
#                         'the output would more aligned if '
#                         f'input size {(input_h, input_w)} is `x+1` and '
#                         f'out size {(output_h, output_w)} is `nx+1`')
#     return F.interpolate(input, size, scale_factor, mode, align_corners)

# class BaseModule(nn.Module, metaclass=ABCMeta):
#     """Base module for all modules in openmmlab. ``BaseModule`` is a wrapper of
#     ``torch.nn.Module`` with additional functionality of parameter
#     initialization. Compared with ``torch.nn.Module``, ``BaseModule`` mainly
#     adds three attributes.

#     - ``init_cfg``: the config to control the initialization.
#     - ``init_weights``: The function of parameter initialization and recording
#       initialization information.
#     - ``_params_init_info``: Used to track the parameter initialization
#       information. This attribute only exists during executing the
#       ``init_weights``.

#     Note:
#         :obj:`PretrainedInit` has a higher priority than any other
#         initializer. The loaded pretrained weights will overwrite
#         the previous initialized weights.

#     Args:
#         init_cfg (dict or List[dict], optional): Initialization config dict.
#     """

#     def __init__(self, init_cfg: Union[dict, List[dict], None] = None):
#         """Initialize BaseModule, inherited from `torch.nn.Module`"""

#         # NOTE init_cfg can be defined in different levels, but init_cfg
#         # in low levels has a higher priority.

#         super().__init__()
#         # define default value of init_cfg instead of hard code
#         # in init_weights() function
#         self._is_init = False

#         self.init_cfg = copy.deepcopy(init_cfg)

#         # Backward compatibility in derived classes
#         # if pretrained is not None:
#         #     warnings.warn('DeprecationWarning: pretrained is a deprecated \
#         #         key, please consider using init_cfg')
#         #     self.init_cfg = dict(type='Pretrained', checkpoint=pretrained)

#     @property
#     def is_init(self):
#         return self._is_init

#     @is_init.setter
#     def is_init(self, value):
#         self._is_init = value

#     def __repr__(self):
#         s = super().__repr__()
#         if self.init_cfg:
#             s += f'\ninit_cfg={self.init_cfg}'
#         return s


# class Sequential(BaseModule, nn.Sequential):
#     """Sequential module in openmmlab.

#     Ensures that all modules in ``Sequential`` have a different initialization
#     strategy than the outer model

#     Args:
#         init_cfg (dict, optional): Initialization config dict.
#     """

#     def __init__(self, *args, init_cfg: Optional[dict] = None):
#         BaseModule.__init__(self, init_cfg)
#         nn.Sequential.__init__(self, *args)


# class ModuleList(BaseModule, nn.ModuleList):
#     """ModuleList in openmmlab.

#     Ensures that all modules in ``ModuleList`` have a different initialization
#     strategy than the outer model

#     Args:
#         modules (iterable, optional): An iterable of modules to add.
#         init_cfg (dict, optional): Initialization config dict.
#     """

#     def __init__(self,
#                  modules: Optional[Iterable] = None,
#                  init_cfg: Optional[dict] = None):
#         BaseModule.__init__(self, init_cfg)
#         nn.ModuleList.__init__(self, modules)

# def constant_init(module, val, bias=0):
#     if hasattr(module, 'weight') and module.weight is not None:
#         nn.init.constant_(module.weight, val)
#     if hasattr(module, 'bias') and module.bias is not None:
#         nn.init.constant_(module.bias, bias)

# def kaiming_init(module,
#                  a=0,
#                  mode='fan_out',
#                  nonlinearity='relu',
#                  bias=0,
#                  distribution='normal'):
#     assert distribution in ['uniform', 'normal']
#     if hasattr(module, 'weight') and module.weight is not None:
#         if distribution == 'uniform':
#             nn.init.kaiming_uniform_(
#                 module.weight, a=a, mode=mode, nonlinearity=nonlinearity)
#         else:
#             nn.init.kaiming_normal_(
#                 module.weight, a=a, mode=mode, nonlinearity=nonlinearity)
#     if hasattr(module, 'bias') and module.bias is not None:
#         nn.init.constant_(module.bias, bias)

# def efficient_conv_bn_eval_forward(bn: _BatchNorm,
#                                    conv: nn.modules.conv._ConvNd,
#                                    x: torch.Tensor):
#     """
#     Implementation based on https://arxiv.org/abs/2305.11624
#     "Tune-Mode ConvBN Blocks For Efficient Transfer Learning"
#     It leverages the associative law between convolution and affine transform,
#     i.e., normalize (weight conv feature) = (normalize weight) conv feature.
#     It works for Eval mode of ConvBN blocks during validation, and can be used
#     for training as well. It reduces memory and computation cost.

#     Args:
#         bn (_BatchNorm): a BatchNorm module.
#         conv (nn._ConvNd): a conv module
#         x (torch.Tensor): Input feature map.
#     """
#     # These lines of code are designed to deal with various cases
#     # like bn without affine transform, and conv without bias
#     weight_on_the_fly = conv.weight
#     if conv.bias is not None:
#         bias_on_the_fly = conv.bias
#     else:
#         bias_on_the_fly = torch.zeros_like(bn.running_var)

#     if bn.weight is not None:
#         bn_weight = bn.weight
#     else:
#         bn_weight = torch.ones_like(bn.running_var)

#     if bn.bias is not None:
#         bn_bias = bn.bias
#     else:
#         bn_bias = torch.zeros_like(bn.running_var)

#     # shape of [C_out, 1, 1, 1] in Conv2d
#     weight_coeff = torch.rsqrt(bn.running_var +
#                                bn.eps).reshape([-1] + [1] *
#                                                (len(conv.weight.shape) - 1))
#     # shape of [C_out, 1, 1, 1] in Conv2d
#     coefff_on_the_fly = bn_weight.view_as(weight_coeff) * weight_coeff

#     # shape of [C_out, C_in, k, k] in Conv2d
#     weight_on_the_fly = weight_on_the_fly * coefff_on_the_fly
#     # shape of [C_out] in Conv2d
#     bias_on_the_fly = bn_bias + coefff_on_the_fly.flatten() *\
#         (bias_on_the_fly - bn.running_mean)

#     return conv._conv_forward(x, weight_on_the_fly, bias_on_the_fly)

# class ConvModule(nn.Module):
#     """A conv block that bundles conv/norm/activation layers.

#     This block simplifies the usage of convolution layers, which are commonly
#     used with a norm layer (e.g., BatchNorm) and activation layer (e.g., ReLU).
#     It is based upon three build methods: `build_conv_layer()`,
#     `build_norm_layer()` and `build_activation_layer()`.

#     Besides, we add some additional features in this module.
#     1. Automatically set `bias` of the conv layer.
#     2. Spectral norm is supported.
#     3. More padding modes are supported. Before PyTorch 1.5, nn.Conv2d only
#     supports zero and circular padding, and we add "reflect" padding mode.

#     Args:
#         in_channels (int): Number of channels in the input feature map.
#             Same as that in ``nn._ConvNd``.
#         out_channels (int): Number of channels produced by the convolution.
#             Same as that in ``nn._ConvNd``.
#         kernel_size (int | tuple[int]): Size of the convolving kernel.
#             Same as that in ``nn._ConvNd``.
#         stride (int | tuple[int]): Stride of the convolution.
#             Same as that in ``nn._ConvNd``.
#         padding (int | tuple[int]): Zero-padding added to both sides of
#             the input. Same as that in ``nn._ConvNd``.
#         dilation (int | tuple[int]): Spacing between kernel elements.
#             Same as that in ``nn._ConvNd``.
#         groups (int): Number of blocked connections from input channels to
#             output channels. Same as that in ``nn._ConvNd``.
#         bias (bool | str): If specified as `auto`, it will be decided by the
#             norm_cfg. Bias will be set as True if `norm_cfg` is None, otherwise
#             False. Default: "auto".
#         conv_cfg (dict): Config dict for convolution layer. Default: None,
#             which means using conv2d.
#         norm_cfg (dict): Config dict for normalization layer. Default: None.
#         act_cfg (dict): Config dict for activation layer.
#             Default: dict(type='ReLU').
#         inplace (bool): Whether to use inplace mode for activation.
#             Default: True.
#         with_spectral_norm (bool): Whether use spectral norm in conv module.
#             Default: False.
#         padding_mode (str): If the `padding_mode` has not been supported by
#             current `Conv2d` in PyTorch, we will use our own padding layer
#             instead. Currently, we support ['zeros', 'circular'] with official
#             implementation and ['reflect'] with our own implementation.
#             Default: 'zeros'.
#         order (tuple[str]): The order of conv/norm/activation layers. It is a
#             sequence of "conv", "norm" and "act". Common examples are
#             ("conv", "norm", "act") and ("act", "conv", "norm").
#             Default: ('conv', 'norm', 'act').
#         efficient_conv_bn_eval (bool): Whether use efficient conv when the
#             consecutive bn is in eval mode (either training or testing), as
#             proposed in https://arxiv.org/abs/2305.11624 . Default: `False`.
#     """

#     _abbr_ = 'conv_block'

#     def __init__(self,
#                  in_channels: int,
#                  out_channels: int,
#                  kernel_size: Union[int, Tuple[int, int]],
#                  stride: Union[int, Tuple[int, int]] = 1,
#                  padding: Union[int, Tuple[int, int]] = 0,
#                  dilation: Union[int, Tuple[int, int]] = 1,
#                  groups: int = 1,
#                  bias: Union[bool, str] = 'auto',
#                  conv_cfg: Optional[Dict] = None,
#                  norm_cfg: Optional[Dict] = None,
#                  act_cfg: Optional[Dict] = dict(type='ReLU'),
#                  inplace: bool = True,
#                  with_spectral_norm: bool = False,
#                  padding_mode: str = 'zeros',
#                  order: tuple = ('conv', 'norm', 'act'),
#                  efficient_conv_bn_eval: bool = False):
#         super().__init__()
#         assert conv_cfg is None or isinstance(conv_cfg, dict)
#         assert norm_cfg is None or isinstance(norm_cfg, dict)
#         assert act_cfg is None or isinstance(act_cfg, dict)
#         official_padding_mode = ['zeros', 'circular']
#         self.conv_cfg = conv_cfg
#         self.norm_cfg = norm_cfg
#         self.act_cfg = act_cfg
#         self.inplace = inplace
#         self.with_spectral_norm = with_spectral_norm
#         self.with_explicit_padding = padding_mode not in official_padding_mode
#         self.order = order
#         assert isinstance(self.order, tuple) and len(self.order) == 3
#         assert set(order) == {'conv', 'norm', 'act'}

#         self.with_norm = norm_cfg is not None
#         self.with_activation = act_cfg is not None
#         # if the conv layer is before a norm layer, bias is unnecessary.
#         if bias == 'auto':
#             bias = not self.with_norm
#         self.with_bias = bias

#         if self.with_explicit_padding:
#             raise Exception("explicit padding used")

#         # reset padding to 0 for conv module
#         conv_padding = 0 if self.with_explicit_padding else padding
#         # build convolution layer
#         self.conv = nn.Conv2d(
#             in_channels,
#             out_channels,
#             kernel_size=kernel_size,
#             stride=stride,
#             padding=conv_padding,
#             dilation=dilation,
#             groups=groups,
#             bias=bias,
#         )
#         # export the attributes of self.conv to a higher level for convenience
#         self.in_channels = self.conv.in_channels
#         self.out_channels = self.conv.out_channels
#         self.kernel_size = self.conv.kernel_size
#         self.stride = self.conv.stride
#         self.padding = padding
#         self.dilation = self.conv.dilation
#         self.transposed = self.conv.transposed
#         self.output_padding = self.conv.output_padding
#         self.groups = self.conv.groups

#         if self.with_spectral_norm:
#             self.conv = nn.utils.spectral_norm(self.conv)

#         # build normalization layers
#         if self.with_norm:
#             # norm layer is after conv layer
#             if order.index('norm') > order.index('conv'):
#                 norm_channels = out_channels
#             else:
#                 norm_channels = in_channels
#             self.norm_name = 'bn'
#             norm = nn.SyncBatchNorm(norm_channels)  # type: ignore
#             self.add_module(self.norm_name, norm)
#             if self.with_bias:
#                 if isinstance(norm, (_BatchNorm, _InstanceNorm)):
#                     warnings.warn(
#                         'Unnecessary conv bias before batch/instance norm')
#         else:
#             self.norm_name = None  # type: ignore

#         self.turn_on_efficient_conv_bn_eval(efficient_conv_bn_eval)

#         # build activation layer
#         if self.with_activation:
#             self.activate = nn.ReLU(inplace=True)

#         # Use msra init by default
#         self.init_weights()

#     @property
#     def norm(self):
#         if self.norm_name:
#             return getattr(self, self.norm_name)
#         else:
#             return None

#     def init_weights(self):
#         # 1. It is mainly for customized conv layers with their own
#         #    initialization manners by calling their own ``init_weights()``,
#         #    and we do not want ConvModule to override the initialization.
#         # 2. For customized conv layers without their own initialization
#         #    manners (that is, they don't have their own ``init_weights()``)
#         #    and PyTorch's conv layers, they will be initialized by
#         #    this method with default ``kaiming_init``.
#         # Note: For PyTorch's conv layers, they will be overwritten by our
#         #    initialization implementation using default ``kaiming_init``.
#         if not hasattr(self.conv, 'init_weights'):
#             if self.with_activation and self.act_cfg['type'] == 'LeakyReLU':
#                 nonlinearity = 'leaky_relu'
#                 a = self.act_cfg.get('negative_slope', 0.01)
#             else:
#                 nonlinearity = 'relu'
#                 a = 0
#             kaiming_init(self.conv, a=a, nonlinearity=nonlinearity)
#         if self.with_norm:
#             constant_init(self.norm, 1, bias=0)

#     def forward(self,
#                 x: torch.Tensor,
#                 activate: bool = True,
#                 norm: bool = True) -> torch.Tensor:
#         layer_index = 0
#         while layer_index < len(self.order):
#             layer = self.order[layer_index]
#             if layer == 'conv':
#                 if self.with_explicit_padding:
#                     x = self.padding_layer(x)
#                 # if the next operation is norm and we have a norm layer in
#                 # eval mode and we have enabled `efficient_conv_bn_eval` for
#                 # the conv operator, then activate the optimized forward and
#                 # skip the next norm operator since it has been fused
#                 if layer_index + 1 < len(self.order) and \
#                         self.order[layer_index + 1] == 'norm' and norm and \
#                         self.with_norm and not self.norm.training and \
#                         self.efficient_conv_bn_eval_forward is not None:
#                     self.conv.forward = partial(
#                         self.efficient_conv_bn_eval_forward, self.norm,
#                         self.conv)
#                     layer_index += 1
#                     x = self.conv(x)
#                     del self.conv.forward
#                 else:
#                     x = self.conv(x)
#             elif layer == 'norm' and norm and self.with_norm:
#                 x = self.norm(x)
#             elif layer == 'act' and activate and self.with_activation:
#                 x = self.activate(x)
#             layer_index += 1
#         return x

#     def turn_on_efficient_conv_bn_eval(self, efficient_conv_bn_eval=True):
#         # efficient_conv_bn_eval works for conv + bn
#         # with `track_running_stats` option
#         if efficient_conv_bn_eval and self.norm \
#                             and isinstance(self.norm, _BatchNorm) \
#                             and self.norm.track_running_stats:
#             self.efficient_conv_bn_eval_forward = efficient_conv_bn_eval_forward  # noqa: E501
#         else:
#             self.efficient_conv_bn_eval_forward = None  # type: ignore

#     @staticmethod
#     def create_from_conv_bn(conv: torch.nn.modules.conv._ConvNd,
#                             bn: torch.nn.modules.batchnorm._BatchNorm,
#                             efficient_conv_bn_eval=True) -> 'ConvModule':
#         """Create a ConvModule from a conv and a bn module."""
#         self = ConvModule.__new__(ConvModule)
#         super(ConvModule, self).__init__()

#         self.conv_cfg = None
#         self.norm_cfg = None
#         self.act_cfg = None
#         self.inplace = False
#         self.with_spectral_norm = False
#         self.with_explicit_padding = False
#         self.order = ('conv', 'norm', 'act')

#         self.with_norm = True
#         self.with_activation = False
#         self.with_bias = conv.bias is not None

#         # build convolution layer
#         self.conv = conv
#         # export the attributes of self.conv to a higher level for convenience
#         self.in_channels = self.conv.in_channels
#         self.out_channels = self.conv.out_channels
#         self.kernel_size = self.conv.kernel_size
#         self.stride = self.conv.stride
#         self.padding = self.conv.padding
#         self.dilation = self.conv.dilation
#         self.transposed = self.conv.transposed
#         self.output_padding = self.conv.output_padding
#         self.groups = self.conv.groups

#         # build normalization layers
#         self.norm_name, norm = 'bn', bn
#         self.add_module(self.norm_name, norm)

#         self.turn_on_efficient_conv_bn_eval(efficient_conv_bn_eval)

#         return self

# class BaseDecodeHead(BaseModule, metaclass=ABCMeta):
#     """Base class for BaseDecodeHead.

#     Args:
#         in_channels (int|Sequence[int]): Input channels.
#         channels (int): Channels after modules, before conv_seg.
#         num_classes (int): Number of classes.
#         dropout_ratio (float): Ratio of dropout layer. Default: 0.1.
#         conv_cfg (dict|None): Config of conv layers. Default: None.
#         norm_cfg (dict|None): Config of norm layers. Default: None.
#         act_cfg (dict): Config of activation layers.
#             Default: dict(type='ReLU')
#         in_index (int|Sequence[int]): Input feature index. Default: -1
#         input_transform (str|None): Transformation type of input features.
#             Options: 'resize_concat', 'multiple_select', None.
#             'resize_concat': Multiple feature maps will be resize to the
#                 same size as first one and than concat together.
#                 Usually used in FCN head of HRNet.
#             'multiple_select': Multiple feature maps will be bundle into
#                 a list and passed into decode head.
#             None: Only one select feature map is allowed.
#             Default: None.
#         loss_decode (dict): Config of decode loss.
#             Default: dict(type='CrossEntropyLoss').
#         ignore_index (int | None): The label index to be ignored. When using
#             masked BCE loss, ignore_index should be set to None. Default: 255
#         sampler (dict|None): The config of segmentation map sampler.
#             Default: None.
#         align_corners (bool): align_corners argument of F.interpolate.
#             Default: False.
#         init_cfg (dict or list[dict], optional): Initialization config dict.
#     """

#     def __init__(self,
#                  in_channels,
#                  channels,
#                  *,
#                  num_classes,
#                  dropout_ratio=0.1,
#                  conv_cfg=None,
#                  norm_cfg=None,
#                  act_cfg=dict(type='ReLU'),
#                  in_index=-1,
#                  input_transform=None,
#                  loss_decode=dict(
#                      type='CrossEntropyLoss',
#                      use_sigmoid=False,
#                      loss_weight=1.0),
#                  ignore_index=255,
#                  sampler=None,
#                  align_corners=False,
#                  init_cfg=dict(
#                      type='Normal', std=0.01, override=dict(name='conv_seg'))):
#         super(BaseDecodeHead, self).__init__(init_cfg)
#         self._init_inputs(in_channels, in_index, input_transform)
#         self.channels = channels
#         self.num_classes = num_classes
#         self.dropout_ratio = dropout_ratio
#         self.conv_cfg = conv_cfg
#         self.norm_cfg = norm_cfg
#         self.act_cfg = act_cfg
#         self.in_index = in_index
#         self.loss_decode = OhemCrossEntropy(
#                 thres=0.9,
#                 min_kept=131072,
#                 class_weight=class_weight,
#                 loss_weight=0.4)
#         self.ignore_index = ignore_index
#         self.align_corners = align_corners
#         self.sampler = None

#         self.conv_seg = nn.Conv2d(channels, num_classes, kernel_size=1)
#         if dropout_ratio > 0:
#             self.dropout = nn.Dropout2d(dropout_ratio)
#         else:
#             self.dropout = None
#         self.fp16_enabled = False

#     def extra_repr(self):
#         """Extra repr."""
#         s = f'input_transform={self.input_transform}, ' \
#             f'ignore_index={self.ignore_index}, ' \
#             f'align_corners={self.align_corners}'
#         return s

#     def _init_inputs(self, in_channels, in_index, input_transform):
#         """Check and initialize input transforms.

#         The in_channels, in_index and input_transform must match.
#         Specifically, when input_transform is None, only single feature map
#         will be selected. So in_channels and in_index must be of type int.
#         When input_transform

#         Args:
#             in_channels (int|Sequence[int]): Input channels.
#             in_index (int|Sequence[int]): Input feature index.
#             input_transform (str|None): Transformation type of input features.
#                 Options: 'resize_concat', 'multiple_select', None.
#                 'resize_concat': Multiple feature maps will be resize to the
#                     same size as first one and than concat together.
#                     Usually used in FCN head of HRNet.
#                 'multiple_select': Multiple feature maps will be bundle into
#                     a list and passed into decode head.
#                 None: Only one select feature map is allowed.
#         """

#         if input_transform is not None:
#             assert input_transform in ['resize_concat', 'multiple_select']
#         self.input_transform = input_transform
#         self.in_index = in_index
#         if input_transform is not None:
#             assert isinstance(in_channels, (list, tuple))
#             assert isinstance(in_index, (list, tuple))
#             assert len(in_channels) == len(in_index)
#             if input_transform == 'resize_concat':
#                 self.in_channels = sum(in_channels)
#             else:
#                 self.in_channels = in_channels
#         else:
#             assert isinstance(in_channels, int)
#             assert isinstance(in_index, int)
#             self.in_channels = in_channels

#     def _transform_inputs(self, inputs):
#         """Transform inputs for decoder.

#         Args:
#             inputs (list[Tensor]): List of multi-level img features.

#         Returns:
#             Tensor: The transformed inputs
#         """

#         if self.input_transform == 'resize_concat':
#             inputs = [inputs[i] for i in self.in_index]
#             upsampled_inputs = [
#                 resize(
#                     input=x,
#                     size=inputs[0].shape[2:],
#                     mode='bilinear',
#                     align_corners=self.align_corners) for x in inputs
#             ]
#             inputs = torch.cat(upsampled_inputs, dim=1)
#         elif self.input_transform == 'multiple_select':
#             inputs = [inputs[i] for i in self.in_index]
#         else:
#             inputs = inputs[self.in_index]

#         return inputs

#     @abstractmethod
#     def forward(self, inputs):
#         """Placeholder of forward function."""
#         pass

#     def forward_train(self, inputs, img_metas, gt_semantic_seg, train_cfg):
#         """Forward function for training.
#         Args:
#             inputs (list[Tensor]): List of multi-level img features.
#             img_metas (list[dict]): List of image info dict where each dict
#                 has: 'img_shape', 'scale_factor', 'flip', and may also contain
#                 'filename', 'ori_shape', 'pad_shape', and 'img_norm_cfg'.
#                 For details on the values of these keys see
#                 `mmseg/datasets/pipelines/formatting.py:Collect`.
#             gt_semantic_seg (Tensor): Semantic segmentation masks
#                 used if the architecture supports semantic segmentation task.
#             train_cfg (dict): The training config.

#         Returns:
#             dict[str, Tensor]: a dictionary of loss components
#         """
#         seg_logits = self.forward(inputs)
#         losses = self.losses(seg_logits, gt_semantic_seg)
#         return losses

#     def forward_test(self, inputs, img_metas, test_cfg):
#         """Forward function for testing.

#         Args:
#             inputs (list[Tensor]): List of multi-level img features.
#             img_metas (list[dict]): List of image info dict where each dict
#                 has: 'img_shape', 'scale_factor', 'flip', and may also contain
#                 'filename', 'ori_shape', 'pad_shape', and 'img_norm_cfg'.
#                 For details on the values of these keys see
#                 `mmseg/datasets/pipelines/formatting.py:Collect`.
#             test_cfg (dict): The testing config.

#         Returns:
#             Tensor: Output segmentation map.
#         """
#         return self.forward(inputs)

#     def cls_seg(self, feat):
#         """Classify each pixel."""
#         if self.dropout is not None:
#             feat = self.dropout(feat)
#         output = self.conv_seg(feat)
#         return output

#     def losses(self, seg_logit, seg_label):
#         with torch.amp.autocast('cuda', enabled=False):
#             """Compute segmentation loss."""
#             loss = dict()
#             seg_logit = resize(
#                 input=seg_logit,
#                 size=seg_label.shape[2:],
#                 mode='bilinear',
#                 align_corners=self.align_corners)
#             if self.sampler is not None:
#                 seg_weight = self.sampler.sample(seg_logit, seg_label)
#             else:
#                 seg_weight = None
#             seg_label = seg_label.squeeze(1)
#             loss['loss_seg'] = self.loss_decode(
#                 seg_logit,
#                 seg_label,
#                 weight=seg_weight,
#                 ignore_index=self.ignore_index)
#             loss['acc_seg'] = accuracy(seg_logit, seg_label)
#             return loss
    
# class FCNHead(BaseDecodeHead):
#     """Fully Convolution Networks for Semantic Segmentation.

#     This head is implemented of `FCNNet <https://arxiv.org/abs/1411.4038>`_.

#     Args:
#         num_convs (int): Number of convs in the head. Default: 2.
#         kernel_size (int): The kernel size for convs in the head. Default: 3.
#         concat_input (bool): Whether concat the input and output of convs
#             before classification layer.
#         dilation (int): The dilation rate for convs in the head. Default: 1.
#     """

#     def __init__(self,
#                  num_convs=2,
#                  kernel_size=3,
#                  concat_input=True,
#                  dilation=1,
#                  **kwargs):
#         assert num_convs >= 0 and dilation > 0 and isinstance(dilation, int)
#         self.num_convs = num_convs
#         self.concat_input = concat_input
#         self.kernel_size = kernel_size
#         super(FCNHead, self).__init__(**kwargs)
#         if num_convs == 0:
#             assert self.in_channels == self.channels

#         conv_padding = (kernel_size // 2) * dilation
#         convs = []
#         convs.append(
#             ConvModule(
#                 self.in_channels,
#                 self.channels,
#                 kernel_size=kernel_size,
#                 padding=conv_padding,
#                 dilation=dilation,
#                 conv_cfg=self.conv_cfg,
#                 norm_cfg=self.norm_cfg,
#                 act_cfg=self.act_cfg))
#         for i in range(num_convs - 1):
#             convs.append(
#                 ConvModule(
#                     self.channels,
#                     self.channels,
#                     kernel_size=kernel_size,
#                     padding=conv_padding,
#                     dilation=dilation,
#                     conv_cfg=self.conv_cfg,
#                     norm_cfg=self.norm_cfg,
#                     act_cfg=self.act_cfg))
#         if num_convs == 0:
#             self.convs = nn.Identity()
#         else:
#             self.convs = nn.Sequential(*convs)
#         if self.concat_input:
#             self.conv_cat = ConvModule(
#                 self.in_channels + self.channels,
#                 self.channels,
#                 kernel_size=kernel_size,
#                 padding=kernel_size // 2,
#                 conv_cfg=self.conv_cfg,
#                 norm_cfg=self.norm_cfg,
#                 act_cfg=self.act_cfg)

#     def forward(self, inputs):
#         """Forward function."""
#         x = self._transform_inputs(inputs)
#         output = self.convs(x)
#         if self.concat_input:
#             output = self.conv_cat(torch.cat([x, output], dim=1))
#         output = self.cls_seg(output)
#         return output


# def last_zero_init(m: Union[nn.Module, nn.Sequential]) -> None:
#     if isinstance(m, nn.Sequential):
#         constant_init(m[-1], val=0)
#     else:
#         constant_init(m, val=0)

# class ContextBlock(nn.Module):
#     """ContextBlock module in GCNet.

#     See 'GCNet: Non-local Networks Meet Squeeze-Excitation Networks and Beyond'
#     (https://arxiv.org/abs/1904.11492) for details.

#     Args:
#         in_channels (int): Channels of the input feature map.
#         ratio (float): Ratio of channels of transform bottleneck
#         pooling_type (str): Pooling method for context modeling.
#             Options are 'att' and 'avg', stand for attention pooling and
#             average pooling respectively. Default: 'att'.
#         fusion_types (Sequence[str]): Fusion method for feature fusion,
#             Options are 'channels_add', 'channel_mul', stand for channelwise
#             addition and multiplication respectively. Default: ('channel_add',)
#     """

#     _abbr_ = 'context_block'

#     def __init__(self,
#                  in_channels: int,
#                  ratio: float,
#                  pooling_type: str = 'att',
#                  fusion_types: tuple = ('channel_add', )):
#         super().__init__()
#         assert pooling_type in ['avg', 'att']
#         assert isinstance(fusion_types, (list, tuple))
#         valid_fusion_types = ['channel_add', 'channel_mul']
#         assert all([f in valid_fusion_types for f in fusion_types])
#         assert len(fusion_types) > 0, 'at least one fusion should be used'
#         self.in_channels = in_channels
#         self.ratio = ratio
#         self.planes = int(in_channels * ratio)
#         self.pooling_type = pooling_type
#         self.fusion_types = fusion_types
#         if pooling_type == 'att':
#             self.conv_mask = nn.Conv2d(in_channels, 1, kernel_size=1)
#             self.softmax = nn.Softmax(dim=2)
#         else:
#             self.avg_pool = nn.AdaptiveAvgPool2d(1)
#         if 'channel_add' in fusion_types:
#             self.channel_add_conv = nn.Sequential(
#                 nn.Conv2d(self.in_channels, self.planes, kernel_size=1),
#                 nn.LayerNorm([self.planes, 1, 1]),
#                 nn.ReLU(inplace=True),  # yapf: disable
#                 nn.Conv2d(self.planes, self.in_channels, kernel_size=1))
#         else:
#             self.channel_add_conv = None
#         if 'channel_mul' in fusion_types:
#             self.channel_mul_conv = nn.Sequential(
#                 nn.Conv2d(self.in_channels, self.planes, kernel_size=1),
#                 nn.LayerNorm([self.planes, 1, 1]),
#                 nn.ReLU(inplace=True),  # yapf: disable
#                 nn.Conv2d(self.planes, self.in_channels, kernel_size=1))
#         else:
#             self.channel_mul_conv = None
#         self.reset_parameters()

#     def reset_parameters(self):
#         if self.pooling_type == 'att':
#             kaiming_init(self.conv_mask, mode='fan_in')
#             self.conv_mask.inited = True

#         if self.channel_add_conv is not None:
#             last_zero_init(self.channel_add_conv)
#         if self.channel_mul_conv is not None:
#             last_zero_init(self.channel_mul_conv)

#     def spatial_pool(self, x: torch.Tensor) -> torch.Tensor:
#         batch, channel, height, width = x.size()
#         if self.pooling_type == 'att':
#             input_x = x
#             # [N, C, H * W]
#             input_x = input_x.view(batch, channel, height * width)
#             # [N, 1, C, H * W]
#             input_x = input_x.unsqueeze(1)
#             # [N, 1, H, W]
#             context_mask = self.conv_mask(x)
#             # [N, 1, H * W]
#             context_mask = context_mask.view(batch, 1, height * width)
#             # [N, 1, H * W]
#             context_mask = self.softmax(context_mask)
#             # [N, 1, H * W, 1]
#             context_mask = context_mask.unsqueeze(-1)
#             # [N, 1, C, 1]
#             context = torch.matmul(input_x, context_mask)
#             # [N, C, 1, 1]
#             context = context.view(batch, channel, 1, 1)
#         else:
#             # [N, C, 1, 1]
#             context = self.avg_pool(x)

#         return context

#     def forward(self, x: torch.Tensor) -> torch.Tensor:
#         # [N, C, 1, 1]
#         context = self.spatial_pool(x)

#         out = x
#         if self.channel_mul_conv is not None:
#             # [N, C, 1, 1]
#             channel_mul_term = torch.sigmoid(self.channel_mul_conv(context))
#             out = out * channel_mul_term
#         if self.channel_add_conv is not None:
#             # [N, C, 1, 1]
#             channel_add_term = self.channel_add_conv(context)
#             out = out + channel_add_term

#         return out
    
# class GCHead(FCNHead):
#     def __init__(self,
#                  ratio=1 / 4.,
#                  pooling_type='att',
#                  fusion_types=('channel_add', ),
#                  **kwargs):
#         super(GCHead, self).__init__(num_convs=2, **kwargs)
#         self.ratio = ratio
#         self.pooling_type = pooling_type
#         self.fusion_types = fusion_types
#         self.gc_block = ContextBlock(
#             in_channels=self.channels,
#             ratio=self.ratio,
#             pooling_type=self.pooling_type,
#             fusion_types=self.fusion_types)

#     def forward(self, inputs):
#         """Forward function."""
#         x = self._transform_inputs(inputs)
#         output = self.convs[0](x)
#         output = self.gc_block(output)
#         output = self.convs[1](output)
#         if self.concat_input:
#             output = self.conv_cat(torch.cat([x, output], dim=1))
#         output = self.cls_seg(output)
#         return output

# class GCNet(BaseModule):
#     """GCNet backbone.

#     Args:
#         in_channels (int): Number of input image channels. Default: 3
#         channels: (int): The base channels of GCNet. Default: 32
#         ppm_channels (int): The channels of PPM module. Default: 128
#         num_blocks_per_stage (List[int]): The number of blocks with a
#             stride of 1 from stage 2 to stage 6. '[4, 4, [5, 4], [5, 4], [2, 2]]'
#             corresponding GCNet-S and GCNet-M,
#             '[5, 5, [5, 5], [5, 5], [3, 3]]' corresponding GCNet-L.
#         align_corners (bool): align_corners argument of F.interpolate.
#             Default: False
#         norm_cfg (dict): Config dict to build norm layer.
#             Default: dict(type='BN', requires_grad=True)
#         act_cfg (dict): Config dict for activation layer.
#             Default: dict(type='ReLU', inplace=True)
#         init_cfg (dict, optional): Initialization config dict.
#             Default: None
#         deploy (bool): Whether in deploy mode. Default: False
#     """

#     def __init__(self,
#                  in_channels: int = 3,
#                  channels: int = 32,
#                  ppm_channels: int = 128,
#                  num_blocks_per_stage: List[int] = [4, 4, [5, 4], [5, 4], [2, 2]],
#                  align_corners: bool = False,
#                  norm_cfg = dict(type='BN', requires_grad=True),
#                  act_cfg = dict(type='ReLU', inplace=True),
#                  init_cfg = None,
#                  deploy: bool = False):
#         super().__init__(init_cfg)

#         self.in_channels = in_channels
#         self.channels = channels
#         self.ppm_channels = ppm_channels
#         self.num_blocks_per_stage = num_blocks_per_stage
#         self.align_corners = align_corners
#         self.norm_cfg = norm_cfg
#         self.act_cfg = act_cfg
#         self.deploy = deploy

#         # stage 1-3
#         self.stem = nn.Sequential(
#             # stage1
#             ConvModule(
#                 in_channels=in_channels,
#                 out_channels=channels,
#                 kernel_size=3,
#                 stride=2,
#                 padding=1,
#                 norm_cfg=norm_cfg,
#                 act_cfg=act_cfg),
#             ConvModule(
#                 in_channels=channels,
#                 out_channels=channels,
#                 kernel_size=3,
#                 stride=2,
#                 padding=1,
#                 norm_cfg=norm_cfg,
#                 act_cfg=act_cfg),

#             # stage2
#             *[GCBlock(in_channels=channels, out_channels=channels, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg, deploy=deploy) for
#               _ in range(num_blocks_per_stage[0])],

#             # stage3
#             GCBlock(in_channels=channels, out_channels=channels * 2, stride=2, norm_cfg=norm_cfg, act_cfg=act_cfg, deploy=deploy),
#             *[GCBlock(in_channels=channels * 2, out_channels=channels * 2, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                  deploy=deploy) for _ in range(num_blocks_per_stage[1] - 1)],
#         )

#         self.relu = nn.ReLU(inplace=True)

#         # semantic branch
#         self.semantic_branch_layers = nn.ModuleList()
#         self.semantic_branch_layers.append(
#             nn.Sequential(
#                 GCBlock(in_channels=channels * 2, out_channels=channels * 4, stride=2, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                    deploy=deploy),
#                 *[GCBlock(in_channels=channels * 4, out_channels=channels * 4, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                      deploy=deploy) for _ in range(num_blocks_per_stage[2][0]- 2)],
#                 GCBlock(in_channels=channels * 4, out_channels=channels * 4, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg, act=False,
#                    deploy=deploy),
#             )
#         )
#         self.semantic_branch_layers.append(
#             nn.Sequential(
#                 GCBlock(in_channels=channels * 4, out_channels=channels * 8, stride=2, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                    deploy=deploy),
#                 *[GCBlock(in_channels=channels * 8, out_channels=channels * 8, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                      deploy=deploy) for _ in range(num_blocks_per_stage[3][0] - 2)],
#                 GCBlock(in_channels=channels * 8, out_channels=channels * 8, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg, act=False,
#                    deploy=deploy),
#             )
#         )
#         self.semantic_branch_layers.append(
#             nn.Sequential(
#                 GCBlock(in_channels=channels * 8, out_channels=channels * 16, stride=2, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                    deploy=deploy),
#                 *[GCBlock(in_channels=channels * 16, out_channels=channels * 16, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                    deploy=deploy) for _ in range(num_blocks_per_stage[4][0] - 2)],
#                 GCBlock(in_channels=channels * 16, out_channels=channels * 16, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg, act=False,
#                    deploy=deploy)
#             )
#         )

#         # bilateral fusion
#         self.compression_1 = ConvModule(
#             channels * 4,
#             channels * 2,
#             kernel_size=1,
#             norm_cfg=norm_cfg,
#             act_cfg=None)
#         self.down_1 = ConvModule(
#             channels * 2,
#             channels * 4,
#             kernel_size=3,
#             stride=2,
#             padding=1,
#             norm_cfg=norm_cfg,
#             act_cfg=None)

#         self.compression_2 = ConvModule(
#             channels * 8,
#             channels * 2,
#             kernel_size=1,
#             norm_cfg=norm_cfg,
#             act_cfg=None)
#         self.down_2 = nn.Sequential(
#             ConvModule(
#                 channels * 2,
#                 channels * 4,
#                 kernel_size=3,
#                 stride=2,
#                 padding=1,
#                 norm_cfg=norm_cfg,
#                 act_cfg=act_cfg),
#             ConvModule(
#                 channels * 4,
#                 channels * 8,
#                 kernel_size=3,
#                 stride=2,
#                 padding=1,
#                 norm_cfg=norm_cfg,
#                 act_cfg=None))

#         # detail branch
#         self.detail_branch_layers = nn.ModuleList()
#         self.detail_branch_layers.append(
#             nn.Sequential(
#                 *[GCBlock(in_channels=channels * 2, out_channels=channels * 2, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                      deploy=deploy) for _ in range(num_blocks_per_stage[2][1] - 1)],
#                 GCBlock(in_channels=channels * 2, out_channels=channels * 2, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg, act=False,
#                    deploy=deploy),
#             )
#         )
#         self.detail_branch_layers.append(
#             nn.Sequential(
#                 *[GCBlock(in_channels=channels * 2, out_channels=channels * 2, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                      deploy=deploy) for _ in range(num_blocks_per_stage[3][1] - 1)],
#                 GCBlock(in_channels=channels * 2, out_channels=channels * 2, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg, act=False,
#                    deploy=deploy),
#             )
#         )
#         self.detail_branch_layers.append(
#             nn.Sequential(
#                 GCBlock(in_channels=channels * 2, out_channels=channels * 4, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                    deploy=deploy),
#                 *[GCBlock(in_channels=channels * 4, out_channels=channels * 4, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg,
#                    deploy=deploy) for _ in range(num_blocks_per_stage[4][1] - 2)],
#                 GCBlock(in_channels=channels * 4, out_channels=channels * 4, stride=1, norm_cfg=norm_cfg, act_cfg=act_cfg, act=False,
#                    deploy=deploy),
#             )
#         )

#         self.spp = DAPPM(
#             channels * 16, ppm_channels, channels * 4, num_scales=5, norm_cfg=norm_cfg, act_cfg=act_cfg)

#         self.kaiming_init()

#     def forward(self, x):
#         """Forward function."""
#         out_size = (math.ceil(x.shape[-2] / 8), math.ceil(x.shape[-1] / 8))

#         # stage 1-3
#         x = self.stem(x)

#         # stage4
#         x_s = self.semantic_branch_layers[0](x) #param4
#         x_d = self.detail_branch_layers[0](x) #param3
#         comp_c = self.compression_1(self.relu(x_s)) #param3
#         x_s = x_s + self.down_1(self.relu(x_d))
#         x_d = x_d + resize(comp_c,
#                            size=out_size,
#                            mode='bilinear',
#                            align_corners=self.align_corners)
#         if self.training:
#             c4_feat = x_d.clone()

#         # stage5
#         x_s = self.semantic_branch_layers[1](self.relu(x_s))
#         x_d = self.detail_branch_layers[1](self.relu(x_d))
#         comp_c = self.compression_2(self.relu(x_s))
#         x_s = x_s + self.down_2(self.relu(x_d))
#         x_d = x_d + resize(comp_c,
#                            size=out_size,
#                            mode='bilinear',
#                            align_corners=self.align_corners)

#         # stage6
#         x_d = self.detail_branch_layers[2](self.relu(x_d))
#         x_s = self.semantic_branch_layers[2](self.relu(x_s))
#         x_s = self.spp(x_s)
#         x_s = resize(
#             x_s,
#             size=out_size,
#             mode='bilinear',
#             align_corners=self.align_corners)

#         return (c4_feat, x_d + x_s) if self.training else x_d + x_s

#     def switch_to_deploy(self):
#         for m in self.modules():
#             if isinstance(m, GCBlock):
#                 m.switch_to_deploy()
#         self.deploy = True

#     def kaiming_init(self):
#         for m in self.modules():
#             if isinstance(m, nn.Conv2d):
#                 nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
#                 if m.bias is not None:
#                     nn.init.constant_(m.bias, 0)
#             elif isinstance(m, nn.BatchNorm2d):
#                 nn.init.constant_(m.weight, 1)
#                 nn.init.constant_(m.bias, 0)


# class Block1x1(BaseModule):
#     """The 1x1_1x1 path of the GCBlock.

#         Args:
#             in_channels (int): Number of channels in the input image
#             out_channels (int): Number of channels produced by the convolution
#             stride (int or tuple): Stride of the convolution. Default: 1
#             padding (int, tuple): Padding added to all four sides of
#                 the input. Default: 1
#             bias (bool) : Whether to use bias.
#                 Default: True
#             norm_cfg (dict): Config dict to build norm layer.
#                 Default: dict(type='BN', requires_grad=True)
#             deploy (bool): Whether in deploy mode. Default: False
#     """

#     def __init__(self,
#                  in_channels: int,
#                  out_channels: int,
#                  stride: Union[int, Tuple[int]] = 1,
#                  padding: Union[int, Tuple[int]] = 0,
#                  bias: bool = True,
#                  norm_cfg = dict(type='BN', requires_grad=True),
#                  deploy: bool = False):
#         super().__init__()

#         self.in_channels = in_channels
#         self.out_channels = out_channels
#         self.stride = stride
#         self.padding = padding
#         self.bias = bias
#         self.deploy = deploy

#         if self.deploy:
#             self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=stride, padding=padding, bias=True)
#         else:
#             self.conv1 = ConvModule(
#                 in_channels=in_channels,
#                 out_channels=out_channels,
#                 kernel_size=1,
#                 stride=stride,
#                 padding=padding,
#                 bias=bias,
#                 norm_cfg=norm_cfg,
#                 act_cfg=None)
#             self.conv2 = ConvModule(
#                 in_channels=out_channels,
#                 out_channels=out_channels,
#                 kernel_size=1,
#                 stride=1,
#                 padding=padding,
#                 bias=bias,
#                 norm_cfg=norm_cfg,
#                 act_cfg=None)

#     def forward(self, x):
#         if self.deploy:
#             x = self.conv(x)
#         else:
#             x = self.conv1(x)
#             x = self.conv2(x)

#         return x

#     def _fuse_bn_tensor(self, conv: nn.Module):
#         kernel = conv.conv.weight
#         bias = conv.conv.bias
#         running_mean = conv.bn.running_mean
#         running_var = conv.bn.running_var
#         gamma = conv.bn.weight
#         beta = conv.bn.bias
#         eps = conv.bn.eps
#         std = (running_var + eps).sqrt()
#         t = (gamma / std).reshape(-1, 1, 1, 1)
#         return kernel * t, beta + (bias - running_mean) * gamma / std if self.bias else beta - running_mean * gamma / std

#     def switch_to_deploy(self):
#         kernel1, bias1 = self._fuse_bn_tensor(self.conv1)
#         kernel2, bias2 = self._fuse_bn_tensor(self.conv2)
#         self.conv = nn.Conv2d(self.in_channels, self.out_channels, kernel_size=1, stride=self.stride, padding=self.padding, bias=True)
#         self.conv.weight.data = torch.einsum('oi,icjk->ocjk', kernel2.squeeze(3).squeeze(2), kernel1)
#         self.conv.bias.data = bias2 + (bias1.view(1, -1, 1, 1) * kernel2).sum(3).sum(2).sum(1)
#         self.__delattr__('conv1')
#         self.__delattr__('conv2')
#         self.deploy = True


# class Block3x3(BaseModule):
#     """The 3x3_1x1 path of the GCBlock.

#         Args:
#             in_channels (int): Number of channels in the input image
#             out_channels (int): Number of channels produced by the convolution
#             stride (int or tuple): Stride of the convolution. Default: 1
#             padding (int, tuple): Padding added to all four sides of
#                 the input. Default: 1
#             bias (bool) : Whether to use bias.
#                 Default: True
#             norm_cfg (dict): Config dict to build norm layer.
#                 Default: dict(type='BN', requires_grad=True)
#             deploy (bool): Whether in deploy mode. Default: False
#     """

#     def __init__(self,
#                  in_channels: int,
#                  out_channels: int,
#                  stride: Union[int, Tuple[int]] = 1,
#                  padding: Union[int, Tuple[int]] = 0,
#                  bias: bool = True,
#                  norm_cfg = dict(type='BN', requires_grad=True),
#                  deploy: bool = False):
#         super().__init__()

#         self.in_channels = in_channels
#         self.out_channels = out_channels
#         self.stride = stride
#         self.padding = padding
#         self.bias = bias
#         self.deploy = deploy

#         if self.deploy:
#             self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=padding, bias=True)
#         else:
#             self.conv1 = ConvModule(
#                 in_channels=in_channels,
#                 out_channels=out_channels,
#                 kernel_size=3,
#                 stride=stride,
#                 padding=padding,
#                 bias=bias,
#                 norm_cfg=norm_cfg,
#                 act_cfg=None)
#             self.conv2 = ConvModule(
#                 in_channels=out_channels,
#                 out_channels=out_channels,
#                 kernel_size=1,
#                 stride=1,
#                 padding=0,
#                 bias=bias,
#                 norm_cfg=norm_cfg,
#                 act_cfg=None)

#     def forward(self, x):
#         if self.deploy:
#             x = self.conv(x)
#         else:
#             x = self.conv1(x)
#             x = self.conv2(x)

#         return x

#     def _fuse_bn_tensor(self, conv: nn.Module):
#         kernel = conv.conv.weight
#         bias = conv.conv.bias
#         running_mean = conv.bn.running_mean
#         running_var = conv.bn.running_var
#         gamma = conv.bn.weight
#         beta = conv.bn.bias
#         eps = conv.bn.eps
#         std = (running_var + eps).sqrt()
#         t = (gamma / std).reshape(-1, 1, 1, 1)
#         return kernel * t, beta + (bias - running_mean) * gamma / std if self.bias else beta - running_mean * gamma / std

#     def switch_to_deploy(self):
#         kernel1, bias1 = self._fuse_bn_tensor(self.conv1)
#         kernel2, bias2 = self._fuse_bn_tensor(self.conv2)
#         self.conv = nn.Conv2d(self.in_channels, self.out_channels, kernel_size=3, stride=self.stride,
#                               padding=self.padding, bias=True)

#         self.conv.weight.data = torch.einsum('oi,icjk->ocjk', kernel2.squeeze(3).squeeze(2), kernel1)
#         self.conv.bias.data = bias2 + (bias1.view(1, -1, 1, 1) * kernel2).sum(3).sum(2).sum(1)

#         self.__delattr__('conv1')
#         self.__delattr__('conv2')
#         self.deploy = True

# class DAPPM(BaseModule):
#     """DAPPM module in `DDRNet <https://arxiv.org/abs/2101.06085>`_.

#     Args:
#         in_channels (int): Input channels.
#         branch_channels (int): Branch channels.
#         out_channels (int): Output channels.
#         num_scales (int): Number of scales.
#         kernel_sizes (list[int]): Kernel sizes of each scale.
#         strides (list[int]): Strides of each scale.
#         paddings (list[int]): Paddings of each scale.
#         norm_cfg (dict): Config dict for normalization layer.
#             Default: dict(type='BN').
#         act_cfg (dict): Config dict for activation layer in ConvModule.
#             Default: dict(type='ReLU', inplace=True).
#         conv_cfg (dict): Config dict for convolution layer in ConvModule.
#             Default: dict(order=('norm', 'act', 'conv'), bias=False).
#         upsample_mode (str): Upsample mode. Default: 'bilinear'.
#     """

#     def __init__(self,
#                  in_channels: int,
#                  branch_channels: int,
#                  out_channels: int,
#                  num_scales: int,
#                  kernel_sizes: List[int] = [5, 9, 17],
#                  strides: List[int] = [2, 4, 8],
#                  paddings: List[int] = [2, 4, 8],
#                  norm_cfg: Dict = dict(type='BN', momentum=0.1),
#                  act_cfg: Dict = dict(type='ReLU', inplace=True),
#                  conv_cfg: Dict = dict(
#                      order=('norm', 'act', 'conv'), bias=False),
#                  upsample_mode: str = 'bilinear'):
#         super().__init__()

#         self.num_scales = num_scales
#         self.unsample_mode = upsample_mode
#         self.in_channels = in_channels
#         self.branch_channels = branch_channels
#         self.out_channels = out_channels
#         self.norm_cfg = norm_cfg
#         self.act_cfg = act_cfg
#         self.conv_cfg = conv_cfg

#         self.scales = ModuleList([
#             ConvModule(
#                 in_channels,
#                 branch_channels,
#                 kernel_size=1,
#                 norm_cfg=norm_cfg,
#                 act_cfg=act_cfg,
#                 **conv_cfg)
#         ])
#         for i in range(1, num_scales - 1):
#             self.scales.append(
#                 Sequential(*[
#                     nn.AvgPool2d(
#                         kernel_size=kernel_sizes[i - 1],
#                         stride=strides[i - 1],
#                         padding=paddings[i - 1]),
#                     ConvModule(
#                         in_channels,
#                         branch_channels,
#                         kernel_size=1,
#                         norm_cfg=norm_cfg,
#                         act_cfg=act_cfg,
#                         **conv_cfg)
#                 ]))
#         self.scales.append(
#             Sequential(*[
#                 nn.AdaptiveAvgPool2d((1, 1)),
#                 ConvModule(
#                     in_channels,
#                     branch_channels,
#                     kernel_size=1,
#                     norm_cfg=norm_cfg,
#                     act_cfg=act_cfg,
#                     **conv_cfg)
#             ]))
#         self.processes = ModuleList()
#         for i in range(num_scales - 1):
#             self.processes.append(
#                 ConvModule(
#                     branch_channels,
#                     branch_channels,
#                     kernel_size=3,
#                     padding=1,
#                     norm_cfg=norm_cfg,
#                     act_cfg=act_cfg,
#                     **conv_cfg))

#         self.compression = ConvModule(
#             branch_channels * num_scales,
#             out_channels,
#             kernel_size=1,
#             norm_cfg=norm_cfg,
#             act_cfg=act_cfg,
#             **conv_cfg)

#         self.shortcut = ConvModule(
#             in_channels,
#             out_channels,
#             kernel_size=1,
#             norm_cfg=norm_cfg,
#             act_cfg=act_cfg,
#             **conv_cfg)

#     def forward(self, inputs: Tensor):
#         feats = []
#         feats.append(self.scales[0](inputs))

#         for i in range(1, self.num_scales):
#             feat_up = F.interpolate(
#                 self.scales[i](inputs),
#                 size=inputs.shape[2:],
#                 mode=self.unsample_mode)
#             feats.append(self.processes[i - 1](feat_up + feats[i - 1]))

#         return self.compression(torch.cat(feats,
#                                           dim=1)) + self.shortcut(inputs)

# class GCBlock(nn.Module):
#     """GCBlock.

#     Args:
#         in_channels (int): Number of channels in the input image
#         out_channels (int): Number of channels produced by the convolution
#         kernel_size (int or tuple): Size of the convolving kernel
#         stride (int or tuple): Stride of the convolution. Default: 1
#         padding (int, tuple): Padding added to all four sides of
#             the input. Default: 1
#         padding_mode (string, optional): Default: 'zeros'
#         norm_cfg (dict): Config dict to build norm layer.
#             Default: dict(type='BN', requires_grad=True)
#         act (bool) : Whether to use activation function.
#             Default: False
#         deploy (bool): Whether in deploy mode. Default: False
#     """

#     def __init__(self,
#                  in_channels: int,
#                  out_channels: int,
#                  kernel_size: Union[int, Tuple[int]] = 3,
#                  stride: Union[int, Tuple[int]] = 1,
#                  padding: Union[int, Tuple[int]] = 1,
#                  padding_mode: Optional[str] = 'zeros',
#                  norm_cfg = dict(type='BN', requires_grad=True),
#                  act_cfg = dict(type='ReLU', inplace=True),
#                  act: bool = True,
#                  deploy: bool = False):
#         super().__init__()

#         self.in_channels = in_channels
#         self.out_channels = out_channels
#         self.kernel_size = kernel_size
#         self.stride = stride
#         self.padding = padding
#         self.deploy = deploy

#         assert kernel_size == 3
#         assert padding == 1

#         padding_11 = padding - kernel_size // 2

#         if act:
#             self.relu = nn.ReLU(inplace=True)
#         else:
#             self.relu = nn.Identity()

#         if deploy:
#             self.reparam_3x3 = nn.Conv2d(
#                 in_channels=in_channels,
#                 out_channels=out_channels,
#                 kernel_size=kernel_size,
#                 stride=stride,
#                 padding=padding,
#                 bias=True,
#                 padding_mode=padding_mode)

#         else:
#             if (out_channels == in_channels) and stride == 1:
#                 self.path_residual = nn.BatchNorm2d(num_features=in_channels, affine=True)[1]
#             else:
#                 self.path_residual = None

#             self.path_3x3_1 = Block3x3(
#                 in_channels=in_channels,
#                 out_channels=out_channels,
#                 stride=stride,
#                 padding=padding,
#                 bias=False,
#                 norm_cfg=norm_cfg,
#             )
#             self.path_3x3_2 = Block3x3(
#                 in_channels=in_channels,
#                 out_channels=out_channels,
#                 stride=stride,
#                 padding=padding,
#                 bias=False,
#                 norm_cfg=norm_cfg,
#             )
#             self.path_1x1 = Block1x1(
#                 in_channels=in_channels,
#                 out_channels=out_channels,
#                 stride=stride,
#                 padding=padding_11,
#                 bias=False,
#                 norm_cfg=norm_cfg,
#             )

#     def forward(self, inputs: Tensor) -> Tensor:

#         if hasattr(self, 'reparam_3x3'):
#             return self.relu(self.reparam_3x3(inputs))

#         if self.path_residual is None:
#             id_out = 0
#         else:
#             id_out = self.path_residual(inputs)

#         return self.relu(self.path_3x3_1(inputs) + self.path_3x3_2(inputs) + self.path_1x1(inputs) + id_out)

#     def get_equivalent_kernel_bias(self):
#         """Derives the equivalent kernel and bias in a differentiable way.

#         Returns:
#             tuple: Equivalent kernel and bias
#         """
#         self.path_3x3_1.switch_to_deploy()
#         kernel3x3_1, bias3x3_1 = self.path_3x3_1.conv.weight.data, self.path_3x3_1.conv.bias.data
#         self.path_3x3_2.switch_to_deploy()
#         kernel3x3_2, bias3x3_2 = self.path_3x3_2.conv.weight.data, self.path_3x3_2.conv.bias.data
#         self.path_1x1.switch_to_deploy()
#         kernel1x1, bias1x1 = self.path_1x1.conv.weight.data, self.path_1x1.conv.bias.data
#         kernelid, biasid = self._fuse_bn_tensor(self.path_residual)

#         return kernel3x3_1 + kernel3x3_2 + self._pad_1x1_to_3x3_tensor(kernel1x1) + kernelid, bias3x3_1 + bias3x3_2 + bias1x1 + biasid

#     def _pad_1x1_to_3x3_tensor(self, kernel1x1):
#         """Pad 1x1 tensor to 3x3.
#         Args:
#             kernel1x1 (Tensor): The input 1x1 kernel need to be padded.

#         Returns:
#             Tensor: 3x3 kernel after padded.
#         """
#         if kernel1x1 is None:
#             return 0
#         else:
#             return torch.nn.functional.pad(kernel1x1, [1, 1, 1, 1])

#     def _fuse_bn_tensor(self, conv: nn.Module) -> Tuple[np.ndarray, Tensor]:
#         """Derives the equivalent kernel and bias of a specific conv layer.

#         Args:
#             conv (nn.Module): The layer that needs to be equivalently
#                 transformed, which can be nn.Sequential or nn.Batchnorm2d

#         Returns:
#             tuple: Equivalent kernel and bias
#         """
#         if conv is None:
#             return 0, 0
#         if isinstance(conv, ConvModule):
#             kernel = conv.conv.weight
#             running_mean = conv.bn.running_mean
#             running_var = conv.bn.running_var
#             gamma = conv.bn.weight
#             beta = conv.bn.bias
#             eps = conv.bn.eps
#         else:
#             assert isinstance(conv, (nn.SyncBatchNorm, nn.BatchNorm2d, _BatchNormXd))
#             if not hasattr(self, 'id_tensor'):
#                 input_in_channels = self.in_channels
#                 kernel_value = np.zeros((self.in_channels, input_in_channels, 3, 3),
#                                         dtype=np.float32)
#                 for i in range(self.in_channels):
#                     kernel_value[i, i % input_in_channels, 1, 1] = 1
#                 self.id_tensor = torch.from_numpy(kernel_value).to(
#                     conv.weight.device)
#             kernel = self.id_tensor
#             running_mean = conv.running_mean
#             running_var = conv.running_var
#             gamma = conv.weight
#             beta = conv.bias
#             eps = conv.eps
#         std = (running_var + eps).sqrt()
#         t = (gamma / std).reshape(-1, 1, 1, 1)
#         return kernel * t, beta - running_mean * gamma / std

#     def switch_to_deploy(self):
#         """Switch to deploy mode."""
#         if hasattr(self, 'reparam_3x3'):
#             return
#         kernel, bias = self.get_equivalent_kernel_bias()
#         self.reparam_3x3 = nn.Conv2d(
#             in_channels=self.in_channels,
#             out_channels=self.out_channels,
#             kernel_size=self.kernel_size,
#             stride=self.stride,
#             padding=self.padding,
#             bias=True)
#         self.reparam_3x3.weight.data = kernel
#         self.reparam_3x3.bias.data = bias
#         for para in self.parameters():
#             para.detach_()
#         self.__delattr__('path_3x3_1')
#         self.__delattr__('path_3x3_2')
#         self.__delattr__('path_1x1')
#         if hasattr(self, 'path_residual'):
#             self.__delattr__('path_residual')
#         if hasattr(self, 'id_tensor'):
#             self.__delattr__('id_tensor')
#         self.deploy = True

# class GCHead(FCNHead):
#     """GCNet: Non-local Networks Meet Squeeze-Excitation Networks and Beyond.

#     This head is the implementation of `GCNet
#     <https://arxiv.org/abs/1904.11492>`_.

#     Args:
#         ratio (float): Multiplier of channels ratio. Default: 1/4.
#         pooling_type (str): The pooling type of context aggregation.
#             Options are 'att', 'avg'. Default: 'avg'.
#         fusion_types (tuple[str]): The fusion type for feature fusion.
#             Options are 'channel_add', 'channel_mul'. Default: ('channel_add',)
#     """

#     def __init__(self,
#                  ratio=1 / 4.,
#                  pooling_type='att',
#                  fusion_types=('channel_add', ),
#                  **kwargs):
#         super().__init__(num_convs=2, **kwargs)
#         self.ratio = ratio
#         self.pooling_type = pooling_type
#         self.fusion_types = fusion_types
#         self.gc_block = ContextBlock(
#             in_channels=self.channels,
#             ratio=self.ratio,
#             pooling_type=self.pooling_type,
#             fusion_types=self.fusion_types)

#     def forward(self, inputs):
#         """Forward function."""
#         x = self._transform_inputs(inputs)
#         output = self.convs[0](x)
#         output = self.gc_block(output)
#         output = self.convs[1](output)
#         if self.concat_input:
#             output = self.conv_cat(torch.cat([x, output], dim=1))
#         output = self.cls_seg(output)
#         return output

# class BaseModel(BaseModule):
#     """Base class for all algorithmic models.

#     BaseModel implements the basic functions of the algorithmic model, such as
#     weights initialize, batch inputs preprocess(see more information in
#     :class:`BaseDataPreprocessor`), parse losses, and update model parameters.

#     Subclasses inherit from BaseModel only need to implement the forward
#     method, which implements the logic to calculate loss and predictions,
#     then can be trained in the runner.

#     Examples:
#         >>> @MODELS.register_module()
#         >>> class ToyModel(BaseModel):
#         >>>
#         >>>     def __init__(self):
#         >>>         super().__init__()
#         >>>         self.backbone = nn.Sequential()
#         >>>         self.backbone.add_module('conv1', nn.Conv2d(3, 6, 5))
#         >>>         self.backbone.add_module('pool', nn.MaxPool2d(2, 2))
#         >>>         self.backbone.add_module('conv2', nn.Conv2d(6, 16, 5))
#         >>>         self.backbone.add_module('fc1', nn.Linear(16 * 5 * 5, 120))
#         >>>         self.backbone.add_module('fc2', nn.Linear(120, 84))
#         >>>         self.backbone.add_module('fc3', nn.Linear(84, 10))
#         >>>
#         >>>         self.criterion = nn.CrossEntropyLoss()
#         >>>
#         >>>     def forward(self, batch_inputs, data_samples, mode='tensor'):
#         >>>         data_samples = torch.stack(data_samples)
#         >>>         if mode == 'tensor':
#         >>>             return self.backbone(batch_inputs)
#         >>>         elif mode == 'predict':
#         >>>             feats = self.backbone(batch_inputs)
#         >>>             predictions = torch.argmax(feats, 1)
#         >>>             return predictions
#         >>>         elif mode == 'loss':
#         >>>             feats = self.backbone(batch_inputs)
#         >>>             loss = self.criterion(feats, data_samples)
#         >>>             return dict(loss=loss)

#     Args:
#         data_preprocessor (dict, optional): The pre-process config of
#             :class:`BaseDataPreprocessor`.
#         init_cfg (dict, optional): The weight initialized config for
#             :class:`BaseModule`.

#     Attributes:
#         data_preprocessor (:obj:`BaseDataPreprocessor`): Used for
#             pre-processing data sampled by dataloader to the format accepted by
#             :meth:`forward`.
#         init_cfg (dict, optional): Initialization config dict.
#     """

#     def __init__(self,
#                  data_preprocessor: Optional[Union[dict, nn.Module]] = None,
#                  init_cfg: Optional[dict] = None):
#         super().__init__(init_cfg)
#         if data_preprocessor is None:
#             data_preprocessor = dict(type='BaseDataPreprocessor')
#         if isinstance(data_preprocessor, nn.Module):
#             self.data_preprocessor = data_preprocessor
#         elif isinstance(data_preprocessor, dict):
#             self.data_preprocessor = MODELS.build(data_preprocessor)
#         else:
#             raise TypeError('data_preprocessor should be a `dict` or '
#                             f'`nn.Module` instance, but got '
#                             f'{type(data_preprocessor)}')

#     def train_step(self, data: Union[dict, tuple, list],
#                    optim_wrapper) -> Dict[str, torch.Tensor]:
#         """Implements the default model training process including
#         preprocessing, model forward propagation, loss calculation,
#         optimization, and back-propagation.

#         During non-distributed training. If subclasses do not override the
#         :meth:`train_step`, :class:`EpochBasedTrainLoop` or
#         :class:`IterBasedTrainLoop` will call this method to update model
#         parameters. The default parameter update process is as follows:

#         1. Calls ``self.data_processor(data, training=False)`` to collect
#            batch_inputs and corresponding data_samples(labels).
#         2. Calls ``self(batch_inputs, data_samples, mode='loss')`` to get raw
#            loss
#         3. Calls ``self.parse_losses`` to get ``parsed_losses`` tensor used to
#            backward and dict of loss tensor used to log messages.
#         4. Calls ``optim_wrapper.update_params(loss)`` to update model.

#         Args:
#             data (dict or tuple or list): Data sampled from dataset.
#             optim_wrapper (OptimWrapper): OptimWrapper instance
#                 used to update model parameters.

#         Returns:
#             Dict[str, torch.Tensor]: A ``dict`` of tensor for logging.
#         """
#         # Enable automatic mixed precision training context.
#         with optim_wrapper.optim_context(self):
#             data = self.data_preprocessor(data, True)
#             losses = self._run_forward(data, mode='loss')  # type: ignore
#         parsed_losses, log_vars = self.parse_losses(losses)  # type: ignore
#         optim_wrapper.update_params(parsed_losses)
#         return log_vars

#     def val_step(self, data: Union[tuple, dict, list]) -> list:
#         """Gets the predictions of given data.

#         Calls ``self.data_preprocessor(data, False)`` and
#         ``self(inputs, data_sample, mode='predict')`` in order. Return the
#         predictions which will be passed to evaluator.

#         Args:
#             data (dict or tuple or list): Data sampled from dataset.

#         Returns:
#             list: The predictions of given data.
#         """
#         data = self.data_preprocessor(data, False)
#         return self._run_forward(data, mode='predict')  # type: ignore

#     def test_step(self, data: Union[dict, tuple, list]) -> list:
#         """``BaseModel`` implements ``test_step`` the same as ``val_step``.

#         Args:
#             data (dict or tuple or list): Data sampled from dataset.

#         Returns:
#             list: The predictions of given data.
#         """
#         data = self.data_preprocessor(data, False)
#         return self._run_forward(data, mode='predict')  # type: ignore

#     def parse_losses(
#         self, losses: Dict[str, torch.Tensor]
#     ) -> Tuple[torch.Tensor, Dict[str, torch.Tensor]]:
#         """Parses the raw outputs (losses) of the network.

#         Args:
#             losses (dict): Raw output of the network, which usually contain
#                 losses and other necessary information.

#         Returns:
#             tuple[Tensor, dict]: There are two elements. The first is the
#             loss tensor passed to optim_wrapper which may be a weighted sum
#             of all losses, and the second is log_vars which will be sent to
#             the logger.
#         """
#         log_vars = []
#         for loss_name, loss_value in losses.items():
#             if isinstance(loss_value, torch.Tensor):
#                 log_vars.append([loss_name, loss_value.mean()])
#             else:
#                 log_vars.append(
#                     [loss_name,
#                      sum(_loss.mean() for _loss in loss_value)])

#         loss = sum(value for key, value in log_vars if 'loss' in key)
#         log_vars.insert(0, ['loss', loss])
#         log_vars = OrderedDict(log_vars)  # type: ignore

#         return loss, log_vars  # type: ignore

#     def to(self, *args, **kwargs) -> nn.Module:
#         """Overrides this method to call :meth:`BaseDataPreprocessor.to`
#         additionally.

#         Returns:
#             nn.Module: The model itself.
#         """

#         device = torch._C._nn._parse_to(*args, **kwargs)[0]
#         if device is not None:
#             self._set_device(torch.device(device))
#         return super().to(*args, **kwargs)

#     def cuda(
#         self,
#         device: Optional[Union[int, str, torch.device]] = None,
#     ) -> nn.Module:
#         """Overrides this method to call :meth:`BaseDataPreprocessor.cuda`
#         additionally.

#         Returns:
#             nn.Module: The model itself.
#         """
#         if device is None or isinstance(device, int):
#             device = torch.device('cuda', index=device)
#         self._set_device(torch.device(device))
#         return super().cuda(device)

#     def musa(
#         self,
#         device: Optional[Union[int, str, torch.device]] = None,
#     ) -> nn.Module:
#         """Overrides this method to call :meth:`BaseDataPreprocessor.musa`
#         additionally.

#         Returns:
#             nn.Module: The model itself.
#         """
#         if device is None or isinstance(device, int):
#             device = torch.device('musa', index=device)
#         self._set_device(torch.device(device))
#         return super().musa(device)

#     def mlu(
#         self,
#         device: Union[int, str, torch.device, None] = None,
#     ) -> nn.Module:
#         """Overrides this method to call :meth:`BaseDataPreprocessor.mlu`
#         additionally.

#         Returns:
#             nn.Module: The model itself.
#         """
#         device = torch.device('mlu', torch.mlu.current_device())
#         self._set_device(device)
#         return super().mlu()

#     def npu(
#         self,
#         device: Union[int, str, torch.device, None] = None,
#     ) -> nn.Module:
#         """Overrides this method to call :meth:`BaseDataPreprocessor.npu`
#         additionally.

#         Returns:
#             nn.Module: The model itself.

#         Note:
#             This generation of NPU(Ascend910) does not support
#             the use of multiple cards in a single process,
#             so the index here needs to be consistent with the default device
#         """
#         device = torch.npu.current_device()
#         self._set_device(device)
#         return super().npu()

#     def cpu(self, *args, **kwargs) -> nn.Module:
#         """Overrides this method to call :meth:`BaseDataPreprocessor.cpu`
#         additionally.

#         Returns:
#             nn.Module: The model itself.
#         """
#         self._set_device(torch.device('cpu'))
#         return super().cpu()

#     def _set_device(self, device: torch.device) -> None:
#         """Recursively set device for `BaseDataPreprocessor` instance.

#         Args:
#             device (torch.device): the desired device of the parameters and
#                 buffers in this module.
#         """

#         def apply_fn(module):
#             if not isinstance(module, BaseDataPreprocessor):
#                 return
#             if device is not None:
#                 module._device = device

#         self.apply(apply_fn)

#     @abstractmethod
#     def forward(self,
#                 inputs: torch.Tensor,
#                 data_samples: Optional[list] = None,
#                 mode: str = 'tensor') -> Union[Dict[str, torch.Tensor], list]:
#         """Returns losses or predictions of training, validation, testing, and
#         simple inference process.

#         ``forward`` method of BaseModel is an abstract method, its subclasses
#         must implement this method.

#         Accepts ``batch_inputs`` and ``data_sample`` processed by
#         :attr:`data_preprocessor`, and returns results according to mode
#         arguments.

#         During non-distributed training, validation, and testing process,
#         ``forward`` will be called by ``BaseModel.train_step``,
#         ``BaseModel.val_step`` and ``BaseModel.test_step`` directly.

#         During distributed data parallel training process,
#         ``MMSeparateDistributedDataParallel.train_step`` will first call
#         ``DistributedDataParallel.forward`` to enable automatic
#         gradient synchronization, and then call ``forward`` to get training
#         loss.

#         Args:
#             inputs (torch.Tensor): batch input tensor collated by
#                 :attr:`data_preprocessor`.
#             data_samples (list, optional):
#                 data samples collated by :attr:`data_preprocessor`.
#             mode (str): mode should be one of ``loss``, ``predict`` and
#                 ``tensor``

#                 - ``loss``: Called by ``train_step`` and return loss ``dict``
#                   used for logging
#                 - ``predict``: Called by ``val_step`` and ``test_step``
#                   and return list of results used for computing metric.
#                 - ``tensor``: Called by custom use to get ``Tensor`` type
#                   results.

#         Returns:
#             dict or list:
#                 - If ``mode == loss``, return a ``dict`` of loss tensor used
#                   for backward and logging.
#                 - If ``mode == predict``, return a ``list`` of inference
#                   results.
#                 - If ``mode == tensor``, return a tensor or ``tuple`` of tensor
#                   or ``dict`` of tensor for custom use.
#         """

#     def _run_forward(self, data: Union[dict, tuple, list],
#                      mode: str) -> Union[Dict[str, torch.Tensor], list]:
#         """Unpacks data for :meth:`forward`

#         Args:
#             data (dict or tuple or list): Data sampled from dataset.
#             mode (str): Mode of forward.

#         Returns:
#             dict or list: Results of training or testing mode.
#         """
#         if isinstance(data, dict):
#             results = self(**data, mode=mode)
#         elif isinstance(data, (list, tuple)):
#             results = self(*data, mode=mode)
#         else:
#             raise TypeError('Output of `data_preprocessor` should be '
#                             f'list, tuple or dict, but got {type(data)}')
#         return results
    
# class BaseSegmentor(BaseModel, metaclass=ABCMeta):
#     """Base class for segmentors.

#     Args:
#         data_preprocessor (dict, optional): Model preprocessing config
#             for processing the input data. it usually includes
#             ``to_rgb``, ``pad_size_divisor``, ``pad_val``,
#             ``mean`` and ``std``. Default to None.
#        init_cfg (dict, optional): the config to control the
#            initialization. Default to None.
#     """

#     def __init__(self,
#                  data_preprocessor = None,
#                  init_cfg = None):
#         super().__init__(
#             data_preprocessor=data_preprocessor, init_cfg=init_cfg)

#     @property
#     def with_neck(self) -> bool:
#         """bool: whether the segmentor has neck"""
#         return hasattr(self, 'neck') and self.neck is not None

#     @property
#     def with_auxiliary_head(self) -> bool:
#         """bool: whether the segmentor has auxiliary head"""
#         return hasattr(self,
#                        'auxiliary_head') and self.auxiliary_head is not None

#     @property
#     def with_decode_head(self) -> bool:
#         """bool: whether the segmentor has decode head"""
#         return hasattr(self, 'decode_head') and self.decode_head is not None

#     @abstractmethod
#     def extract_feat(self, inputs: Tensor) -> bool:
#         """Placeholder for extract features from images."""
#         pass

#     @abstractmethod
#     def encode_decode(self, inputs: Tensor, batch_data_samples):
#         """Placeholder for encode images with backbone and decode into a
#         semantic segmentation map of the same size as input."""
#         pass

#     def forward(self,
#                 inputs: Tensor,
#                 data_samples = None,
#                 mode: str = 'tensor'):
#         """The unified entry for a forward process in both training and test.

#         The method should accept three modes: "tensor", "predict" and "loss":

#         - "tensor": Forward the whole network and return tensor or tuple of
#         tensor without any post-processing, same as a common nn.Module.
#         - "predict": Forward and return the predictions, which are fully
#         processed to a list of :obj:`SegDataSample`.
#         - "loss": Forward and return a dict of losses according to the given
#         inputs and data samples.

#         Note that this method doesn't handle neither back propagation nor
#         optimizer updating, which are done in the :meth:`train_step`.

#         Args:
#             inputs (torch.Tensor): The input tensor with shape (N, C, ...) in
#                 general.
#             data_samples (list[:obj:`SegDataSample`]): The seg data samples.
#                 It usually includes information such as `metainfo` and
#                 `gt_sem_seg`. Default to None.
#             mode (str): Return what kind of value. Defaults to 'tensor'.

#         Returns:
#             The return type depends on ``mode``.

#             - If ``mode="tensor"``, return a tensor or a tuple of tensor.
#             - If ``mode="predict"``, return a list of :obj:`DetDataSample`.
#             - If ``mode="loss"``, return a dict of tensor.
#         """
#         if mode == 'loss':
#             return self.loss(inputs, data_samples)
#         elif mode == 'predict':
#             return self.predict(inputs, data_samples)
#         elif mode == 'tensor':
#             return self._forward(inputs, data_samples)
#         else:
#             raise RuntimeError(f'Invalid mode "{mode}". '
#                                'Only supports loss, predict and tensor mode')

#     @abstractmethod
#     def loss(self, inputs: Tensor, data_samples) -> dict:
#         """Calculate losses from a batch of inputs and data samples."""
#         pass

#     @abstractmethod
#     def predict(self,
#                 inputs: Tensor,
#                 data_samples = None):
#         """Predict results from a batch of inputs and data samples with post-
#         processing."""
#         pass

#     @abstractmethod
#     def _forward(self,
#                  inputs: Tensor,
#                  data_samples = None) -> Tuple[List[Tensor]]:
#         """Network forward process.

#         Usually includes backbone, neck and head forward without any post-
#         processing.
#         """
#         pass

# class GCNetHead(BaseDecodeHead):
#     """Decode head for RDRNetV2.

#     Args:
#         in_channels (int): Number of input channels.
#         channels (int): Number of output channels.
#         num_classes (int): Number of classes.
#         norm_cfg (dict, optional): Config dict for normalization layer.
#             Default: dict(type='BN').
#         act_cfg (dict, optional): Config dict for activation layer.
#             Default: dict(type='ReLU', inplace=True).
#     """

#     def __init__(self,
#                  in_channels: int,
#                  channels: int,
#                  num_classes: int,
#                  norm_cfg = dict(type='BN', requires_grad=True),
#                  act_cfg = dict(type='ReLU', inplace=True),
#                  **kwargs):
#         super().__init__(
#             in_channels,
#             channels,
#             num_classes=num_classes,
#             norm_cfg=norm_cfg,
#             act_cfg=act_cfg,
#             **kwargs)

#         self.head = self._make_base_head(self.in_channels, self.channels)

#         self.aux_head_c4 = self._make_base_head(self.in_channels // 2, self.channels)
#         self.aux_cls_seg_c4 = nn.Conv2d(self.channels, self.out_channels, kernel_size=1)

#     def init_weights(self):
#         for m in self.modules():
#             if isinstance(m, nn.Conv2d):
#                 nn.init.kaiming_normal_(
#                     m.weight, mode='fan_out', nonlinearity='relu')
#             elif isinstance(m, nn.BatchNorm2d):
#                 nn.init.constant_(m.weight, 1)
#                 nn.init.constant_(m.bias, 0)

#     def forward(
#             self,
#             inputs: Union[Tensor,
#                           Tuple[Tensor]]) -> Union[Tensor, Tuple[Tensor]]:
#         if self.training:
#             c4_feat, c6_feat = inputs
#             c4_feat = self.aux_head_c4(c4_feat)
#             c4_feat = self.aux_cls_seg_c4(c4_feat)
#             c6_feat = self.head(c6_feat)
#             c6_feat = self.cls_seg(c6_feat)

#             return c4_feat, c6_feat
#         else:
#             c6_feat = self.head(inputs)
#             c6_feat = self.cls_seg(c6_feat)

#             return c6_feat

#     def _make_base_head(self, in_channels: int,
#                         channels: int) -> nn.Sequential:
#         layers = [
#             nn.BatchNorm2d(in_channels, affine=True)[1],
#             nn.ReLU(inplace=True),
#             ConvModule(
#                 in_channels,
#                 channels,
#                 kernel_size=3,
#                 padding=1,
#                 norm_cfg=self.norm_cfg,
#                 act_cfg=self.act_cfg,
#                 order=('conv', 'norm', 'act')),
#         ]

#         return nn.Sequential(*layers)

#     def loss_by_feat(self, seg_logits: Tuple[Tensor],
#                      batch_data_samples) -> dict:
#         loss = dict()
#         c4_logit, c6_logit = seg_logits
#         seg_label = self._stack_batch_gt(batch_data_samples)

#         c4_logit = resize(
#             c4_logit,
#             size=seg_label.shape[2:],
#             mode='bilinear',
#             align_corners=self.align_corners)
#         c6_logit = resize(
#             c6_logit,
#             size=seg_label.shape[2:],
#             mode='bilinear',
#             align_corners=self.align_corners)
#         seg_label = seg_label.squeeze(1)

#         loss['loss_c4'] = self.loss_decode[0](c4_logit, seg_label)
#         loss['loss_c6'] = self.loss_decode[1](c6_logit, seg_label)
#         loss['acc_seg'] = accuracy(
#             c6_logit, seg_label, ignore_index=self.ignore_index)

#         return loss
    
# class WholeGCNet(BaseSegmentor):
#     """Encoder Decoder segmentors.

#     EncoderDecoder typically consists of backbone, decode_head, auxiliary_head.
#     Note that auxiliary_head is only used for deep supervision during training,
#     which could be dumped during inference.

#     1. The ``loss`` method is used to calculate the loss of model,
#     which includes two steps: (1) Extracts features to obtain the feature maps
#     (2) Call the decode head loss function to forward decode head model and
#     calculate losses.

#     .. code:: text

#      loss(): extract_feat() -> _decode_head_forward_train() -> _auxiliary_head_forward_train (optional)
#      _decode_head_forward_train(): decode_head.loss()
#      _auxiliary_head_forward_train(): auxiliary_head.loss (optional)

#     2. The ``predict`` method is used to predict segmentation results,
#     which includes two steps: (1) Run inference function to obtain the list of
#     seg_logits (2) Call post-processing function to obtain list of
#     ``SegDataSample`` including ``pred_sem_seg`` and ``seg_logits``.

#     .. code:: text

#      predict(): inference() -> postprocess_result()
#      infercen(): whole_inference()/slide_inference()
#      whole_inference()/slide_inference(): encoder_decoder()
#      encoder_decoder(): extract_feat() -> decode_head.predict()

#     3. The ``_forward`` method is used to output the tensor by running the model,
#     which includes two steps: (1) Extracts features to obtain the feature maps
#     (2)Call the decode head forward function to forward decode head model.

#     .. code:: text

#      _forward(): extract_feat() -> _decode_head.forward()

#     Args:

#         backbone (ConfigType): The config for the backnone of segmentor.
#         decode_head (ConfigType): The config for the decode head of segmentor.
#         neck (OptConfigType): The config for the neck of segmentor.
#             Defaults to None.
#         auxiliary_head (OptConfigType): The config for the auxiliary head of
#             segmentor. Defaults to None.
#         train_cfg (OptConfigType): The config for training. Defaults to None.
#         test_cfg (OptConfigType): The config for testing. Defaults to None.
#         data_preprocessor (dict, optional): The pre-process config of
#             :class:`BaseDataPreprocessor`.
#         pretrained (str, optional): The path for pretrained model.
#             Defaults to None.
#         init_cfg (dict, optional): The weight initialized config for
#             :class:`BaseModule`.
#     """  # noqa: E501

#     def __init__(self,
#                  backbone = None,
#                  decode_head = None,
#                  auxiliary_head = None,
#                  train_cfg = None,
#                  test_cfg = None,
#                  data_preprocessor = None,
#                  pretrained: Optional[str] = None,
#                  init_cfg = None):
#         super().__init__(
#             data_preprocessor=data_preprocessor, init_cfg=init_cfg)
#         if pretrained is not None:
#             assert backbone.get('pretrained') is None, \
#                 'both backbone and segmentor set pretrained weight'
#             backbone.pretrained = pretrained
#         self.backbone = GCNet(
#             in_channels=3,
#             channels=32,
#             ppm_channels=128,
#             num_blocks_per_stage=[4, 4, [5, 4], [5, 4], [2, 2]],
#             norm_cfg=dict(type='BN', requires_grad=True),
#             act_cfg=dict(type='ReLU', inplace=True),
#             align_corners=False,
#             deploy=False
#         )
#         self._init_decode_head(decode_head)

#         self.train_cfg = train_cfg
#         self.test_cfg = test_cfg

#         assert self.with_decode_head

#     def _init_decode_head(self, decode_head) -> None:
#         """Initialize ``decode_head``"""
#         self.decode_head = GCNetHead(
#             in_channels=32 * 4,
#             channels=64,
#             dropout_ratio=0.,
#             num_classes=19,
#             align_corners=False
#         )
#         self.align_corners = self.decode_head.align_corners
#         self.num_classes = self.decode_head.num_classes
#         self.out_channels = self.decode_head.out_channels

#     def extract_feat(self, inputs: Tensor) -> List[Tensor]:
#         """Extract features from images."""
#         x = self.backbone(inputs)
#         if self.with_neck:
#             x = self.neck(x)
#         return x

#     def encode_decode(self, inputs: Tensor,
#                       batch_img_metas: List[dict]) -> Tensor:
#         """Encode images with backbone and decode into a semantic segmentation
#         map of the same size as input."""
#         x = self.extract_feat(inputs)
#         seg_logits = self.decode_head.predict(x, batch_img_metas,
#                                               self.test_cfg)

#         return seg_logits

#     def _decode_head_forward_train(self, inputs: List[Tensor],
#                                    data_samples) -> dict:
#         """Run forward function and calculate loss for decode head in
#         training."""
#         losses = dict()
#         loss_decode = self.decode_head.loss(inputs, data_samples,
#                                             self.train_cfg)

#         losses.update(add_prefix(loss_decode, 'decode'))
#         return losses

#     def loss(self, inputs: Tensor, data_samples) -> dict:
#         """Calculate losses from a batch of inputs and data samples.

#         Args:
#             inputs (Tensor): Input images.
#             data_samples (list[:obj:`SegDataSample`]): The seg data samples.
#                 It usually includes information such as `metainfo` and
#                 `gt_sem_seg`.

#         Returns:
#             dict[str, Tensor]: a dictionary of loss components
#         """

#         x = self.extract_feat(inputs)

#         losses = dict()

#         loss_decode = self._decode_head_forward_train(x, data_samples)
#         losses.update(loss_decode)

#         if self.with_auxiliary_head:
#             loss_aux = self._auxiliary_head_forward_train(x, data_samples)
#             losses.update(loss_aux)

#         return losses

#     def predict(self,
#                 inputs: Tensor,
#                 data_samples = None):
#         """Predict results from a batch of inputs and data samples with post-
#         processing.

#         Args:
#             inputs (Tensor): Inputs with shape (N, C, H, W).
#             data_samples (List[:obj:`SegDataSample`], optional): The seg data
#                 samples. It usually includes information such as `metainfo`
#                 and `gt_sem_seg`.

#         Returns:
#             list[:obj:`SegDataSample`]: Segmentation results of the
#             input images. Each SegDataSample usually contain:

#             - ``pred_sem_seg``(PixelData): Prediction of semantic segmentation.
#             - ``seg_logits``(PixelData): Predicted logits of semantic
#                 segmentation before normalization.
#         """
#         if data_samples is not None:
#             batch_img_metas = [
#                 data_sample.metainfo for data_sample in data_samples
#             ]
#         else:
#             batch_img_metas = [
#                 dict(
#                     ori_shape=inputs.shape[2:],
#                     img_shape=inputs.shape[2:],
#                     pad_shape=inputs.shape[2:],
#                     padding_size=[0, 0, 0, 0])
#             ] * inputs.shape[0]

#         seg_logits = self.inference(inputs, batch_img_metas)

#         return self.postprocess_result(seg_logits, data_samples)

#     def _forward(self,
#                  inputs: Tensor,
#                  data_samples = None) -> Tensor:
#         """Network forward process.

#         Args:
#             inputs (Tensor): Inputs with shape (N, C, H, W).
#             data_samples (List[:obj:`SegDataSample`]): The seg
#                 data samples. It usually includes information such
#                 as `metainfo` and `gt_sem_seg`.

#         Returns:
#             Tensor: Forward output of model without any post-processes.
#         """
#         x = self.extract_feat(inputs)
#         return self.decode_head.forward(x)

#     def slide_inference(self, inputs: Tensor,
#                         batch_img_metas: List[dict]) -> Tensor:
#         """Inference by sliding-window with overlap.

#         If h_crop > h_img or w_crop > w_img, the small patch will be used to
#         decode without padding.

#         Args:
#             inputs (tensor): the tensor should have a shape NxCxHxW,
#                 which contains all images in the batch.
#             batch_img_metas (List[dict]): List of image metainfo where each may
#                 also contain: 'img_shape', 'scale_factor', 'flip', 'img_path',
#                 'ori_shape', and 'pad_shape'.
#                 For details on the values of these keys see
#                 `mmseg/datasets/pipelines/formatting.py:PackSegInputs`.

#         Returns:
#             Tensor: The segmentation results, seg_logits from model of each
#                 input image.
#         """

#         h_stride, w_stride = self.test_cfg.stride
#         h_crop, w_crop = self.test_cfg.crop_size
#         batch_size, _, h_img, w_img = inputs.size()
#         out_channels = self.out_channels
#         h_grids = max(h_img - h_crop + h_stride - 1, 0) // h_stride + 1
#         w_grids = max(w_img - w_crop + w_stride - 1, 0) // w_stride + 1
#         preds = inputs.new_zeros((batch_size, out_channels, h_img, w_img))
#         count_mat = inputs.new_zeros((batch_size, 1, h_img, w_img))
#         for h_idx in range(h_grids):
#             for w_idx in range(w_grids):
#                 y1 = h_idx * h_stride
#                 x1 = w_idx * w_stride
#                 y2 = min(y1 + h_crop, h_img)
#                 x2 = min(x1 + w_crop, w_img)
#                 y1 = max(y2 - h_crop, 0)
#                 x1 = max(x2 - w_crop, 0)
#                 crop_img = inputs[:, :, y1:y2, x1:x2]
#                 # change the image shape to patch shape
#                 batch_img_metas[0]['img_shape'] = crop_img.shape[2:]
#                 # the output of encode_decode is seg logits tensor map
#                 # with shape [N, C, H, W]
#                 crop_seg_logit = self.encode_decode(crop_img, batch_img_metas)
#                 preds += F.pad(crop_seg_logit,
#                                (int(x1), int(preds.shape[3] - x2), int(y1),
#                                 int(preds.shape[2] - y2)))

#                 count_mat[:, :, y1:y2, x1:x2] += 1
#         assert (count_mat == 0).sum() == 0
#         seg_logits = preds / count_mat

#         return seg_logits

#     def whole_inference(self, inputs: Tensor,
#                         batch_img_metas: List[dict]) -> Tensor:
#         """Inference with full image.

#         Args:
#             inputs (Tensor): The tensor should have a shape NxCxHxW, which
#                 contains all images in the batch.
#             batch_img_metas (List[dict]): List of image metainfo where each may
#                 also contain: 'img_shape', 'scale_factor', 'flip', 'img_path',
#                 'ori_shape', and 'pad_shape'.
#                 For details on the values of these keys see
#                 `mmseg/datasets/pipelines/formatting.py:PackSegInputs`.

#         Returns:
#             Tensor: The segmentation results, seg_logits from model of each
#                 input image.
#         """

#         seg_logits = self.encode_decode(inputs, batch_img_metas)

#         return seg_logits

#     def inference(self, inputs: Tensor, batch_img_metas: List[dict]) -> Tensor:
#         """Inference with slide/whole style.

#         Args:
#             inputs (Tensor): The input image of shape (N, 3, H, W).
#             batch_img_metas (List[dict]): List of image metainfo where each may
#                 also contain: 'img_shape', 'scale_factor', 'flip', 'img_path',
#                 'ori_shape', 'pad_shape', and 'padding_size'.
#                 For details on the values of these keys see
#                 `mmseg/datasets/pipelines/formatting.py:PackSegInputs`.

#         Returns:
#             Tensor: The segmentation results, seg_logits from model of each
#                 input image.
#         """
#         assert self.test_cfg.get('mode', 'whole') in ['slide', 'whole'], \
#             f'Only "slide" or "whole" test mode are supported, but got ' \
#             f'{self.test_cfg["mode"]}.'
#         ori_shape = batch_img_metas[0]['ori_shape']
#         if not all(_['ori_shape'] == ori_shape for _ in batch_img_metas):
#             print('Image shapes are different in the batch.')
#         if self.test_cfg.mode == 'slide':
#             seg_logit = self.slide_inference(inputs, batch_img_metas)
#         else:
#             seg_logit = self.whole_inference(inputs, batch_img_metas)

#         return seg_logit

#     def aug_test(self, inputs, batch_img_metas, rescale=True):
#         """Test with augmentations.

#         Only rescale=True is supported.
#         """
#         # aug_test rescale all imgs back to ori_shape for now
#         assert rescale
#         # to save memory, we get augmented seg logit inplace
#         seg_logit = self.inference(inputs[0], batch_img_metas[0], rescale)
#         for i in range(1, len(inputs)):
#             cur_seg_logit = self.inference(inputs[i], batch_img_metas[i],
#                                            rescale)
#             seg_logit += cur_seg_logit
#         seg_logit /= len(inputs)
#         seg_pred = seg_logit.argmax(dim=1)
#         # unravel batch dim
#         seg_pred = list(seg_pred)
#         return seg_pred
