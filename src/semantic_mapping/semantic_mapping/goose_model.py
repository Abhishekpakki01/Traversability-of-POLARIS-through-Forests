"""Inference-only PP-LiteSeg-B adapter for the official 64-class GOOSE weights.

Adapted from Deci-AI/super-gradients (Apache-2.0): ppliteseg.py, stdc.py,
context_modules.py, common.py and conv_bn_act_block.py. Training/framework
dependencies are omitted; parameter names and forward operations are retained.
See THIRD_PARTY_NOTICES.md and LICENSE.super-gradients in this package.
"""

from collections import OrderedDict

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


class ConvBNReLU(nn.Module):
    def __init__(self, inputs, outputs, kernel_size, stride=1, padding=0,
                 groups=1, use_activation=True):
        super().__init__()
        layers = [('conv', nn.Conv2d(inputs, outputs, kernel_size, stride,
                                     padding, groups=groups, bias=False)),
                  ('bn', nn.BatchNorm2d(outputs))]
        if use_activation:
            layers.append(('act', nn.ReLU()))
        self.seq = nn.Sequential(OrderedDict(layers))

    def forward(self, x):
        return self.seq(x)


class STDCBlock(nn.Module):
    def __init__(self, inputs, outputs, stride):
        super().__init__()
        self.conv_list = nn.ModuleList([ConvBNReLU(inputs, outputs//2, 1)])
        self.skip_step1 = (nn.Identity() if stride == 1
                           else nn.AvgPool2d(3, stride=2, padding=1))
        channels = outputs//2
        for output in (outputs//4, outputs//8, outputs//8):
            self.conv_list.append(ConvBNReLU(channels, output, 3, padding=1))
            channels = output
        if stride == 2:
            self.conv_list[1] = nn.Sequential(
                ConvBNReLU(outputs//2, outputs//2, 3, stride=2, padding=1,
                           groups=outputs//2, use_activation=False), self.conv_list[1])

    def forward(self, x):
        x = self.conv_list[0](x)
        outputs = [self.skip_step1(x)]
        for conv in self.conv_list[1:]:
            x = conv(x)
            outputs.append(x)
        return torch.cat(outputs, dim=1)


class Backbone(nn.Module):
    def __init__(self):
        super().__init__()
        self.stages = nn.ModuleDict()
        inputs = 3
        for down, outputs, blocks in zip((2, 4, 8, 16, 32),
                                         (32, 64, 256, 512, 1024), (1, 1, 4, 5, 3)):
            if down < 8:
                stage = [ConvBNReLU(inputs, outputs, 3, stride=2, padding=1)]
            else:
                stage = [STDCBlock(inputs, outputs, 2)]
                stage += [STDCBlock(outputs, outputs, 1) for _ in range(blocks-1)]
            self.stages[f'block_s{down}'] = nn.Sequential(*stage)
            inputs = outputs

    def forward(self, x):
        features = []
        for name, stage in self.stages.items():
            x = stage(x)
            if name in ('block_s8', 'block_s16', 'block_s32'):
                features.append(x)
        return features


class SPPM(nn.Module):
    def __init__(self):
        super().__init__()
        self.branches = nn.ModuleList([
            nn.Sequential(nn.AdaptiveAvgPool2d(size), ConvBNReLU(1024, 128, 1))
            for size in (1, 2, 4)])
        self.conv_out = ConvBNReLU(128, 128, 3, padding=1)

    def forward(self, x):
        outputs = [F.interpolate(branch(x), size=x.shape[2:], mode='bilinear',
                                 align_corners=False) for branch in self.branches]
        return self.conv_out(outputs[0]+outputs[1]+outputs[2])


class Encoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.backbone = Backbone()
        self.context_module = SPPM()
        self.proj_convs = nn.ModuleList([
            ConvBNReLU(inputs, outputs, 3, padding=1)
            for inputs, outputs in zip((256, 512, 1024), (96, 128, 128))])

    def forward(self, x):
        features = self.backbone(x)
        context = self.context_module(features[-1])
        return [conv(feat) for conv, feat in zip(self.proj_convs, features)] + [context]


class UAFM(nn.Module):
    def __init__(self, inputs, skips, outputs, factor):
        super().__init__()
        self.conv_atten = nn.Sequential(ConvBNReLU(4, 2, 3, padding=1),
            ConvBNReLU(2, 1, 3, padding=1, use_activation=False))
        self.proj_skip = (nn.Identity() if skips == inputs
                          else ConvBNReLU(skips, inputs, 3, padding=1))
        self.up_x = (nn.Identity() if factor == 1 else nn.Upsample(
            scale_factor=factor, mode='bilinear', align_corners=False))
        self.conv_out = ConvBNReLU(inputs, outputs, 3, padding=1)

    def forward(self, x, skip):
        x, skip = self.up_x(x), self.proj_skip(skip)
        attention = torch.cat([x.mean(1, keepdim=True), x.max(1, keepdim=True)[0],
                              skip.mean(1, keepdim=True), skip.max(1, keepdim=True)[0]], dim=1)
        attention = torch.sigmoid(self.conv_atten(attention))
        return self.conv_out(x*attention+skip*(1-attention))


class Decoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.up_stages = nn.ModuleList([UAFM(128, 128, 128, 1),
                                       UAFM(128, 128, 96, 2), UAFM(96, 96, 64, 2)])

    def forward(self, features):
        x = features[-1]
        for stage, skip in zip(self.up_stages, reversed(features[:-1])):
            x = stage(x, skip)
        return x


class SegmentationHead(nn.Module):
    def __init__(self):
        super().__init__()
        self.seg_head = nn.Sequential(ConvBNReLU(64, 64, 3, padding=1),
                                      nn.Dropout(0), nn.Conv2d(64, 64, 1, bias=False))

    def forward(self, x):
        return self.seg_head(x)


class PPLiteSegB(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder, self.decoder = Encoder(), Decoder()
        self.seg_head = nn.Sequential(SegmentationHead(), nn.Upsample(
            scale_factor=8, mode='bilinear', align_corners=False))

    def forward(self, x):
        return self.seg_head(self.decoder(self.encoder(x)))


class GoosePredictor:
    def __init__(self, checkpoint, class_groups, threads=2):
        torch.set_num_threads(threads)
        self.model = PPLiteSegB().eval()
        state = torch.load(checkpoint, map_location='cpu', weights_only=True)
        # Strict loading must succeed: never run with partially/randomly loaded weights.
        self.model.load_state_dict(state['net'], strict=True)
        self.class_groups = torch.as_tensor(class_groups, dtype=torch.int64)
        if self.class_groups.shape != (64,) or torch.any(self.class_groups < 0):
            raise ValueError('Expected an explicit mapping for every GOOSE class')

    def predict(self, bgr):
        # GOOSE's published inference example uses RGB ToTensor [0,1], without
        # ImageNet normalization, and Resize. Preserve the full image (no crop).
        image = torch.from_numpy(np.ascontiguousarray(bgr[:, :, ::-1])).permute(2, 0, 1)
        image = F.interpolate(image[None].float()/255.0, size=(512, 512),
                              mode='bilinear', align_corners=False, antialias=True)
        with torch.inference_mode():
            probabilities = self.model(image).softmax(dim=1)[0]
            grouped = torch.zeros((6, 512, 512), dtype=probabilities.dtype)
            grouped.index_add_(0, self.class_groups, probabilities)
        return grouped.permute(1, 2, 0).numpy()
