"""Aspect-preserving GOOSE inference, optional mirror averaging and image masks.

GoosePredictor (unchanged) squashes every image to 512x512. Here the image is
resized to a chosen height x width (keep the aspect ratio, multiples of 32),
optionally averaged with its mirror image, and the grouped class probabilities
are returned at the model resolution. Masked pixels (e.g. the vehicle hood)
are returned as NaN so callers can drop them instead of labelling them.
"""

import json

import cv2
import numpy as np
import torch
import torch.nn.functional as F

from semantic_mapping.goose_model import GoosePredictor


def aspect_size(width, height, short_side=512, multiple=32):
    """Model input (height, width) keeping the image aspect ratio."""
    if width <= 0 or height <= 0:
        raise ValueError('Invalid image size')
    scale = short_side/min(width, height)
    snap = lambda value: max(multiple, int(round(value*scale/multiple))*multiple)
    return snap(height), snap(width)


def polygon_mask(config_path, width, height):
    """Boolean image mask (True = masked) from a polygon JSON file."""
    config = json.loads(open(config_path).read())
    if (config['width'], config['height']) != (width, height):
        raise ValueError('Mask was traced for a different image size')
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(config['polygon'], dtype=np.int32)], 1)
    margin = int(config.get('margin_px', 0))
    if margin:
        mask = cv2.dilate(mask, np.ones((2*margin+1, 2*margin+1), np.uint8))
    return mask.astype(bool)


class AspectPredictor(GoosePredictor):
    def __init__(self, checkpoint, class_groups, threads=2, size=None, mirror=False, mask=None):
        super().__init__(checkpoint, class_groups, threads)
        if size is not None and (len(size) != 2 or size[0] % 32 or size[1] % 32):
            raise ValueError('Model input size must be (height, width) in multiples of 32')
        self.size, self.mirror, self.mask = size, mirror, mask

    def _grouped(self, image):
        with torch.inference_mode():
            probabilities = self.model(image).softmax(dim=1)[0]
            grouped = torch.zeros((6,)+probabilities.shape[1:], dtype=probabilities.dtype)
            grouped.index_add_(0, self.class_groups, probabilities)
        return grouped

    def predict(self, bgr):
        size = self.size or aspect_size(bgr.shape[1], bgr.shape[0])
        image = torch.from_numpy(np.ascontiguousarray(bgr[:, :, ::-1])).permute(2, 0, 1)
        image = F.interpolate(image[None].float()/255.0, size=size, mode='bilinear',
                              align_corners=False, antialias=True)
        grouped = self._grouped(image)
        if self.mirror:
            grouped = (grouped+self._grouped(image.flip(-1)).flip(-1))/2
        result = grouped.permute(1, 2, 0).numpy()
        if self.mask is not None:
            small = cv2.resize(self.mask.astype(np.uint8), (size[1], size[0]), interpolation=cv2.INTER_NEAREST)
            result[small.astype(bool)] = np.nan
        return result
