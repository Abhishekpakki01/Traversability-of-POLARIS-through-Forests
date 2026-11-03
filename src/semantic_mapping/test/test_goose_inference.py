"""Check aspect-preserving inference, mirror averaging and masks with a stub network."""

import json

import numpy as np
import pytest
import torch

from semantic_mapping.goose_inference import AspectPredictor, aspect_size, polygon_mask


def stub(size=None, mirror=False, mask=None):
    predictor = AspectPredictor.__new__(AspectPredictor)
    torch.manual_seed(0)
    predictor.model = torch.nn.Conv2d(3, 64, 1).eval()
    predictor.class_groups = torch.arange(64) % 6
    predictor.size, predictor.mirror, predictor.mask = size, mirror, mask
    return predictor


def test_aspect_size_keeps_ratio_in_multiples_of_32():
    assert aspect_size(1024, 544) == (512, 960)
    assert aspect_size(1920, 1200) == (512, 832)
    with pytest.raises(ValueError):
        aspect_size(0, 10)


def test_prediction_shape_normalisation_mirror_and_mask(tmp_path):
    image = np.random.default_rng(1).integers(0, 255, (68, 128, 3), dtype=np.uint8)
    p = stub().predict(image)
    assert p.shape == aspect_size(128, 68)+(6,)  # default: 512 short side, aspect kept
    np.testing.assert_allclose(p.sum(2), 1, atol=1e-5)
    mirrored = stub(size=(32, 64), mirror=True).predict(image)
    single = stub(size=(32, 64)).predict(image)
    flipped = stub(size=(32, 64)).predict(image[:, ::-1])[:, ::-1]
    np.testing.assert_allclose(mirrored, (single+flipped)/2, atol=1e-5)
    config = tmp_path/'mask.json'
    config.write_text(json.dumps(dict(width=128, height=68, margin_px=0,
                                      polygon=[[0, 68], [0, 50], [128, 50], [128, 68]])))
    mask = polygon_mask(config, 128, 68)
    assert mask[60, 10] and not mask[10, 10]
    masked = stub(size=(32, 64), mask=mask).predict(image)
    assert np.isnan(masked[-1]).all() and np.isfinite(masked[0]).all()
    with pytest.raises(ValueError):
        polygon_mask(config, 64, 32)
