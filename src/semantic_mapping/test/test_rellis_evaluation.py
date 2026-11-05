import numpy as np
import pytest

from semantic_mapping.rellis_evaluation import Confusion, scores

GROUPS = ['unknown', 'ground', 'vegetation']
MAPPING = {0: dict(group=None), 1: dict(group='ground'), 3: dict(group='vegetation')}


def test_scores_from_a_known_table():
    confusion = Confusion(GROUPS)
    labels = np.array([1, 1, 1, 3, 3, 0])
    predicted = np.array([1, 1, 2, 2, 1, 2])
    confusion.add(labels, predicted)
    result = scores(confusion.grouped(MAPPING), GROUPS)
    assert result['evaluated_pixels'] == 5  # the void pixel is not scored
    assert result['pixel_accuracy'] == pytest.approx(3/5)
    assert result['per_group']['ground']['recall'] == pytest.approx(2/3)
    assert result['per_group']['ground']['precision'] == pytest.approx(2/3)
    assert result['per_group']['ground']['iou'] == pytest.approx(2/4)
    assert result['per_group']['vegetation']['iou'] == pytest.approx(1/3)
    assert result['mean_iou'] == pytest.approx(5/12)


def test_counts_accumulate_across_images():
    confusion = Confusion(GROUPS)
    confusion.add(np.array([1, 1]), np.array([1, 2]))
    confusion.add(np.array([1]), np.array([1]))
    assert confusion.counts[1].tolist() == [0, 2, 1]
