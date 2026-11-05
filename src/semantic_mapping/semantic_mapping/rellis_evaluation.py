"""Accumulate and score grouped predictions against RELLIS-3D labels."""
import numpy as np


class Confusion:
    """Counts of RELLIS label ID against predicted group, over all images."""

    def __init__(self, groups):
        self.groups = list(groups)
        self.counts = {}

    def add(self, labels, predicted):
        size = len(self.groups)
        for label in np.unique(labels).tolist():
            found = np.bincount(predicted[labels == label].ravel(), minlength=size)[:size]
            self.counts[label] = self.counts.get(label, np.zeros(size, np.int64))+found

    def grouped(self, mapping):
        """Square truth-group by predicted-group table; labels with no group are not scored."""
        table = np.zeros((len(self.groups), len(self.groups)), np.int64)
        for label, counts in self.counts.items():
            group = mapping[label]['group']
            if group is not None:
                table[self.groups.index(group)] += counts
        return table


def scores(table, groups):
    """Pixel accuracy, per-group IoU, precision and recall, and mean IoU of groups with truth."""
    per_group = {}
    for index, name in enumerate(groups):
        truth, predicted, hit = int(table[index].sum()), int(table[:, index].sum()), int(table[index, index])
        if truth:
            union = truth+predicted-hit
            per_group[name] = dict(truth_pixels=truth, predicted_pixels=predicted, iou=hit/union,
                                   precision=hit/predicted if predicted else 0., recall=hit/truth)
    total = int(table.sum())
    return dict(evaluated_pixels=total, pixel_accuracy=float(np.trace(table)/total) if total else None,
                mean_iou=float(np.mean([v['iou'] for v in per_group.values()])) if per_group else None,
                per_group=per_group)
