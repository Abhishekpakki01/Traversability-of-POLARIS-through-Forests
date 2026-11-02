#!/usr/bin/env python3
"""Compare grouped GOOSE predictions with a human-labelled RELLIS camera frame.

One labelled image is a sanity check of the class grouping and of obvious
failure modes, not a segmentation accuracy measurement for the dataset.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from semantic_mapping.goose_model import GoosePredictor
from semantic_mapping.semantic_fusion import CLASS_NAMES, PALETTE

EXAMPLE = Path.home()/'RELLIS-3D/utils/example'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scores(truth, predicted, evaluated):
    """Per-group IoU/precision/recall over evaluated pixels for groups present in truth."""
    result = {}
    for i, name in enumerate(CLASS_NAMES[1:], 1):
        t, p = evaluated & (truth == i), evaluated & (predicted == i)
        if not t.any():
            continue
        tp = int((t & p).sum())
        result[name] = dict(truth_pixels=int(t.sum()), predicted_pixels=int(p.sum()),
                            iou=tp/int((t | p).sum()), precision=tp/int(p.sum()) if p.any() else 0.,
                            recall=tp/int(t.sum()))
    return result


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, default=EXAMPLE/'frame000104-1581624663_149.jpg')
    parser.add_argument('--labels', type=Path, default=EXAMPLE/'frame000104-1581624663_149.png')
    parser.add_argument('--mapping', type=Path, default=package/'config/rellis_label_groups.json')
    parser.add_argument('--model', type=Path, default=Path.home()/'semantic_map_ws/models/goose/ppliteseg_class_512.pth')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    bgr = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    raw = cv2.imread(str(args.labels), cv2.IMREAD_UNCHANGED)
    if bgr is None or raw is None or raw.ndim != 2 or raw.shape != bgr.shape[:2]:
        raise ValueError('Need a colour image and a same-size single-channel RELLIS label-ID image')
    mapping = {int(k): v for k, v in json.loads(args.mapping.read_text())['labels'].items()}
    missing = set(np.unique(raw).tolist())-set(mapping)
    if missing:
        raise ValueError(f'Label IDs without an explicit mapping: {sorted(missing)}')
    goose = json.loads((package/'config/goose_classes.json').read_text())['classes']
    predictor = GoosePredictor(args.model, [CLASS_NAMES.index(c['group']) for c in goose], 2)
    probabilities = predictor.predict(bgr)
    # Upsample probabilities, not labels, to the annotation resolution.
    probabilities = cv2.resize(probabilities, (raw.shape[1], raw.shape[0]), interpolation=cv2.INTER_LINEAR)
    predicted = probabilities.argmax(axis=2).astype(np.uint8)
    confident = probabilities.max(axis=2) >= .6

    truth = np.zeros(raw.shape, dtype=np.uint8)
    evaluated = np.zeros(raw.shape, dtype=bool)
    for label, entry in mapping.items():
        if entry['group'] is not None:
            truth[raw == label] = CLASS_NAMES.index(entry['group'])
            evaluated[raw == label] = True
    confusion = {}
    for label in np.unique(raw).tolist():
        pixels = raw == label
        counts = np.bincount(predicted[pixels], minlength=len(CLASS_NAMES))
        confusion[mapping[label]['name']] = dict(
            rellis_id=label, group=mapping[label]['group'], reason=mapping[label].get('reason'),
            pixels=int(pixels.sum()),
            predicted_fraction={name: float(counts[i]/pixels.sum()) for i, name in enumerate(CLASS_NAMES)},
            confident_fraction=float(confident[pixels].mean()))
    per_group = scores(truth, predicted, evaluated)
    per_group_confident = scores(truth, np.where(confident, predicted, 0), evaluated)
    report = dict(
        image=str(args.image.resolve()), image_sha256=sha(args.image),
        labels=str(args.labels.resolve()), labels_sha256=sha(args.labels),
        mapping_sha256=sha(args.mapping), checkpoint_sha256=sha(args.model),
        frames_evaluated=1,
        note='Single human-labelled frame: a sanity check, not dataset accuracy.',
        evaluated_pixel_fraction=float(evaluated.mean()),
        pixel_accuracy=float((predicted[evaluated] == truth[evaluated]).mean()),
        mean_iou_present_groups=float(np.mean([v['iou'] for v in per_group.values()])),
        per_group=per_group,
        per_group_at_display_threshold_0_6=per_group_confident,
        confusion_by_rellis_class=confusion)
    args.output.mkdir(parents=True)
    colour = lambda groups: PALETTE[groups][:, :, ::-1]
    error = np.full(bgr.shape, 200, dtype=np.uint8)
    error[evaluated & (predicted == truth)] = (12, 163, 12)[::-1]
    error[evaluated & (predicted != truth)] = (208, 59, 59)[::-1]
    panels = [('camera', bgr), ('truth_grouped', cv2.addWeighted(bgr, .35, colour(np.where(evaluated, truth, 0)), .65, 0)),
              ('prediction_grouped', cv2.addWeighted(bgr, .35, colour(predicted), .65, 0)),
              ('agreement', error)]
    for name, image in panels:
        cv2.imwrite(str(args.output/f'{name}.jpg'), image, [cv2.IMWRITE_JPEG_QUALITY, 92])
    small = [cv2.resize(image, (640, 400), interpolation=cv2.INTER_AREA) for _, image in panels]
    titles = ['Camera', 'RELLIS labels (grouped; gray = not scored)', 'GOOSE prediction (grouped)',
              'Green agree / red disagree / gray not scored']
    for image, title in zip(small, titles):
        cv2.rectangle(image, (0, 0), (640, 26), (27, 27, 27), -1)
        cv2.putText(image, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .5, (235, 235, 235), 1, cv2.LINE_AA)
    cv2.imwrite(str(args.output/'summary.jpg'), np.vstack([np.hstack(small[:2]), np.hstack(small[2:])]),
                [cv2.IMWRITE_JPEG_QUALITY, 92])
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('pixel_accuracy', 'mean_iou_present_groups', 'per_group')}, indent=2))


if __name__ == '__main__':
    main()
