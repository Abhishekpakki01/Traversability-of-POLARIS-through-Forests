#!/usr/bin/env python3
"""Score the GOOSE camera model on the human-labelled RELLIS-3D test split.

The model was not trained on RELLIS-3D, and the class grouping (rellis_label_groups.json) was fixed
before this test, so nothing here is tuned to these labels. Classes with no group, such as tree and
sky, are reported in the class table but not scored.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from semantic_mapping.goose_inference import AspectPredictor, aspect_size
from semantic_mapping.rellis_evaluation import Confusion, scores
from semantic_mapping.semantic_fusion import CLASS_NAMES


def main():
    package = Path(__file__).resolve().parents[1]
    root = Path.home()/'thesis_data/rellis_3d_full'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=root/'data/Rellis-3D')
    parser.add_argument('--split', type=Path, default=root/'splits/test.lst')
    parser.add_argument('--mapping', type=Path, default=package/'config/rellis_label_groups.json')
    parser.add_argument('--model', type=Path, default=Path.home()/'semantic_map_ws/models/goose/ppliteseg_class_512.pth')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--short-side', type=int, default=512)
    parser.add_argument('--threads', type=int, default=6)
    parser.add_argument('--limit', type=int, default=0, help='test: use only the first images')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    mapping = {int(k): v for k, v in json.loads(args.mapping.read_text())['labels'].items()}
    goose = json.loads((package/'config/goose_classes.json').read_text())['classes']
    groups = [CLASS_NAMES.index(c['group']) for c in goose]
    pairs = [line.split() for line in args.split.read_text().splitlines() if line.strip()]
    pairs = pairs[:args.limit] if args.limit else pairs
    first = cv2.imread(str(args.data/pairs[0][0]))
    size = aspect_size(first.shape[1], first.shape[0], args.short_side)
    predictor = AspectPredictor(args.model, groups, args.threads, size)
    overall, sequences, started = Confusion(CLASS_NAMES), {}, time.time()
    for count, (image_path, label_path) in enumerate(pairs, 1):
        bgr = cv2.imread(str(args.data/image_path))
        labels = cv2.imread(str(args.data/label_path), cv2.IMREAD_UNCHANGED)
        if bgr is None or labels is None or labels.shape != bgr.shape[:2]:
            raise ValueError(f'Unreadable or mismatched pair: {image_path}')
        unknown = set(np.unique(labels).tolist())-set(mapping)
        if unknown:
            raise ValueError(f'Label IDs without a mapping: {sorted(unknown)}')
        probabilities = cv2.resize(predictor.predict(bgr), (labels.shape[1], labels.shape[0]), interpolation=cv2.INTER_LINEAR)
        predicted = probabilities.argmax(axis=2).astype(np.uint8)
        overall.add(labels, predicted)
        sequences.setdefault(image_path.split('/')[0], Confusion(CLASS_NAMES)).add(labels, predicted)
        if count % 100 == 0:
            print(f'{count}/{len(pairs)} images, {(time.time()-started)/count:.2f} s each', flush=True)
    result = scores(overall.grouped(mapping), CLASS_NAMES)
    by_class = {}
    for label, counts in sorted(overall.counts.items()):
        total = int(counts.sum())
        by_class[mapping[label]['name']] = dict(group=mapping[label]['group'], pixels=total, predicted_fraction={
            name: float(counts[i]/total) for i, name in enumerate(CLASS_NAMES)})
    report = dict(images=len(pairs), split=str(args.split), model_input=f'{size[0]}x{size[1]}',
                  mapping_sha256=hashlib.sha256(args.mapping.read_bytes()).hexdigest(), **result,
                  by_sequence={k: scores(v.grouped(mapping), CLASS_NAMES) for k, v in sorted(sequences.items())},
                  by_rellis_class=by_class,
                  note='Pretrained GOOSE model on another dataset, grouped classes. Tree and sky are not scored.')
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('images', 'pixel_accuracy', 'mean_iou')}, indent=2))
    for name, values in report['per_group'].items():
        print(f"{name:12s} IoU {values['iou']:.2f}  precision {values['precision']:.2f}  recall {values['recall']:.2f}")


if __name__ == '__main__':
    main()
