# Pretrained image model

This folder records the official GOOSE PP-LiteSeg-B checkpoint's origin and
architecture/preprocessing reference. The camera model was pretrained externally;
the student has not trained a new model. The mapping demonstrations use RELLIS
and TartanDrive recordings, not a GOOSE recording.

The checkpoint is not stored in normal Git history. On the original PC it is:

```text
/home/abhishek/semantic_map_ws/models/goose/ppliteseg_class_512.pth
```

Official source: <https://goose-dataset.de/models/ppliteseg_class_512.pth>

SHA-256: `6dd412c0c99115e359896c4cab43a8e6bce9e09b843e7fa885fe597b0a6121cd`

`manifest.json` records provenance, source hashes and preprocessing.
`parity_check.json` records implementation equivalence for the checked forward
pass; it does not measure segmentation accuracy on the mapping recordings.
`upstream/` retains reference code/license, and `labeling_policy.pdf` supports
the explicit 64-class table used in the application.

The current predictor uses RGB floats in [0,1], antialiased resize to 512×512,
softmax over 64 classes, then probability grouping into six displayed classes.
The adapter strictly loads the weights and runs on CPU on the original PC.

Existing launch defaults expect the original home-workspace path. For a fresh
setup, acquire the official checkpoint and verify its hash before running; do
not overwrite a working checkpoint blindly. The RELLIS launch accepts
`model_path:=...`; the current Tartan launch uses the node's default path.

