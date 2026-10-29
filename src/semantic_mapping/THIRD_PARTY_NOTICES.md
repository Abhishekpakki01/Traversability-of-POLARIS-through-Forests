# PP-LiteSeg inference adapter

`semantic_mapping/goose_model.py` adapts the inference architecture from
[Deci-AI/super-gradients](https://github.com/Deci-AI/super-gradients), licensed
under Apache-2.0. The upstream license is included as `LICENSE.super-gradients`.
The adapter removes training/framework dependencies and retains the PP-LiteSeg-B,
STDC2, SPPM, UAFM and segmentation-head parameter names and forward operations.
Upstream source snapshots and the exact tree commit are recorded locally under
`~/semantic_map_ws/models/goose/`.

The pretrained checkpoint comes from the
[official GOOSE model download page](https://goose-dataset.de/docs/setup/).
Class IDs were transcribed programmatically from tables on PDF pages 4–6 of
the [GOOSE labeling policy](https://goose-dataset.de/docs/resources/labeling_policy.pdf).
The checkpoint and labeling policy are separate from the adapter's code license.
GOOSE's repository identifies its dataset license as CC BY-SA 4.0; consult the
publisher's terms when redistributing data or model artifacts.
