# Traversability of POLARIS through Forests

This is my thesis project on using LiDAR and camera data to build a 3D voxel map
of off-road terrain. The aim is to use the map to study traversability for the
POLARIS vehicle in forest environments.

## What runs so far

- ROS 2 reads recorded LiDAR scans and vehicle poses, and OHM accumulates them
  into a 3D voxel map in a fixed frame. An offline mode processes every frame
  in order, and rebuilding gives identical results.
- Returns from the recording vehicle's own body (about 28% of each TartanDrive
  scan) are filtered out in the offline pipeline.
- Camera labels are fused into the voxels. The camera input keeps the image
  aspect ratio and masks the vehicle hood.
- A terrain layer gives ground height, slope, roughness, obstacle height and
  traversability classes.
- LiDAR-only features (how see-through a voxel is and its shape) help separate
  vegetation from solid obstacles.
- A water layer projects camera water pixels onto the mapped ground.
- RViz shows all of this, and `src/semantic_mapping/tools/view_demo.sh` replays
  it from a single terminal.

The mapping tests use RELLIS-3D and a short moving TartanDrive 2.0 recording.
Camera labels come from public pretrained PP-LiteSeg-B weights provided by the
GOOSE project. I have not trained a new segmentation model.

## Results so far

These come from one TartanDrive excerpt (a meadow, 318 mapped scans, about
100 m) and one labelled RELLIS-3D frame and scan.

| Check | Result |
|---|---|
| Map poses against independent GPS/INS | 0.06–0.18 m RMSE, map vertical within 0.35° |
| Ground thickness in the accumulated map | 2.8 cm (1.9 cm in a single scan) |
| Driven path wrongly marked non-traversable | 1.6% with camera + LiDAR |
| LiDAR vegetation agreement with camera labels | 80–84% |
| Labelled RELLIS-3D camera frame | pixel accuracy 0.68, mean IoU 0.36 |

The driven-path figure only tests positive examples, and GPS/INS agreement is a
comparison between two systems, not ground truth.

## What is still unfinished

- The vehicle limits are placeholders, not POLARIS values.
- LiDAR-only classes agree with human labels on 55% of RELLIS-3D voxels, and many
  tree voxels get no class. The camera evaluation on the RELLIS-3D test split is
  still to be reported.
- Camera–LiDAR alignment is not confirmed. An outline check suggests an offset of
  about 22 px (2.6°), but it comes from two places in one recording, and other
  tests were inconclusive. The camera labels use the uncorrected calibration.
- Camera water detection is unreliable, for example on dark wet soil.
- The recording has almost no trees, so forest performance is untested.
- The self-return filter is not yet in the live C++ mapper.

## Running it

The existing working workspace is `~/semantic_map_ws`. This Git checkout is
separate, at `~/Traversability-of-POLARIS-through-Forests`; changes in one folder
do not automatically change the other.

To replay the newest demo:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
src/semantic_mapping/tools/view_demo.sh explore
```

This needs the locally built results in `artifacts/`; large replay files are not
in Git. The full terminal commands are in:

- [Project guide: commands, work done and code changes](docs/PROJECT_GUIDE.md)
- [New recorded-transform calibration check](src/semantic_mapping/docs/TARTAN_CALIBRATION_CHECK.md)
- [Offline demo](src/semantic_mapping/docs/OFFLINE_DEMO.md) and [terrain analysis and validation checks](src/semantic_mapping/docs/TERRAIN_AND_VALIDATION.md)
- [LiDAR-only features: permeability, shape and the labelled RELLIS scan](src/semantic_mapping/docs/LIDAR_FEATURES.md)
- [Water on the map: camera water projected onto LiDAR ground](src/semantic_mapping/docs/WATER_DETECTION.md)
- [Accuracy v2: aspect-correct camera input, hood mask, 0.2 m / 20 m map, cloth ground, LiDAR-aware traversability](src/semantic_mapping/docs/ACCURACY_V2.md)
- [How do we know the map is right? GPS comparison, map sharpness, open checks](src/semantic_mapping/docs/MAPPING_ACCURACY.md)
- [Pictures and videos from RViz, with the camera beside the map](src/semantic_mapping/docs/RVIZ_MEDIA.md)
- [RELLIS-3D evaluation against human labels](src/semantic_mapping/docs/RELLIS_EVALUATION.md)

- [RELLIS run guide](src/semantic_mapping/docs/RUN_IT_YOURSELF.md)
- [TartanDrive run guide](src/semantic_mapping/docs/SECOND_TERRAIN_TARTAN.md)
- [Commands reference](THESIS_TERMINAL_COMMANDS_2026-10-29.txt)

To build edits from this checkout on the existing PC:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
```

The original workspace supplies the already-built OHM dependency and the model
weights. A fresh machine also needs ROS 2 Humble, dependencies, model weights
and the recordings; this repository is not a complete environment installer.

## Where the files are

- `src/semantic_mapping/`: mapping code, camera processing, launch files, RViz
  settings, tests and backups.
- `src/ohm/`: the OHM source used by the project, with its original license.
- `artifacts/`: saved reports, images and voxel/pose outputs.
- `models/goose/` and `data_provenance/`: model, calibration and recording sources.
- `legacy/original_upload/`: the ten files from the earlier GitHub upload,
  preserved without changing their contents.

The [project guide](docs/PROJECT_GUIDE.md) explains the code and remaining
tasks. [The GitHub guide](docs/GITHUB_WORKFLOW.md) explains how to edit, build,
commit and upload later changes.

Raw recordings, model weights and build files are kept outside normal Git
tracking. The [release downloads](https://github.com/Abhishekpakki01/Traversability-of-POLARIS-through-Forests/releases/tag/handoff-2026-10-29)
contain the demonstration video.

## Sources

The project uses [OHM](https://github.com/csiro-robotics/ohm),
[RELLIS-3D](https://github.com/unmannedlab/RELLIS-3D),
[TartanDrive 2.0](https://github.com/castacks/tartan_drive_2.0), and pretrained
weights from [GOOSE](https://github.com/FraunhoferIOSB/goose_dataset).
The segmentation architecture adapter is based on SuperGradients. The original
licenses and [third-party notices](src/semantic_mapping/THIRD_PARTY_NOTICES.md)
are included with the source.
