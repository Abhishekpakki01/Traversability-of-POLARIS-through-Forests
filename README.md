# Traversability of POLARIS through Forests

This is my thesis project on using LiDAR and camera data to build a 3D voxel map
of off-road terrain. The aim is to use the map to study traversability for the
POLARIS vehicle in forest environments.

## What runs so far

- ROS 2 reads recorded LiDAR scans and vehicle poses.
- OHM accumulates the accepted scans in a fixed coordinate frame.
- RViz shows occupied voxels, the vehicle pose and camera images.
- An initial camera-label fusion node is included. It still needs alignment and
  accuracy checks.

The mapping tests use RELLIS-3D and a short moving TartanDrive 2.0 recording.
Camera labels come from public pretrained PP-LiteSeg-B weights provided by the
GOOSE project. I have not trained a new segmentation model.

## What is still unfinished

A separate calibration check now uses the mounting transforms from the original
TartanDrive recording. It also avoids adding a rectification rotation from a
later example. The old version is kept for comparison. The camera matrix has
been cross-checked against the recorded stereo output, but physical alignment
and the resulting voxel labels still need validation.

There are also dropped frames and limited semantic coverage. In an earlier
Tartan run, 569 of 24,888 occupied voxels received a confirmed class. Most voxels
remain gray because their class is unknown. Water/trunk recognition, height
above local ground, slope, clearance and navigation still need work.

The next step is to verify the calibration and projection on the exact
recording, then check frame losses and repeat the test over a longer sequence.
The current results are an initial integration test.

## Running it

The existing working workspace is `~/semantic_map_ws`. This Git checkout is
separate, at `~/Traversability-of-POLARIS-through-Forests`; changes in one folder
do not automatically change the other.

The full terminal commands are in:

- [New recorded-transform calibration check](src/semantic_mapping/docs/TARTAN_CALIBRATION_CHECK.md)

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

[The handoff file](THESIS_HANDOFF_2026-10-29.md) explains the code and remaining
tasks. [The GitHub guide](docs/GITHUB_WORKFLOW.md) explains how to edit, build,
commit and upload later changes.

Raw recordings, model weights and build files are kept outside normal Git
tracking. The [release downloads](https://github.com/Abhishekpakki01/Traversability-of-POLARIS-through-Forests/releases/tag/handoff-2026-10-29)
contain the demonstration video and portable handoff ZIP.

## Sources

The project uses [OHM](https://github.com/csiro-robotics/ohm),
[RELLIS-3D](https://github.com/unmannedlab/RELLIS-3D),
[TartanDrive 2.0](https://github.com/castacks/tartan_drive_2.0), and pretrained
weights from [GOOSE](https://github.com/FraunhoferIOSB/goose_dataset).
The segmentation architecture adapter is based on SuperGradients. The original
licenses and [third-party notices](src/semantic_mapping/THIRD_PARTY_NOTICES.md)
are included with the source.
