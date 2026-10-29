# Sensor data and large local assets

The Git repository contains source/calibration records and bag metadata, not
raw multi-GB sensor payloads. The existing local recordings are unchanged.

| Asset | Original local path |
|---|---|
| RELLIS example, about 60 s | `~/RELLIS-3D/example_filtered_ros2/` |
| Tartan moving excerpt, about 32 s | `~/thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2/` |
| Tartan source/calibration metadata | `~/thesis_data/tartan_drive_2_sample/source_metadata/` |
| RViz demonstration video | `~/Different environment testing.mp4` |
| Portable source/evidence ZIP | `~/THESIS_HANDOFF_2026-10-29.zip` |

`local_large_assets.json` records exact current sizes and whether each asset is
Git-tracked. The raw RELLIS database is about 8.91 GB and the selected Tartan
database about 797 MB. They are not included in normal Git commits. The trained
weights are recovered from the official model source in `models/goose/README.md`.

The Tartan excerpt came from the publisher's
`2023-11-02-16-00-43_down_meadows/down_meadows_0.bag`, selecting source offset
45–75 seconds with synchronization margins. The source is about 32.50 GB;
selective HTTP range extraction transferred about 940 MB. The pinned extractor
and exact recovery command are in the handoff and Tartan guide.

Important limitations: CameraInfo was generated from publisher S21 calibration;
the chosen optical extrinsic differs from the bag's default URDF. Pose TF was
generated from recorded odometry with the original pose timestamps and offline
dispatch timing. These are documented assumptions, not established online
latency or calibration-accuracy results. Preserve the provenance when presenting
or reusing the outputs.

