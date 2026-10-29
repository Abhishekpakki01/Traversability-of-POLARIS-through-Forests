# OHM source snapshot

`src/ohm/` is an unchanged source snapshot from
<https://github.com/csiro-robotics/ohm>, commit
`4e2e76945e550eb8a96cef9c4434e7c1811dd030`. The original checkout was clean when
this snapshot was prepared. Git internals and compiled files are excluded.
Keep its original license and third-party notices with the source.

The current mapper uses OHM's CPU `RayMapperOccupancy` API. The original local
build cache had CUDA OFF, OpenCL ON, OHM tests OFF and heightmap support ON.
Those build capabilities do not mean the thesis application currently estimates
ground-relative height or uses a GPU mapper. On the existing PC, the original
workspace supplies the already-built OHM dependency when overlay-building the
semantic package. A fresh machine still needs the dependencies/build steps
documented by OHM and ROS.
