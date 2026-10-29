# Original GitHub upload

`original_upload/` preserves all ten files that were present at commit
`78424146926550e2775c847970385b1fa6030156` before the complete workspace snapshot
was added. They were moved without changing their file contents.

The old flattened layout is historical reference. The current ROS package is
`src/semantic_mapping/`. `COLCON_IGNORE` prevents this historical package.xml
from being discovered as a duplicate package during a workspace build.

