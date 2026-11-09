# Sparse-voxel fill: plan fixed before testing (9 November 2026)

Diagnosis (sequences 00000 and 00001 only): tree voxels are unknown mainly because they get
fewer than 3 points (39% of tree voxels). Three times more scans only cut unknowns from 48% to 40%.

Rule tested: an unknown voxel with fewer than 3 points, or fewer than 5 hits plus passes, takes
the most common known class of its 26 neighbours, if at least 3 neighbours have a known class.

Decision: adopt it only if, on development sequences 00000 and 00001, more tree voxels get an
accepted class and the overall consistent fraction does not drop. Sequences 00002 to 00004 are
then scored once and reported whatever the result. No settings are changed after seeing them.
