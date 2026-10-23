# Notice

This repository (`so101_ros2`) is licensed under the BSD-3-Clause license; see
[LICENSE](LICENSE). It includes material from the following third-party project,
which remains under its own license.

## TheRobotStudio / SO-ARM100

- Source: <https://github.com/TheRobotStudio/SO-ARM100>
- License: **Apache License 2.0** (`Apache-2.0`), as stated in the repository's
  [LICENSE](https://github.com/TheRobotStudio/SO-ARM100/blob/main/LICENSE) file
  (verified against upstream commit `a758567c3978dfeefe282ede0500085a48fe8f78`).
- Copyright: TheRobotStudio and SO-ARM100 contributors.

What is used here:

- **Meshes:** `so101_description/meshes/so_arm101/*.stl` and `*.part` come from
  `Simulation/SO101/assets/` of SO-ARM100 and are unmodified.
- **Kinematic and inertial data:** the link and joint origins, masses and
  inertias in `so101_description/urdf/robot.urdf` and
  `so101_description/urdf/so101_robot.urdf.xacro` derive from
  `Simulation/SO101/so101_new_calib.urdf` of SO-ARM100 (generated upstream from
  the Onshape CAD with onshape-to-robot). Changes made here: conversion to a xacro macro,
  `package`/`file://` mesh paths, the optional `world`/mount joint, name
  prefixing, Gazebo collision primitives, transmissions and ros2_control
  blocks.

These files are distributed under the terms of the Apache License 2.0. The
upstream license text is included at
[so101_description/meshes/so_arm101/LICENSE](so101_description/meshes/so_arm101/LICENSE)
(installed with the meshes) and is also available at
<https://www.apache.org/licenses/LICENSE-2.0>. The BSD-3-Clause license of
this repository does not apply to them.
