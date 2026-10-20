# 6-DOF Robot — DH Kinematics and MATLAB Visualization

MATLAB implementation of the homogeneous transformation matrices. The repository demonstrates cumulative forward transformations, frame origins, and 3D coordinate-frame visualization of a six-joint manipulator.

## Scope

- Seven relative homogeneous transforms: one base offset and six joint-dependent transforms.
- Cumulative frame transforms and frame-origin positions (in **millimetres**).
- Static visualization of the zero-angle configuration.
- Optional smooth, illustrative joint-motion animation and MP4 export.

**Important:** The animation trajectories were added for visualization and are **not** trajectories computed by the original assignment. This repository does not claim inverse kinematics, collision checking, motion planning, or dynamic control. The transformations are preserved from the submitted MATLAB script; they have **not been independently checked against a physical robot or MATLAB execution**.

## Repository structure

```text
MATLAB/
  robotTransforms6DOF.m  # reusable original forward-transformation model
  plotRobot6DOF.m        # static robot and frame plot
  runStaticDemo.m        # baseline zero-angle demo
  animate6DOF.m          # illustrative animation, optional MP4 export
Report/
  Assignment_03_Report.pdf
Original/
  Assignment_03_Original.m
  Assignment_03_Maple.mw
.gitignore
README.md
```

## How to run

1. Open MATLAB and set the **Current Folder** to `MATLAB/`.
2. Run `runStaticDemo` for the original zero-joint-angle configuration and the end-effector transform.
3. Run `plotRobot6DOF([0 pi/6 -pi/4 0 pi/8 0])` to inspect a custom configuration (radians).
4. Run `animate6DOF` to display the illustrative animation and write `6DOF_DH_Kinematics_Demo.mp4` into the current MATLAB folder.
5. To preview without video export, change `exportVideo = true` to `false` at the start of `animate6DOF.m`.

The scripts use core MATLAB graphics and matrix operations. `VideoWriter(...,'MPEG-4')` support depends on the MATLAB platform/version. The original `.mw` file requires Maple to open.

## Model and visualization notes

- All translation entries in the source transforms are interpreted as **mm**, consistent with the source script's `d4 = 887` and `d6 = 200` comments.
- There are eight plotted frames: world/base, fixed base offset, and six successive transforms. Some neighboring frames may coincide, depending on joint configuration and zero-length translations.
- Coordinate axes: **red X**, **green Y**, **blue Z**.
- Joint inputs are in radians; the animation's readout is in degrees.
- The original assignment MATLAB script and report are preserved under `Original/` and `Report/` for comparison.

## Verification status

The MATLAB files have been refactored from the supplied coursework and checked structurally, but **not run in MATLAB** in this environment. Before using results in a technical publication, compare the zero-angle end-effector transform and plot with the original script and report, then test video export on your MATLAB installation.

## Suggested portfolio presentation

Show a static zero configuration, then a short animation with local coordinate frames and the end-effector path. Label it **6-DOF DH Forward-Kinematics Visualization — Illustrative Motion**.
