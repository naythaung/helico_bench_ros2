# CAD Integration

This document records how CAD from the Helico bench Onshape assembly is incorporated into the ROS 2 bench model.

The aim is to keep the original CAD export traceable while giving the runtime ROS model clear, functional names that are easier to understand and maintain.

---

## Link 6 instrument mapping

| Functional component | ROS link | Runtime mesh | Original Onshape export mesh | RViz colour |
|---|---|---|---|---|
| Main tool mount | `tools_mount` | `tools_mount.stl` | `Part_1_2.stl` | Purple |
| Laser sensor mount | `laser_sensor_mount` | `laser_sensor_mount.stl` | `Part_1.stl` | Orange |
| Laser displacement sensor | `laser_sensor` | `laser_sensor.stl` | `_8777405.stl` | Light blue |
| Force sensor | `force_sensor` | `force_sensor.stl` | `Force_sensor.stl` | Light grey |
| Force poker | `force_poker` | `force_poker.stl` | `Part_1_1.stl` | Green |
| AprilTag mount | `april_tag_mount` | `april_tag_mount.stl` | `Part_1_3.stl` | Yellow |

### Current RViz colours

```text
tools_mount         = 0.682353 0.333333 0.737255 1
laser_sensor_mount  = 0.729412 0.250980 0.105882 1
laser_sensor        = 0.615686 0.811765 0.929412 1
force_sensor        = 0.901961 0.901961 0.901961 1
force_poker         = 0.282353 0.549020 0.160784 1
april_tag_mount     = 0.980392 0.713726 0.00392157 1
```


---

## Directory structure

The project intentionally keeps the original CAD export separate from the runtime ROS assets.

```text
cad/onshape_reference/URDF_OnshapeBench/
├── meshes/
└── urdf/
    └── assembly.urdf
```

This directory is treated as the **raw CAD reference** and should normally remain unchanged.

```text
src/meca500_scene_description/
├── meshes/
│   └── onshape/
└── urdf/
    └── scene.urdf.xacro
```

This directory contains the **curated ROS runtime geometry** used by RViz and MoveIt.

The runtime meshes may be renamed to functional names, provided the original CAD provenance is recorded in this document.

> Do not manually edit generated files under `install/`.  
> Rebuild using `colcon build --symlink-install` instead.

---

## Naming convention

The project uses different naming conventions for raw CAD and ROS runtime assets.

```text
Raw CAD name     = provenance
ROS link name    = physical / functional meaning
Runtime STL name = physical / functional meaning
```

For example:

```text
Original CAD mesh:  Part_1_2.stl
Runtime mesh:       tools_mount.stl
ROS link:           tools_mount
```