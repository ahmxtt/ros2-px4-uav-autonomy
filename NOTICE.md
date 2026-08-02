# Third-party notices

This repository builds on the following third-party software and assets.
Nothing listed here is vendored into this repository unless stated otherwise.

## Runtime dependencies (not distributed here)

| Project | Role | License |
|---|---|---|
| [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) | Middleware, `rclpy`, message types | Apache-2.0 |
| [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | Flight stack, SITL, airframe models | BSD-3-Clause |
| [px4_msgs](https://github.com/PX4/px4_msgs) | PX4 ROS 2 message definitions | BSD-3-Clause |
| [Gazebo Sim (Harmonic)](https://gazebosim.org/) | Simulator | Apache-2.0 |
| [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) | PX4 ↔ ROS 2 transport | Apache-2.0 |
| [NumPy](https://numpy.org/) | Numerics | BSD-3-Clause |
| [SciPy](https://scipy.org/) | Splines, distance transform, KD-tree | BSD-3-Clause |
| [Matplotlib](https://matplotlib.org/) | Figures | Matplotlib License (BSD-style) |
| [Pillow](https://python-pillow.org/) | PGM/PNG I/O | MIT-CMU |
| [PyYAML](https://pyyaml.org/) | Configuration and manifests | MIT |

## Gazebo Fuel models — required, not included

`sim/worlds/forest_map2.sdf` references two models by `model://` URI:

| Model | Owner | Fuel revision tested | Licence | Attribution |
|---|---|---|---|---|
| **Pine Tree** | OpenRobotics | **6** | [CC0 1.0 Universal](https://creativecommons.org/publicdomain/zero/1.0/) | Not legally required; given as courtesy |
| **Oak tree** | OpenRobotics | **7** | [CC0 1.0 Universal](https://creativecommons.org/publicdomain/zero/1.0/) | Not legally required; given as courtesy |

Source: the `OpenRobotics` collection on
[Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/fuel/models).
Licence confirmed against the Fuel API
(`GET https://fuel.gazebosim.org/1.0/openrobotics/models/{name}` →
`license_name: "Creative Commons Zero v1.0 Universal"`), and cross-checked
against the cached `model.config`, which credits Louise Poubel of the Open
Source Robotics Foundation as author.

CC0 is a public-domain dedication: **no attribution is legally required**. It
is recorded here because a reader needs to know where the scene geometry comes
from.

### Why the URI is not version-pinned

The world uses plain `model://Pine Tree` / `model://Oak tree` rather than a
versioned Fuel URL. This is deliberate:

- Gazebo resolves these names from `GZ_SIM_RESOURCE_PATH` and the local model
  path (`~/.gz/models/`), which is what the validated runs used.
- Pinning a versioned Fuel URL would make the world depend on Fuel resolution
  and would break the `GZ_FUEL_OFFLINE=1` workflow this project relies on for
  reproducibility.

Instead, the exact tested artefacts are recorded here. SHA-256 of the model
metadata as used in the validated runs:

| Model | File | SHA-256 |
|---|---|---|
| Pine Tree | `model.config` | `01d6020e5589492b84a6a36bffacdcfd68f0a18abba9e3057806e818b2966528` |
| Pine Tree | `model.sdf` | `fcee93ac7bf580598d08863349069b1973ddde7a56b381970dbf7c4137558e04` |
| Oak tree | `model.config` | `f258f8caff9d910972c484baa4624a966731efd95a06cb66e6ab1cf26134f7c7` |
| Oak tree | `model.sdf` | `a73612207941e23e3ff4dc0921847d70572460c5a9ee4b948aa8bf94a2ce2a0f` |

Note that the models' own internal `<version>` field reads `1.0`; the numbers
6 and 7 above are the **Fuel revisions**, which is what identifies them on the
server.

To obtain the tested versions:

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree"
```

Then compare the SHA-256 values above. Because both models are CC0, a
mismatch is a reproducibility concern, not a licensing one.

Their meshes and materials are **not** contained in this repository; without
them the world will not load. See
[docs/installation.md](docs/installation.md#4--gazebo-fuel-models--required).

### Derived material

The occupancy grids used for planning are rasterised from the tree **poses**
declared literally in `sim/worlds/forest_map2.sdf`, using per-species footprint
radii held as constants in `src/uav_autonomy/mapping.py`.

The canopy radii used by the execution prior (pine 1.5 m, oak 4.0 m) were
measured once from the Collada meshes of the two models above, in the
z = [1.5, 2.5] m slab. They are carried as documented constants so that this
repository need not redistribute those meshes.

Because both models are CC0, no licence obligation propagates to these derived
grids or to the figures built from them.

### First-party scene content

`sim/obstacles/srr_unknown_obstacle.sdf`, the unknown obstacle used by the
Stop-Replan-Resume (SRR) scenario, is first-party content in this repository.
It is a static cylinder built from SDF primitive geometry with an inline
material: it references no external mesh and no Fuel model, so no third-party
licence obligation attaches to it. It is not part of
`sim/worlds/forest_map2.sdf`; `tools/spawn_srr_obstacle.py` inserts it into an
already-running world.

## Airframe

The simulated airframe is PX4's `x500_lidar_2d`. PX4 ships no X650 SITL model.

The planning and collision geometry is parameterised toward the Holybro X650
target platform using radii derived from the vendor's published figures: a
650 mm wheelbase (diagonal motor-to-motor) and 15 inch propellers. Mass,
inertia, propulsion and runtime flight dynamics remain those of the PX4 X500
SITL model. The reported execution results are therefore
**X650-geometry-informed SITL results**, not an autonomous real-flight
validation of the X650.

Those two published figures give a frame radius of `0.650 / 2` = 0.3250 m, a
propeller radius of `15 * 0.0254 / 2` = 0.1905 m, and therefore a
propeller-tip envelope radius of 0.5155 m. That envelope is the smallest
circle containing the turning aircraft, so any planning radius has to exceed
it. Nothing beyond these two vendor figures is a measurement.

| Parameter | Adapted to X650 | Value | Kind |
|---|---|---|---|
| `footprint` | assumed | ±0.46 m square | **Assumption, not a measurement.** No measurement provenance exists for the half-extent, and it is not an arm-tip dimension. Documentation only: no code in this repository reads the polygon. |
| legacy nominal radius (not shipped) | **no** | 0.50 m | Historical nominal value. 0.0155 m **smaller** than the 0.5155 m propeller-tip envelope, so not a conservative propeller-tip envelope and not a conservative collision radius. Not shipped as an operational configuration; no file in this release carries it. |
| `robot_radius` (planning config, `r_eff`) | yes | 0.65 m | 0.3250 + 0.1905 + 0.1345 m safety margin. Encloses the propeller-tip envelope with 0.1345 m to spare. The radius every published planning result was computed with. |
| mass, inertia, propulsion, motor layout | **no** | X500 model | PX4 model as shipped |
| `lidar_mount.xyz` | **no** | X500 SDF | pending hardware verification |

The ±0.46 m square and `r_eff` agree arithmetically rather than
independently: the square's circumscribed radius is
`sqrt(2) * 0.46` = 0.650538 m, within 0.54 mm of 0.65 m, but 0.46 is itself
`0.65 / sqrt(2)` to two decimals. Note also that the circumscribed circle,
not the inscribed one, is the conservative bound for a square footprint;
`robot_radius` = 0.50 m does not satisfy it.

An earlier nominal radius of 0.50 m was rejected for public planning because
it does not enclose the 0.5155 m propeller-tip envelope. All published planning
results use `r_eff` = 0.65 m from `sim/config/robot_map2_safe.yaml`. The 0.50 m
value is history, not an available configuration, and the file that carried it
is not part of this release.

See `sim/config/robot_map2_safe.yaml` (the planning radius actually used),
`sim/config/mission_policy.yaml` and `docs/results.md`.
