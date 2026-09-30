The `ros2_control` hardware interface for the LiteGrip gripper, and the exact
list of things that must be present before it will move a finger — read this
before building or deploying it.

# litegrip_ros2_control

**English** · [简体中文](README.zh-CN.md)

A `ros2_control` **SystemInterface** plugin for the LiteGrip two-finger gripper,
backed directly by the [litegrip_cpp](https://github.com/nexform-tech/litegrip-cpp)
C++ SDK. It is the L2 hardware component of the LiteGrip stack.

## Position in the stack

```text
litegrip_cpp (C++ SDK)  --linked by-->  litegrip_ros2_control  --loaded by-->  litegrip_moveit_config
```

| Layer | Package | Owns | Does not own |
| --- | --- | --- | --- |
| L3 | `litegrip_moveit_config` | semantic description (SRDF), planning group, joint limits, controller mapping, demo launch | any hardware access |
| L2 | **`litegrip_ros2_control`** (this package) | URDF parameter parsing, the `ControlLoop` lifecycle, the `m` ⇄ `rad` conversion, the state interfaces | rate limiting, safety decisions, real-time control |
| L1 | `litegrip_cpp` | SocketCAN, the DM MIT protocol, the safety gate, the trajectory rate ceiling, the torque budget, the watchdogs, the frame stream | it does not know ROS exists |

Everything the earlier Python daemon did now runs inside
`litegrip_cpp::ControlLoop` on the SDK's own thread: one process, no shared
memory, no Python, no ctypes layout to keep in sync. `read()` and `write()` touch
neither CAN nor the allocator — they copy a cached snapshot and post one target,
so the `controller_manager` cycle stays cheap no matter how the bus is behaving.

**This package cannot move a gripper on its own.** It ships no launch file, no
URDF geometry and no controller implementation. The sections below state exactly
what it needs.

## Dependencies

### Build and link dependencies

| Dependency | Kind | Why this component needs it | If it is missing |
| --- | --- | --- | --- |
| `litegrip_cpp` | build, link, runtime | the whole control path: SocketCAN, DM MIT framing, the safety gate, the rate ceiling, the torque budget, the watchdogs | `find_package(litegrip_cpp REQUIRED)` fails and the package does not configure |
| `hardware_interface` | build | `hardware_interface::SystemInterface`, the base class, and `HardwareInfo` | no base class to implement |
| `pluginlib` | build | exports `litegrip_ros2_control/LitegripSystem` (see `litegrip_ros2_control.xml`) | `controller_manager` cannot load the component |
| `rclcpp`, `rclcpp_lifecycle` | build | the logger, and the lifecycle state each callback receives | the same |
| `ament_cmake` | build tool | the build type declared in `package.xml` | colcon cannot build the package |
| `ament_cmake_gtest` | test | the 19-case test suite | `colcon test` cannot build the tests |

### Runtime ROS dependencies

| Dependency | Why | If it is missing |
| --- | --- | --- |
| `controller_manager` | loads, configures, activates and deactivates the hardware component | nothing loads the plugin; the gripper never appears |
| `joint_state_broadcaster` | publishes `/joint_states` from the state interfaces, which is what puts the gripper into TF and MoveIt | the gripper joint is invisible outside ros2_control |
| `gripper_controllers` | provides `position_controllers/GripperActionController`, the type `config/litegrip_controllers.yaml` configures | the shipped controller configuration has no implementation to load |
| `robot_state_publisher`, `xacro` | expand the URDF that includes `urdf/litegrip.ros2_control.xacro` | the `<ros2_control>` block never reaches `controller_manager` |
| `launch`, `launch_ros` | a launch file that passes `config/litegrip_controllers.yaml` to `controller_manager` as a params file | the controller configuration has to be passed by hand |

### `litegrip_cpp` is a source dependency, not a packaged one

The SDK is plain CMake rather than ament, and is published to no package index.
It must come from source, in one of two ways:

- **Same colcon workspace** — check it out next to this package and let colcon
  order the build from `package.xml`. This is what CI does: the `test` job in
  [.github/workflows/ci.yml](.github/workflows/ci.yml) checks out
  `nexform-tech/litegrip-cpp` into the workspace and builds
  `colcon build --packages-select litegrip_cpp litegrip_ros2_control`.
- **An installed prefix** — build the SDK with its own CMake and install it;
  `find_package(litegrip_cpp REQUIRED)` then resolves
  `litegrip_cpp-config.cmake` through the normal CMake package mechanism.

Do not expect `rosdep` to fetch it. `rosdep install --from-paths src --ignore-src`
resolves the ROS dependencies listed above from the manifests, but the SDK has no
package-index entry of its own.

### Runtime data: the safety baseline, and how it is found

`litegrip_cpp` installs its data under `<prefix>/share/litegrip_cpp/calibration`
(`safety_limits_350.json`, `safety_limits_025.json`, `factory_calibration.json`)
and exports that directory as `litegrip_cpp_DATA_DIR`. This package's CMake bakes
it in as the compile definition `LITEGRIP_DEFAULT_DATA_DIR`, and `on_init`
publishes it as the `LITEGRIP_DATA_DIR` environment variable. Resolution order,
strongest first:

| # | Mechanism | Set by |
| --- | --- | --- |
| 1 | a `safety_baseline` parameter that contains a path (it holds a `/`) | the URDF or launch that deploys the gripper |
| 2 | the `LITEGRIP_DATA_DIR` environment variable | the deployment (`setenv` is called with `overwrite=0`, so this value is never clobbered) |
| 3 | the build-time `LITEGRIP_DEFAULT_DATA_DIR` | this component, automatically |

**Do not rely on the working directory.** A bare version name such as `3.5`
resolves only relative to the process's working directory, and a
`controller_manager` is started from wherever the user happens to be. A missing
or unknown baseline is fail-closed — the component refuses to start rather than
fall back to limits nobody confirmed.

The conversion parameters `closed_rad` and `rad_to_mm` are component parameters
whose defaults in the xacro (`0.04139`, `65.0231`) are copies of that unit's
`factory_calibration.json`. After a recalibration, update the xacro values from
the file; the two must not drift apart.

### Hardware

Needed only when `dry_run` is false; none of it is touched in the default
configuration.

| Item | Value |
| --- | --- |
| Bus | SocketCAN `can0`, Classic CAN at 1 Mbit/s (CAN-FD off) |
| Access | raw-socket permission for the user running `controller_manager` (`CAP_NET_RAW`, root, or a udev/capability rule) |
| Addressing | command id `8`, status (master) id `24` |
| Motor | Damiao DM-J4310-2EC, MIT mode |

Three parameters decide whether a real gripper moves at all, and each blocks
something different:

- `dry_run` (default `true`) — "do not touch hardware while debugging".
- `hardware_enable` (default `false`) — "is a gripper actually attached to this
  machine". Both must be switched for the real-hardware path.
- `max_feedback_velocity_rad_s` (default `-1.0`) — `-1` means **not given**,
  which makes the loop **refuse to send any motion frame**. It must be measured
  on the real hardware before the gripper will move, even with both switches set.

### What this package deliberately does not contain

- **No geometry.** The joint name (`gripper_opening_joint`), the two fingers and
  their `<mimic>` come from the description package, `litegrip_urdf` (note: the
  package name differs from its directory name, `litegrip_description`).
- **No launch file.** Deploy through a stack that brings up `controller_manager`
  — `litegrip_moveit_config` is the one in this organisation.
- **More than one joint is rejected.** A real LiteGrip has a single
  transmission and the two fingers are `<mimic>`, so a `<ros2_control>` block
  that declares two joints fails `on_init` by design.

## Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/nexform-tech/litegrip-ros2
git clone https://github.com/nexform-tech/litegrip-cpp litegrip_cpp

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y     # the ROS dependencies; not litegrip_cpp
colcon build --packages-select litegrip_cpp litegrip_ros2_control
source install/setup.bash
```

Target platform: ROS 2 Humble Hawksbill on Ubuntu 22.04 (x86_64), C++17.

## Interfaces

| Joint | Command interfaces | State interfaces |
| --- | --- | --- |
| `gripper_opening_joint` | `position` | `position`, `velocity`, `effort`, `temperature_mos`, `temperature_coil`, `error_code`, `fault_code`, `feedback_age` |

- **Command exposes `position` only.** The trajectory rate ceiling and the torque
  budget are deployment parameters of this component, not fields of a command:
  they describe how this gripper is allowed to move, so a command publisher must
  not be able to decide them.
- **State exposes the standard trio** — the same three interfaces the arm joints
  expose, so `joint_state_broadcaster` treats both identically — **plus
  diagnostics.** `joint_state_broadcaster` does not claim the diagnostics, and
  `controller_manager` warns on activation that they are unclaimed; that is
  expected. Set `export_diagnostics` to `false` to drop the last five (3 state
  interfaces remain).
- The two fingers are `<mimic>` and appear nowhere here: a real gripper has one
  transmission.

## Parameters

Every `<param>` in `urdf/litegrip.ros2_control.xacro` is fed by a xacro argument
of the same name with a `litegrip_` prefix, so a launch file can override any of
them without editing the URDF.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `channel` | `can0` | SocketCAN interface |
| `can_id` / `mst_id` | `8` / `24` | command id / status frame id |
| `canfd_mode` | `false` | Classic CAN |
| `dry_run` | `true` | run the whole control path against a simulated plant; skip opening CAN and sending |
| `hardware_enable` | `false` | the real-hardware switch |
| `control_rate_hz` | `200.0` | the SDK thread's frame rate |
| `feedback_timeout_s` | `0.5` | how long without a *new* status frame counts as communication loss |
| `temperature_limit_c` | `80` | stop before the driver trips its own overtemperature fault |
| `command_timeout_s` | `0.2` | stale-command criterion: hold position once no new command arrives for this long |
| `max_velocity_rad_s` | `1.5` | trajectory advance rate ceiling; can only be turned down, never up |
| `torque_limit_nm` | `3.5` | total torque budget, allocated between the position gains per frame; can only be turned down |
| `safety_baseline` | `3.5` | a versioned baseline name, or a path to a baseline file |
| `max_position_error_rad` | `-1.0` | `-1` derives it from the width of the red lines |
| `max_feedback_velocity_rad_s` | `-1.0` | `-1` means not given, which refuses to send any motion frame |
| `kp` / `kd` | `20.0` / `0.5` | upper bounds; each frame re-allocates them from the budget |
| `closed_rad` / `rad_to_mm` | `0.04139` / `65.0231` | this unit's calibration |
| `min_width` / `max_width` | `0.0` / `0.087` | model-layer opening range in m, matching the URDF joint limits |
| `export_diagnostics` | `true` | export the five diagnostic state interfaces |

`dry_run` is not a no-op: the SDK runs the entire control path — rate limiting,
torque-budget allocation, the safety gate, the watchdogs — against a simulated
plant, and only skips opening CAN and sending frames. That is why a
configuration error shows up in a dry run without a gripper attached.

## Units, sign and clamping

One number, one unit:

| Side | Unit |
| --- | --- |
| ROS interface storage | opening in **m** (aligned with the URDF joint limits) |
| SDK | motor angle in **rad** (the red lines, the torque budget and the MIT quantization are defined and verified in rad) |

```text
opening [mm] = (closed_rad - rad) x rad_to_mm
rad          = closed_rad - opening [mm] / rad_to_mm
```

**The more negative `rad` is, the wider the opening**, so the velocity
conversion must flip sign:

```text
opening velocity [m/s] = -v_rad x rad_to_mm x 1e-3
```

This is the one inconspicuous-but-wrong line in the component: drop the minus
sign and the position stays correct while the velocity is inverted. Velocity
feedback is what decides "stopped or not", so an inverted sign raises no error —
it quietly makes that decision wrong.

**Do not remove the clamping** in `width_to_rad()`. The model-layer range is
`0` to `87 mm`, while the commandable range is narrower because the red lines
bound it, and the SDK **rejects** an out-of-range target rather than clamping it.
Without clamping here, an ordinary "open fully" command would be dropped whole,
the gripper would not move, and all that would be left behind is a fault code.
The commandable range is derived from the red lines the SDK actually loaded
(`loop_->safety_limits()`), not from a second copy in the URDF.

## Deploying it

Include the hardware block from the description package's URDF:

```xml
<xacro:include filename="$(find litegrip_ros2_control)/urdf/litegrip.ros2_control.xacro"/>
```

Then load `config/litegrip_controllers.yaml` as an **extra params file**
alongside the arm's controller configuration. The component must already be
loaded when the controllers are spawned — that is, the description file has to
include the xacro above — or the spawner will not find
`gripper_opening_joint`'s interfaces.

The shipped controller is `position_controllers/GripperActionController` on the
single joint, exposing `control_msgs/action/GripperCommand` on
`/gripper_controller/gripper_cmd` (not `FollowJointTrajectory`). Stalling is
configured as success, because "closed onto the object and stopped" is the
desired outcome of a grasp. Anything hard-wired to a trajectory action name must
be updated.

## Testing

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select litegrip_ros2_control
colcon test-result --verbose
```

Result on ROS 2 Humble / Ubuntu 22.04 x86_64, 2026-09-30:

```text
Summary: 39 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result` counts result entries, one of which each gtest binary adds
for itself; the suite itself is **19 gtest cases** in `test_litegrip_system`,
reported by the test's own XML as `tests="19" failures="0"`. Every case runs with
`dry_run=true`, driving the component's lifecycle in-process with a synthetic
`HardwareInfo` and no CAN socket, which is what makes parameter validation, the
`m` ⇄ `rad` conversion, the clamping and the activation latch testable without a
gripper.

## Known gaps that affect deployment

| Gap | Effect |
| --- | --- |
| The shipped red lines are a reference machine's hand-measured values | re-measure on your own unit before commanding real motion, `max_feedback_velocity_rad_s` included |
| There is no controlled software path back inside the red lines | a gripper stopped outside them — fully closed is one such position — cannot be recovered by this stack yet |
| Diagnostic interfaces are exported but unclaimed | a `fault_code` is visible only in the node log (`grep "gripper fault code"`), not on a topic |
| The transport send/receive path has no test coverage | it needs a vcan interface (root) or the real device |

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK — this package's hard dependency |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 driver |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |

## License

The repository ships the Apache License 2.0 in [LICENSE](../LICENSE);
`package.xml` still declares `BSD-3-Clause`, which is a leftover from the
package template and not the licence this repository is licensed under.
