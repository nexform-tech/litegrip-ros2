# litegrip-ros2

This document is for a first-time ROS 2 user: it explains what the LiteGrip ROS 2
driver is, how to build it, and how to use it to control the gripper.

**English** · [简体中文](README.zh-CN.md)

ROS 2 driver for the **LiteGrip** adaptive two-finger gripper. The repository
contains one package, `litegrip_ros2_control`: a `ros2_control` hardware plugin
backed directly by the [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp)
SDK.

```text
litegrip_cpp (C++ SDK)  --linked by-->  litegrip_ros2_control  --loaded by-->  litegrip_moveit_config
```

| Package | Type | Role |
| --- | --- | --- |
| `litegrip_ros2_control` | ament_cmake | `ros2_control` `SystemInterface` plugin. The SDK's `ControlLoop` owns the CAN device, the rate ceiling, the torque budget, the safety gate and the frame stream; the plugin parses the URDF parameters, drives the loop's lifecycle, converts metres to radians, and exposes the state interfaces |

The package cannot move a gripper on its own. It ships no launch file, no URDF
geometry and no controller implementation. The sections below show the two ways
to use it: with the ready-made MoveIt demo, or inside an existing robot's
`controller_manager`.

## 1. Prepare the environment

### 1.1 Requirements

| | |
| --- | --- |
| Operating system | Ubuntu 22.04 (x86_64) |
| ROS 2 | Humble Hawksbill |
| Hardware | Optional. Dry run exercises the full control path without a gripper |

### 1.2 Install ROS 2 Humble

If ROS 2 is not installed yet:

```bash
sudo apt update && sudo apt install -y curl software-properties-common
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

Add the ROS environment to `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.3 Install dependencies

For the driver package:

```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gripper-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-position-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

For the full demo described in section 5, also install MoveIt:

```bash
sudo apt install -y ros-humble-moveit ros-humble-moveit-configs-utils ros-humble-rviz2
```

### 1.4 About Conda

If Conda is active, leave it first:

```bash
conda deactivate
```

Conda's Python shadows the system interpreter, and `colcon build` fails with
`ModuleNotFoundError: No module named 'catkin_pkg'`.

## 2. Create the workspace and fetch the repositories

### 2.1 Driver only

```bash
mkdir -p ~/litegrip_ws/src
cd ~/litegrip_ws/src

git clone https://github.com/nexform-tech/litegrip-cpp.git
git clone https://github.com/nexform-tech/litegrip-ros2.git
```

### 2.2 Full stack, including the demo

Add the description and the MoveIt configuration:

```bash
cd ~/litegrip_ws/src
git clone https://github.com/nexform-tech/litegrip-description.git
git clone https://github.com/nexform-tech/litegrip-moveit2.git
```

The MoveIt configuration requires a description that provides a single master
joint `gripper_opening_joint` with both fingers as `<mimic>` followers, and that
accepts `include_mock_ros2_control`. Check your copy:

```bash
grep -n "gripper_opening_joint\|include_mock_ros2_control" \
  ~/litegrip_ws/src/litegrip-description/litegrip_description/urdf/litegrip_description.urdf.xacro
```

If the grep prints nothing, the description is incompatible: the demo starts,
but `move_group` reports the `gripper` planning group as empty. See the
[MoveIt configuration README](https://github.com/nexform-tech/litegrip-moveit2)
for details.

## 3. Build

For the full stack:

```bash
cd ~/litegrip_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to litegrip_moveit_config
source install/setup.bash
```

For the driver only:

```bash
cd ~/litegrip_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select litegrip_cpp litegrip_ros2_control
source install/setup.bash
```

Run `source install/setup.bash` again in every new terminal.

## 4. Test

```bash
cd ~/litegrip_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select litegrip_cpp litegrip_ros2_control
colcon test-result --verbose
```

The `litegrip_ros2_control` tests drive the component's lifecycle directly with
a synthetic `HardwareInfo`, in dry run, so they need no CAN interface and no
gripper.

## 5. Start the stack and control the gripper

The ready-made full stack lives in `litegrip-moveit2`:

```bash
ros2 launch litegrip_moveit_config demo.launch.py
```

The default is **dry run**: no CAN socket is opened and no motor is touched, but
the SDK runs its entire control path against a simulated plant.

When it comes up you should see the gripper in RViz, the MotionPlanning panel's
planning group set to `gripper`, and both controllers `active`. Check from
another terminal:

```bash
source ~/litegrip_ws/install/setup.bash

ros2 control list_controllers
ros2 action list -t | grep gripper
ros2 topic echo /joint_states --once
```

Control the gripper through the action:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 3.5}}" --feedback
```

| Field | Meaning |
| --- | --- |
| `position` | Opening in metres. 0 is fully closed, 0.085452 is fully open |
| `max_effort` | Maximum effort in N·m. Used only by the controller's stall detection; the real torque budget is enforced by the SDK |

`--feedback` prints `reached_goal` and `stalled`. When the gripper closes onto an
object and stops, `stalled=true` is a normal success.

You can also drive the gripper from RViz: the MotionPlanning panel's **Joints**
tab has a `gripper_opening_joint` slider, and the **Goal State** list contains
`open` and `closed`.

## 6. Use it with an existing arm

The plugin is a `ros2_control` hardware component, so it can share the arm's
`controller_manager`.

1. Include the hardware block in the robot URDF:

   ```xml
   <xacro:include filename="$(find litegrip_ros2_control)/urdf/litegrip.ros2_control.xacro"/>
   ```

2. Load `config/litegrip_controllers.yaml` as an extra params file alongside the
   arm's controller configuration.

3. Spawn `gripper_controller`. The hardware component must already be loaded when
   the spawner runs, or the spawner will not find `gripper_opening_joint`'s
   interfaces.

The shipped controller is `position_controllers/GripperActionController` on the
single joint. It exposes `control_msgs/action/GripperCommand` on
`/gripper_controller/gripper_cmd`, not `FollowJointTrajectory`.

A `<ros2_control>` block that declares more than one joint is rejected at
`on_init` by design: a real LiteGrip has one transmission, and the two fingers
are `<mimic>`.

## 7. Connect real hardware

In real-hardware mode the gripper moves. For the first attempt, clear the finger
travel and keep your hands away from the gripper.

### 7.1 Configure CAN

```bash
sudo apt install -y can-utils
sudo modprobe gs_usb

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

Confirm that the output shows `state UP` and `bitrate 1000000`. The process
running `controller_manager` needs raw-socket permission (`CAP_NET_RAW`, root, or
a udev/capability rule).

### 7.2 Start with both switches

```bash
ros2 launch litegrip_moveit_config demo.launch.py \
  dry_run:=false hardware_enable:=true \
  max_feedback_velocity_rad_s:=0.9
```

`dry_run` and `hardware_enable` must both be given. With either one missing, the
gripper does not move.

`0.9` is an example. `max_feedback_velocity_rad_s` must be calibrated on your
unit: run it through real working conditions, take the peak feedback velocity,
and add margin. The default `-1.0` means **not given**, and the control loop
refuses to send any motion frame. That is deliberate fail-closed behaviour.

### 7.3 Safety baseline and calibration

The red lines in the safety baseline are the reference unit's measurements, not
your unit's. If your calibration puts the closed end outside those lines, the
gripper refuses all motion until the lines are re-derived. The `closed_rad` and
`rad_to_mm` defaults in the xacro are copies of this unit's
`factory_calibration.json`; after a recalibration, update the xacro values from
the file.

## 8. Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `colcon build` fails with `ModuleNotFoundError: No module named 'catkin_pkg'` | Conda's Python shadows the system interpreter. Run `conda deactivate` and build again |
| `controller_manager` cannot load `litegrip_ros2_control/LitegripSystem` | The package was not sourced in this shell, or the workspace was not built. Source `install/setup.bash` |
| The hardware component fails `on_init` with "supports 1 joint only" | The `<ros2_control>` block declares more than one joint. The fingers must be `<mimic>`, not separate command joints |
| The hardware component refuses to start with a safety baseline error | The baseline file cannot be found. Set `LITEGRIP_DATA_DIR` or pass an explicit path in `safety_baseline` |
| The gripper does not move in real-hardware mode | `max_feedback_velocity_rad_s` is missing or wrong, or one of the two switches is missing. Read the launch warnings |
| `gripper_cmd` is missing from `ros2 action list` | The controller did not start. Run `ros2 control list_controllers` and `ros2 control list_hardware_interfaces` |

## 9. Package documentation

The package README is the reference for the dependency list, every parameter,
the unit conversion, the clamping rules and the test suite:

- [litegrip_ros2_control/README.md](litegrip_ros2_control/README.md)
- [litegrip_ros2_control/README.zh-CN.md](litegrip_ros2_control/README.zh-CN.md)

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-description](https://github.com/nexform-tech/litegrip-description) | Robot description |
| [litegrip-moveit2](https://github.com/nexform-tech/litegrip-moveit2) | MoveIt configuration and demo |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 driver |

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
