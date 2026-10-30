本文件面向第一次使用 ROS 2 的人，说明 LiteGrip 的 ROS 2 驱动是什么、怎么编译，以及怎么用它控制夹爪。

[English](README.md) · **简体中文**

# litegrip-ros2

**LiteGrip** 自适应两指夹爪的 ROS 2 驱动。仓库里只有一个包 `litegrip_ros2_control`：一个由 [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) SDK 直接支撑的 `ros2_control` 硬件插件。

```text
litegrip_cpp（C++ SDK）  --链接-->  litegrip_ros2_control  --加载-->  litegrip_moveit_config
```

| 包 | 类型 | 作用 |
| --- | --- | --- |
| `litegrip_ros2_control` | ament_cmake | `ros2_control` 的 `SystemInterface` 插件。SDK 的 `ControlLoop` 负责 CAN 设备、限速、力矩预算、安全闸和帧流；插件负责解析 URDF 参数、驱动控制环生命周期、把米换算成弧度，并导出状态接口 |

这个包不能单独让夹爪动起来。它不带 launch、不带 URDF 几何、也不带控制器实现。下面给出两种用法：配合现成的 MoveIt 演示，或接进已有机械臂的 `controller_manager`。

## 1. 准备环境

### 1.1 系统要求

| | |
| --- | --- |
| 操作系统 | Ubuntu 22.04（x86_64） |
| ROS 2 | Humble Hawksbill |
| 硬件 | 可选。dry run 没有夹爪也能跑完整条控制链路 |

### 1.2 安装 ROS 2 Humble

如果机器上还没有 ROS 2：

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

把 ROS 环境写进 `~/.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.3 安装依赖

驱动包本身需要：

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

第 5 节的全栈演示还需要 MoveIt：

```bash
sudo apt install -y ros-humble-moveit ros-humble-moveit-configs-utils ros-humble-rviz2
```

### 1.4 关于 Conda

如果启用了 Conda，先退出：

```bash
conda deactivate
```

Conda 的 Python 会顶掉系统 Python，导致 `colcon build` 报 `ModuleNotFoundError: No module named 'catkin_pkg'`。

## 2. 创建工作区并获取仓库

### 2.1 只要驱动

```bash
mkdir -p ~/litegrip_ws/src
cd ~/litegrip_ws/src

git clone https://github.com/nexform-tech/litegrip-cpp.git
git clone https://github.com/nexform-tech/litegrip-ros2.git
```

### 2.2 全栈，含演示

再加上 description 和 MoveIt 配置：

```bash
cd ~/litegrip_ws/src
git clone https://github.com/nexform-tech/litegrip-description.git
git clone https://github.com/nexform-tech/litegrip-moveit2.git
```

MoveIt 配置要求 description 提供单个主关节 `gripper_opening_joint`，两指是它的 `<mimic>` 跟随关节，并且接受 `include_mock_ros2_control`。检查已拉取的版本：

```bash
grep -n "gripper_opening_joint\|include_mock_ros2_control" \
  ~/litegrip_ws/src/litegrip-description/litegrip_description/urdf/litegrip_description.urdf.xacro
```

输出为空表示不兼容：演示仍会启动，但 `move_group` 会把 `gripper` 规划组当作空组。详见 [MoveIt 配置仓库的 README](https://github.com/nexform-tech/litegrip-moveit2)。

## 3. 编译

全栈：

```bash
cd ~/litegrip_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to litegrip_moveit_config
source install/setup.bash
```

只要驱动：

```bash
cd ~/litegrip_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select litegrip_cpp litegrip_ros2_control
source install/setup.bash
```

每开一个新终端，都要重新执行一次 `source install/setup.bash`。

## 4. 测试

```bash
cd ~/litegrip_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select litegrip_cpp litegrip_ros2_control
colcon test-result --verbose
```

`litegrip_ros2_control` 的测试用合成 `HardwareInfo` 直接驱动组件生命周期，全程 dry run，不需要 CAN 接口，也不需要夹爪。

## 5. 启动整条栈并控制夹爪

现成的全栈在 `litegrip-moveit2` 里：

```bash
ros2 launch litegrip_moveit_config demo.launch.py
```

默认是 **dry run**：不打开 CAN 套接字，不碰电机，但 SDK 会针对一个仿真被控对象跑完整条控制链路。

启动后应该看到 RViz 里的夹爪、MotionPlanning 面板的规划组 `gripper`，以及两个都是 `active` 的控制器。另开一个终端检查：

```bash
source ~/litegrip_ws/install/setup.bash

ros2 control list_controllers
ros2 action list -t | grep gripper
ros2 topic echo /joint_states --once
```

通过 action 控制夹爪：

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 3.5}}" --feedback
```

| 字段 | 含义 |
| --- | --- |
| `position` | 开口，单位米。0 = 全闭，0.085452 = 全开 |
| `max_effort` | 最大力矩，单位 N·m。只用于控制器的失速判定，真正的力矩预算由 SDK 执行 |

`--feedback` 会打印 `reached_goal` 和 `stalled`。夹到物体停住时 `stalled=true` 是正常成功。

也可以在 RViz 里控制：MotionPlanning 面板的 **Joints** 标签有 `gripper_opening_joint` 滑条，**Goal State** 列表有 `open` 和 `closed`。

## 6. 接进已有的机械臂

这个插件是 `ros2_control` 硬件组件，可以和机械臂共用一个 `controller_manager`。

1. 在机器人 URDF 里包含硬件块：

   ```xml
   <xacro:include filename="$(find litegrip_ros2_control)/urdf/litegrip.ros2_control.xacro"/>
   ```

2. 把 `config/litegrip_controllers.yaml` 作为额外的 params 文件，和机械臂的控制器配置一起加载。

3. spawn `gripper_controller`。spawn 时硬件组件必须已经加载，否则 spawner 找不到 `gripper_opening_joint` 的接口。

随包提供的控制器是单关节上的 `position_controllers/GripperActionController`，暴露 `control_msgs/action/GripperCommand`，action 名是 `/gripper_controller/gripper_cmd`，不是 `FollowJointTrajectory`。

声明多个关节的 `<ros2_control>` 块会在 `on_init` 被拒绝，这是刻意设计：真实的 LiteGrip 只有一个传动，两个手指是 `<mimic>`。

## 7. 接真机

真机模式下夹爪会实际动作。第一次尝试时清空手指行程，手离开夹爪。

### 7.1 配置 CAN

```bash
sudo apt install -y can-utils
sudo modprobe gs_usb

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

确认输出里是 `state UP` 和 `bitrate 1000000`。运行 `controller_manager` 的进程需要 raw socket 权限（`CAP_NET_RAW`、root，或 udev/capability 规则）。

### 7.2 用两个开关启动

```bash
ros2 launch litegrip_moveit_config demo.launch.py \
  dry_run:=false hardware_enable:=true \
  max_feedback_velocity_rad_s:=0.9
```

`dry_run` 和 `hardware_enable` 必须同时给出，少一个都不会动。

`0.9` 只是示例。`max_feedback_velocity_rad_s` 必须按你这台机器标定：让它跑真实工况，取峰值反馈速度，再加余量。默认值 `-1.0` 表示**未给出**，控制环会拒绝下发任何运动帧。这是刻意的 fail-closed 行为。

### 7.3 安全基线与标定

安全基线里的红线是参考机的实测值，不是你这台的。如果本机标定结果落在红线之外，夹爪会被拒绝一切运动，直到红线重新推导。xacro 里的 `closed_rad` 和 `rad_to_mm` 默认值来自本机的 `factory_calibration.json`；重新标定后要同步更新 xacro。

## 8. 排错

| 现象 | 原因与处理 |
| --- | --- |
| `colcon build` 报 `ModuleNotFoundError: No module named 'catkin_pkg'` | Conda Python 顶掉了系统 Python，执行 `conda deactivate` 后重来 |
| `controller_manager` 加载不到 `litegrip_ros2_control/LitegripSystem` | 当前 shell 没有 source 这个包，或工作区没有编译。source `install/setup.bash` |
| 硬件组件 `on_init` 报 "supports 1 joint only" | `<ros2_control>` 块声明了多个关节。手指必须是 `<mimic>`，不能作为独立命令关节 |
| 硬件组件因安全基线报错而拒绝启动 | 找不到基线文件。设置 `LITEGRIP_DATA_DIR`，或在 `safety_baseline` 里给绝对路径 |
| 真机模式下夹爪不动 | `max_feedback_velocity_rad_s` 没给或给错，或两个开关少了一个。看 launch 警告 |
| `ros2 action list` 中没有 `gripper_cmd` | 控制器没起来。执行 `ros2 control list_controllers` 和 `ros2 control list_hardware_interfaces` |

## 9. 包文档

包 README 是依赖清单、全部参数、单位换算、钳位规则和测试套件的参考：

- [litegrip_ros2_control/README.md](litegrip_ros2_control/README.md)
- [litegrip_ros2_control/README.zh-CN.md](litegrip_ros2_control/README.zh-CN.md)

## 相关仓库

| 仓库 | 作用 |
| --- | --- |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-description](https://github.com/nexform-tech/litegrip-description) | 机器人描述 |
| [litegrip-moveit2](https://github.com/nexform-tech/litegrip-moveit2) | MoveIt 配置与演示 |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | 产品文档 |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 驱动 |

## 许可证

版权所有 © 2026 NEXFORM ROBOTICS。以 [Apache License 2.0](LICENSE) 发布。
