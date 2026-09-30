LiteGrip 夹爪的 `ros2_control` 硬件接口，以及它在动一根手指之前必须具备的全部条件
—— 在构建或部署之前请先读这一篇。

# litegrip_ros2_control

[English](README.md) · **简体中文**

LiteGrip 两指夹爪的 `ros2_control` **SystemInterface** 插件，直接由
[litegrip_cpp](https://github.com/nexform-tech/litegrip-cpp) C++ SDK 支撑。
它是 litegrip 栈的 L2 硬件组件。

## 在栈中的位置

```text
litegrip_cpp (C++ SDK)  --链接-->  litegrip_ros2_control  --加载-->  litegrip_moveit_config
```

| 层 | 包 | 负责 | 不负责 |
| --- | --- | --- | --- |
| L3 | `litegrip_moveit_config` | 语义描述（SRDF）、规划组、关节限位、控制器映射、demo 启动 | 任何硬件交互 |
| L2 | **`litegrip_ros2_control`**（本包） | 解析 URDF 参数、驱动 `ControlLoop` 生命周期、`m` ⇄ `rad` 换算、把状态搬进接口 | 限速、安全判断、实时控制 |
| L1 | `litegrip_cpp` | SocketCAN、达妙 MIT 协议、安全闸门、轨迹限速、力矩预算、看门狗、帧流 | 不知道 ROS 的存在 |

以前那个 Python 守护进程做的事，现在全部在 `litegrip_cpp::ControlLoop` 里，跑在
SDK 自己的线程上：单进程、无共享内存、无 Python、也不再有需要手工同步的 ctypes
布局。`read()` 与 `write()` 既不碰 CAN 也不分配内存 —— 它们只复制一份缓存快照、
投递一个目标，所以总线状况再差，`controller_manager` 的控制周期依然很便宜。

**本包自己动不了夹爪。** 它不含启动文件、不含 URDF 几何、也不含控制器实现。
下面逐条说明它需要什么。

## 依赖关系

### 构建与链接依赖

| 依赖 | 类别 | 为什么需要 | 缺失后果 |
| --- | --- | --- | --- |
| `litegrip_cpp` | 构建 / 链接 / 运行 | 整条控制路径：SocketCAN、达妙 MIT 组帧、安全闸门、限速、力矩预算、看门狗 | `find_package(litegrip_cpp REQUIRED)` 失败，包无法配置 |
| `hardware_interface` | 构建 | `hardware_interface::SystemInterface` 基类与 `HardwareInfo` | 没有可实现的基类 |
| `pluginlib` | 构建 | 导出 `litegrip_ros2_control/LitegripSystem`（见 `litegrip_ros2_control.xml`） | `controller_manager` 加载不到该组件 |
| `rclcpp`、`rclcpp_lifecycle` | 构建 | 日志器，以及每个回调收到的生命周期状态 | 同上 |
| `ament_cmake` | 构建工具 | `package.xml` 声明的构建类型 | colcon 无法构建本包 |
| `ament_cmake_gtest` | 测试 | 19 个用例的测试套件 | `colcon test` 无法构建测试 |

### 运行时 ROS 依赖

| 依赖 | 为什么需要 | 缺失后果 |
| --- | --- | --- |
| `controller_manager` | 加载、配置、激活、去激活硬件组件 | 没人加载插件，夹爪根本不会出现 |
| `joint_state_broadcaster` | 从状态接口发布 `/joint_states`，夹爪才进入 TF 与 MoveIt | 夹爪关节在 ros2_control 之外不可见 |
| `gripper_controllers` | 提供 `position_controllers/GripperActionController`，即 `config/litegrip_controllers.yaml` 配置的类型 | 随包控制器配置找不到实现 |
| `robot_state_publisher`、`xacro` | 展开包含 `urdf/litegrip.ros2_control.xacro` 的 URDF | `<ros2_control>` 段永远到不了 `controller_manager` |
| `launch`、`launch_ros` | 用启动文件把 `config/litegrip_controllers.yaml` 作为 params 文件传给 `controller_manager` | 控制器配置只能手工传 |

### `litegrip_cpp` 是源码依赖，不是包依赖

SDK 是纯 CMake（非 ament），也没有发布到任何包索引。它必须以源码形式存在，两种方式：

- **同一个 colcon 工作区** —— 与本包并列 checkout，colcon 会依据 `package.xml`
  排好构建顺序。CI 就是这么做的：[.github/workflows/ci.yml](.github/workflows/ci.yml)
  的 `test` 任务把 `nexform-tech/litegrip-cpp` checkout 进工作区，再执行
  `colcon build --packages-select litegrip_cpp litegrip_ros2_control`。
- **已安装的 prefix** —— 用 SDK 自己的 CMake 构建并安装，`find_package(litegrip_cpp REQUIRED)`
  会通过正常 CMake 包机制找到 `litegrip_cpp-config.cmake`。

**不要指望 `rosdep` 把它拉下来。** `rosdep install --from-paths src --ignore-src`
只会按 manifest 解决上面那些 ROS 依赖；SDK 没有自己的索引条目。

### 运行时数据：安全基线，以及它如何被找到

`litegrip_cpp` 把数据装在 `<prefix>/share/litegrip_cpp/calibration`
（`safety_limits_350.json`、`safety_limits_025.json`、`factory_calibration.json`），
并把该目录导出为 `litegrip_cpp_DATA_DIR`。本包的 CMake 把它编成编译期宏
`LITEGRIP_DEFAULT_DATA_DIR`，`on_init` 再把它发布成 `LITEGRIP_DATA_DIR` 环境变量。
解析优先级（从强到弱）：

| # | 手段 | 由谁设置 |
| --- | --- | --- |
| 1 | `safety_baseline` 参数**直接给路径**（含 `/`） | 部署该夹爪的 URDF 或启动文件 |
| 2 | `LITEGRIP_DATA_DIR` 环境变量 | 部署方（`setenv` 用 `overwrite=0`，不会被覆盖） |
| 3 | 构建期的 `LITEGRIP_DEFAULT_DATA_DIR` | 本组件自动 |

**不要依赖工作目录。** 版本名 `3.5` 之类只在相对于进程工作目录时才解析得到，
而 `controller_manager` 是从用户当时所在的目录启动的。基线缺失或版本未知是
fail-closed 的 —— 组件宁可拒绝启动，也不会退回没人确认过的限值。

换算参数 `closed_rad` / `rad_to_mm` 是本组件的参数，xacro 里的默认值
（`0.04139` / `65.0231`）是该台 `factory_calibration.json` 的拷贝。重新标定后，
请从该文件更新 xacro 里的值，两者不能漂移。

### 硬件

只在 `dry_run` 为 false 时才需要；默认配置下完全不碰。

| 项目 | 值 |
| --- | --- |
| 总线 | SocketCAN `can0`，Classic CAN 1 Mbit/s（CAN-FD 关闭） |
| 权限 | 运行 `controller_manager` 的用户需有 raw socket 权限（`CAP_NET_RAW`、root，或 udev/capability 规则） |
| 寻址 | 命令 id `8`，状态（master）id `24` |
| 电机 | 达妙 DM-J4310-2EC，MIT 模式 |

有三个参数决定真机到底会不会动，而且各自拦的是不同的东西：

- `dry_run`（默认 `true`）—— 「调试时不要碰硬件」。
- `hardware_enable`（默认 `false`）—— 「这台机器上真的挂了夹爪吗」。
  走真机路径时两个开关都要翻。
- `max_feedback_velocity_rad_s`（默认 `-1.0`）—— `-1` 表示**未给出**，控制环会
  **拒绝发送任何运动帧**。即使两个开关都翻对了，这个值也必须先在真机上标定出来。

### 本包刻意不含的东西

- **没有几何。** 关节名（`gripper_opening_joint`）、两根手指及其 `<mimic>` 都来自
  描述包 `litegrip_urdf`（注意：包名与目录名 `litegrip_description` 不一致）。
- **没有启动文件。** 要通过一个能拉起 `controller_manager` 的栈来部署，本组织的
  那一个是 `litegrip_moveit_config`。
- **多关节会被拒绝。** 真爪只有一套传动、两指是 `<mimic>`，所以声明两个关节的
  `<ros2_control>` 段会在 `on_init` 按设计失败。

## 构建

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/nexform-tech/litegrip-ros2
git clone https://github.com/nexform-tech/litegrip-cpp litegrip_cpp

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y     # 只装 ROS 依赖，不含 litegrip_cpp
colcon build --packages-select litegrip_cpp litegrip_ros2_control
source install/setup.bash
```

目标平台：ROS 2 Humble Hawksbill / Ubuntu 22.04 (x86_64)，C++17。

## 接口

| 关节 | 命令接口 | 状态接口 |
| --- | --- | --- |
| `gripper_opening_joint` | `position` | `position`、`velocity`、`effort`、`temperature_mos`、`temperature_coil`、`error_code`、`fault_code`、`feedback_age` |

- **命令只有 `position`。** 轨迹限速与力矩预算是本组件的**部署参数**，不是逐条命令
  的字段：它们描述「这台夹爪允许怎么动」，不能让每个命令发布者各自决定。
- **状态是标准三件套** —— 与机械臂关节完全相同的三个接口，`joint_state_broadcaster`
  对两者一视同仁 —— **外加诊断项**。`joint_state_broadcaster` 不认领诊断项，
  激活时 `controller_manager` 会告警「接口无人认领」，这是预期行为；把
  `export_diagnostics` 设为 `false` 可以去掉后五项（只剩 3 个状态接口）。
- 两指是 `<mimic>`，这里不出现：真爪只有一套传动。

## 参数

`urdf/litegrip.ros2_control.xacro` 里的每个 `<param>` 都由**同名的 `litegrip_` 前缀
xacro 参数**提供，因此启动文件可以覆盖任何一个，而不必改 URDF。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `channel` | `can0` | SocketCAN 接口 |
| `can_id` / `mst_id` | `8` / `24` | 命令 id / 状态帧 id |
| `canfd_mode` | `false` | Classic CAN |
| `dry_run` | `true` | 对着仿真对象跑整条控制路径，只跳过开 CAN 和发帧 |
| `hardware_enable` | `false` | 真机总开关 |
| `control_rate_hz` | `200.0` | SDK 线程的帧率 |
| `feedback_timeout_s` | `0.5` | 多久没有**新**状态帧算失联 |
| `temperature_limit_c` | `80` | 在驱动自己跳闸**之前**停 |
| `command_timeout_s` | `0.2` | 命令陈旧判据：这么久没有新命令就保持不动 |
| `max_velocity_rad_s` | `1.5` | 轨迹推进速率上限；只能调小，不能调大 |
| `torque_limit_nm` | `3.5` | 总力矩预算，每帧在位置增益之间分配；只能调小 |
| `safety_baseline` | `3.5` | 版本化基线名，**或**基线文件的路径 |
| `max_position_error_rad` | `-1.0` | `-1` = 由红线宽度推导 |
| `max_feedback_velocity_rad_s` | `-1.0` | `-1` = 未给出 ⇒ 拒绝发送任何运动帧 |
| `kp` / `kd` | `20.0` / `0.5` | 只是上限；每帧按预算重新分配 |
| `closed_rad` / `rad_to_mm` | `0.04139` / `65.0231` | 该台的标定值 |
| `min_width` / `max_width` | `0.0` / `0.087` | 模型层开口范围（m），与 URDF 关节限位一致 |
| `export_diagnostics` | `true` | 是否导出那五个诊断状态接口 |

`dry_run` **不是空转**：SDK 会对着一个仿真对象跑完整条控制路径 —— 限速、力矩预算
分配、安全闸门、看门狗 —— 只跳过开 CAN 和发帧。所以配置错误在没有夹爪的情况下
就能被 dry run 抓出来。

## 单位、符号与钳位

一个数一个单位：

| 侧 | 单位 |
| --- | --- |
| ROS 接口存储 | 开口 **m**（对齐 URDF 关节限位） |
| SDK | 电机角 **rad**（红线、力矩预算、MIT 量化都在这个单位下定义和验证） |

```text
开口 [mm] = (closed_rad - rad) x rad_to_mm
rad       = closed_rad - 开口 [mm] / rad_to_mm
```

**数值越负的 `rad`，开口越大**，因此速度换算必须翻号：

```text
开口速度 [m/s] = -v_rad x rad_to_mm x 1e-3
```

这是本组件唯一一处「不显眼但错了」的地方：漏掉负号，位置仍然正确，只有速度反了。
而速度反馈正是用来判断「停了没有」的量 —— 符号反了不会报错，只会让那个判断静默出错。

**不要删掉 `width_to_rad()` 里的钳位。** 模型层范围是 `0 ~ 87 mm`，而可命令范围
受红线约束更窄，且 SDK 对越界目标是**拒绝**（不是钳位）。不在这里钳位的话，一条
完全正常的「全开」命令会被整条丢掉、夹爪纹丝不动，只留下一个故障码。可命令范围
是从 **SDK 实际加载的红线**推导的（`loop_->safety_limits()`），不是 URDF 里的
第二份拷贝。

## 部署方式

在描述包的 URDF 里包含这个硬件段：

```xml
<xacro:include filename="$(find litegrip_ros2_control)/urdf/litegrip.ros2_control.xacro"/>
```

然后把 `config/litegrip_controllers.yaml` 作为**额外的 params 文件**，与机械臂的
控制器配置一起加载。生成控制器时硬件组件必须已经加载 —— 也就是说描述文件必须
已经包含上面的 xacro —— 否则 spawner 找不到 `gripper_opening_joint` 的接口。

随包控制器是单关节上的 `position_controllers/GripperActionController`，在
`/gripper_controller/gripper_cmd` 上提供 `control_msgs/action/GripperCommand`
（**不是** `FollowJointTrajectory`）。堵转被配置为成功，因为「夹住物体后停住」
正是抓取想要的结果。任何写死了轨迹 action 名的上层都必须改。

## 测试

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select litegrip_ros2_control
colcon test-result --verbose
```

在 ROS 2 Humble / Ubuntu 22.04 x86_64 上的实测结果（2026-09-30）：

```text
Summary: 39 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result` 统计的是**结果条目**，每个 gtest 二进制还会额外多计一条；
套件本身是 `test_litegrip_system` 里的 **19 个 gtest 用例**，测试自己的 XML 报的是
`tests="19" failures="0"`。所有用例都在 `dry_run=true` 下、用合成的 `HardwareInfo`
在进程内驱动组件生命周期，不开 CAN socket —— 这正是参数校验、`m` ⇄ `rad` 换算、
钳位与激活 latch 能在没有夹爪的情况下被测试的原因。

## 影响部署的已知缺口

| 缺口 | 影响 |
| --- | --- |
| 随包红线是参考机的手推实测值 | 在本台机器上命令真实运动之前必须重标，`max_feedback_velocity_rad_s` 同理 |
| 没有受控的软件手段回到红线内 | 停在红线外的夹爪（完全闭合就是其中一种）当前无法由本栈恢复 |
| 诊断接口已导出但无人认领 | 故障码只能从节点日志看（`grep "gripper fault code"`），不在话题上 |
| 传输层发送/接收路径无测试覆盖 | 需要 vcan 接口（root）或真机 |

## 相关仓库

| 仓库 | 作用 |
| --- | --- |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK —— 本包的硬依赖 |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 驱动 |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | 产品文档 |

## 许可证

仓库在 [LICENSE](../LICENSE) 中提供 Apache License 2.0；`package.xml` 里仍写着
`BSD-3-Clause`，那是包模板遗留，并非本仓库实际使用的许可证。
