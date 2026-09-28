# litegrip-ros2

ROS 2 driver for the **LiteGrip lightweight robotic gripper series**.

## Packages

| Package | Type | Role |
| --- | --- | --- |
| `litegrip_ros2_control` | ament_cmake | `ros2_control` hardware interface: a C++ `SystemInterface` plugin backed directly by the [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) SDK. A thin shell — the SDK's `ControlLoop` owns the CAN device, the trajectory rate ceiling, the torque budget, the red-line safety gate and the DM MIT frame stream, on its own thread |

## Scope

| | |
| --- | --- |
| Product | LiteGrip lightweight robotic gripper series |
| Repository role | ROS 2 driver |
| Status | Active — ros2_control hardware interface backed by the C++ SDK (no Python daemon, no shared memory) |

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 driver |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
