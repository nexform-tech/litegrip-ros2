"""litegrip_hw.launch.py — start only the LiteGrip gripper's hardware daemon.

    ros2 launch litegrip_ros2_control litegrip_hw.launch.py                 # dry-run
    ros2 launch litegrip_ros2_control litegrip_hw.launch.py dry_run:=false hardware_enable:=true

★ Why this launch does **not** start a controller_manager
---------------------------------------------------------
The gripper's hardware interface is a ros2_control **hardware component**, and
what loads it is controller_manager; this project has exactly **one** CM (in
litearm_ros2_control's launch, shared with the arm). Starting a second CM would
fight over the same hardware resource and the same controller namespace, for no
benefit at all.

Division of labour:

* This launch only takes care of "making the daemon bring the shared memory
  segment up" — the daemon is the segment's **owner** and has to be in place
  before the CM (the plugin's on_configure waits for it, and on timeout gives
  actionable troubleshooting hints).
* Loading the hardware component (the ``<ros2_control>`` + ``<plugin>`` in the
  URDF) is decided by whichever description includes litegrip.ros2_control.xacro
  — that is **next round**'s wiring (this round only delivers the hardware
  interface itself).

⚠ dry_run defaults to true. Real hardware needs **both** switches at once
  (``dry_run:=false`` **and** ``hardware_enable:=true``); giving only the former
  will not connect to the hardware — the daemon rejects every command and logs
  an ERROR — there is no "give one switch fewer and it silently connects" path.

⚠ The daemon exiting **disables** the gripper (both fingers go limp and can be
  pushed). Do not stop it while it is holding something; if you need it to hold
  a little longer before exiting, raise ``exit_hold_s``.
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration


def _declare_arguments():
    return [
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true = the mock adapter layer "
                                          "(no hardware touched)"),
        DeclareLaunchArgument("hardware_enable", default_value="false",
                              description="master switch for real hardware; must "
                                          "be given together with dry_run:=false"),
        DeclareLaunchArgument("shm_name", default_value="/litegrip_hw",
                              description="shared memory segment name; must match "
                                          "shm_name in the URDF plugin parameters"),
        DeclareLaunchArgument("hw_config", default_value="",
                              description="optional litegrip_hw.yaml path "
                                          "(PC-side parameters only)"),
        DeclareLaunchArgument("channel", default_value="can0",
                              description="gripper CAN interface (the STM32's "
                                          "gs_usb bridge; needs 1 Mbit/s)"),
        DeclareLaunchArgument("rate_hz", default_value="50.0",
                              description="control cycle rate of the daemon"),
        DeclareLaunchArgument("sdk_path", default_value="",
                              description="path to the LiteGrip SDK copy (the red "
                                          "line comes from it); empty = use the "
                                          "copy carried in the workspace"),
        DeclareLaunchArgument("feedback_velocity_rad_s", default_value="-1.0",
                              description="worst-case feedback velocity bound "
                                          "(rad/s); -1 = not given ⇒ the "
                                          "hardware path refuses to send any "
                                          "motion frame"),
        DeclareLaunchArgument("exit_hold_s", default_value="2.0",
                              description="seconds to keep holding before exit "
                                          "(0 = shut down right away)"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    def is_true(text: str) -> bool:
        return text.strip().lower() in ("1", "true", "yes", "on")

    daemon = os.path.join(get_package_prefix("litegrip_ros2_control"), "lib",
                          "litegrip_ros2_control", "litegrip_hw_daemon")

    # The two switches are passed explicitly in BooleanOptionalAction's paired
    # form: "not given" and "explicitly off" mean different things to the daemon
    # (the former falls back to the config file / defaults).
    cmd = [
        daemon,
        "--shm-name", resolve("shm_name"),
        "--channel", resolve("channel"),
        "--rate-hz", resolve("rate_hz").strip() or "50.0",
        "--max-feedback-velocity-rad-s", resolve("feedback_velocity_rad_s"),
        "--exit-hold-s", resolve("exit_hold_s").strip() or "2.0",
        "--dry-run" if is_true(resolve("dry_run")) else "--no-dry-run",
        ("--hardware-enable" if is_true(resolve("hardware_enable"))
         else "--no-hardware-enable"),
    ]

    sdk_path = resolve("sdk_path").strip()
    if not sdk_path:
        # The workspace's SDK copy sits at <ws>/src/litegrip/sdk (a sibling of
        # the two packages). <ws> = prefix/../.. (prefix is <ws>/install/<pkg>)
        # — the convention this project already follows.
        # ⚠ Do not walk up from the share/ directory: its depth differs from
        #   prefix's.
        sdk_path = os.path.normpath(os.path.join(
            get_package_prefix("litegrip_ros2_control"),
            os.pardir, os.pardir, "src", "litegrip", "sdk"))
    cmd += ["--sdk-path", sdk_path]

    hw_config = resolve("hw_config").strip()
    if hw_config:
        cmd += ["--hw-config", hw_config]

    target = "dry-run (mock data, no hardware)" if is_true(resolve("dry_run")) else (
        "real hardware (can0 taken exclusively)" if is_true(resolve("hardware_enable"))
        else "**driving refused**: dry_run=false but hardware_enable=false")

    return [
        LogInfo(msg=(
            "───── LiteGrip gripper hardware daemon ─────\n"
            f"  mode  : {target}\n"
            f"  shm   : {resolve('shm_name')}\n"
            f"  can   : {resolve('channel')}\n"
            f"  SDK   : {sdk_path}\n"
            "  ⚠ this launch does not start a controller_manager: the\n"
            "     gripper hardware component is loaded by the CM shared with\n"
            "     the arm (next round's wiring).\n"
            "────────────────────────────────────────────")),
        ExecuteProcess(cmd=cmd, output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                            [OpaqueFunction(function=_launch_setup)])
