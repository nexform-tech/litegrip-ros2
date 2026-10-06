"""Start the gripper's controllers on a controller_manager that is already running.

This package ships no URDF geometry, so it cannot start `ros2_control_node` itself: the
stack that owns the hardware component brings the manager up (the description package's
`ros2_control.launch.py`, or the arm's control launch). This file adds the other half —
spawning the controllers that make the gripper commandable, with the configuration that
lives in this package.

Run it once that stack is up:

    ros2 launch litegrip_ros2_control litegrip_controllers.launch.py
    ros2 launch litegrip_ros2_control litegrip_controllers.launch.py inactive:=true

Then command the gripper over its action (`GripperActionController` has no command topic):

    ros2 action send_goal /gripper_controller/gripper_cmd \
        control_msgs/action/GripperCommand "{command: {position: 0.043, max_effort: 3.5}}"

Why the spawner gets both `--param-file` and `--controller-type`: the parameter file
supplies the controller's own parameters (`joint`, tolerances, stall handling), but its
`type` lives under the file's `controller_manager` section, which the spawner never reads.
Passing the type explicitly is what lets these controllers be spawned onto a manager that
was started without this package's configuration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    return text.strip().lower() in ("1", "true", "yes", "on")


def _spawner(name, controller_type, manager, param_file, timeout, inactive):
    arguments = [
        name,
        "--controller-manager", manager,
        "--controller-type", controller_type,
        "--controller-manager-timeout", timeout,
    ]
    if param_file:
        arguments += ["--param-file", param_file]
    if inactive:
        arguments.append("--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
        output="screen",
        emulate_tty=True,
    )


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    manager = resolve("controller_manager")
    controllers_file = resolve("controllers_file")
    joint_state_broadcaster = resolve("joint_state_broadcaster")
    joint_state_broadcaster_type = resolve("joint_state_broadcaster_type")
    gripper_controller = resolve("gripper_controller")
    gripper_controller_type = resolve("gripper_controller_type")
    controller_manager_timeout = resolve("controller_manager_timeout")
    start_joint_state_broadcaster = _as_bool(resolve("start_joint_state_broadcaster"))
    start_gripper_controller = _as_bool(resolve("start_gripper_controller"))
    inactive = _as_bool(resolve("inactive"))

    actions = []
    broadcaster_spawner = None
    if start_joint_state_broadcaster:
        broadcaster_spawner = _spawner(
            joint_state_broadcaster, joint_state_broadcaster_type, manager, "",
            controller_manager_timeout, inactive)
        actions.append(broadcaster_spawner)

    if start_gripper_controller:
        gripper_spawner = _spawner(
            gripper_controller, gripper_controller_type, manager, controllers_file,
            controller_manager_timeout, inactive)
        if broadcaster_spawner is not None:
            # The gripper controller configures against joint states, so it starts only
            # after the broadcaster has been activated and has exited.
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=broadcaster_spawner, on_exit=[gripper_spawner])))
        else:
            actions.append(gripper_spawner)

    namespace = manager.rsplit("/", 1)[0] if "/" in manager else ""
    action_name = "{}/{}/gripper_cmd".format(namespace, gripper_controller)
    actions.append(LogInfo(msg=(
        "\nControllers requested on {}.\n"
        "  state:   ros2 control list_controllers\n"
        "  command: ros2 action send_goal {} control_msgs/action/GripperCommand "
        "\"{{command: {{position: 0.043, max_effort: 3.5}}}}\"\n"
        "  position is the opening in metres: 0.0 closed, 0.087 fully open.\n"
        "  The hardware must be enabled for the fingers to move: dry_run, "
        "hardware_enable and max_feedback_velocity_rad_s are URDF parameters.".format(
            manager, action_name))))
    return actions


def generate_launch_description():
    controllers_file = os.path.join(
        get_package_share_directory("litegrip_ros2_control"),
        "config", "litegrip_controllers.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller_manager", default_value="/controller_manager",
            description="Name of the controller_manager to spawn onto."),
        DeclareLaunchArgument(
            "controllers_file", default_value=controllers_file,
            description="Controller parameter file; its controller section is passed to "
                        "the spawner, its controller_manager section is not."),
        DeclareLaunchArgument(
            "joint_state_broadcaster", default_value="joint_state_broadcaster",
            description="Name of the joint state broadcaster to spawn."),
        DeclareLaunchArgument(
            "joint_state_broadcaster_type",
            default_value="joint_state_broadcaster/JointStateBroadcaster",
            description="Type passed to the spawner for the broadcaster."),
        DeclareLaunchArgument(
            "gripper_controller", default_value="gripper_controller",
            description="Name of the gripper controller to spawn."),
        DeclareLaunchArgument(
            "gripper_controller_type",
            default_value="position_controllers/GripperActionController",
            description="Type passed to the spawner for the gripper controller; the "
                        "default is what config/litegrip_controllers.yaml declares."),
        DeclareLaunchArgument(
            "start_joint_state_broadcaster", default_value="true",
            description="Spawn and activate the joint state broadcaster. Set false when "
                        "another stack already publishes /joint_states."),
        DeclareLaunchArgument(
            "start_gripper_controller", default_value="true",
            description="Spawn and activate the gripper controller."),
        DeclareLaunchArgument(
            "inactive", default_value="false",
            description="Load and configure the controllers without activating them."),
        DeclareLaunchArgument(
            "controller_manager_timeout", default_value="30.0",
            description="Seconds each spawner waits for the controller_manager service."),
        OpaqueFunction(function=_launch_setup),
    ])
