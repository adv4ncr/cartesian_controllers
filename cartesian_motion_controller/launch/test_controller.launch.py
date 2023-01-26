from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
distro = os.environ['ROS_DISTRO']
if distro == 'humble' or distro == 'galactic':
    spawner = "spawner"
else:  # foxy
    spawner = "spawner.py"


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Build the URDF with command line xacro.
    # We also pass parameters for the system_interface here.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("cartesian_controller_simulation"), "urdf", "robot.urdf.xacro"]
            ),
            " ",
            "mujoco_model:=",
            PathJoinSubstitution(
                [FindPackageShare("cartesian_controller_simulation"), "etc", "robot_mujoco.xml"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cartesian_motion_controller"), "config", "test_config.yaml",
        ]
    )

    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        #prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        output="both",
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ]
    )

    # Joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["cartesian_motion_controller", "--inactive", "-c", "/controller_manager"],
    )
    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["motion_control_handle", "--inactive", "-c", "/controller_manager"],
    )
    # joint_trajectory_controller_spawner = Node(
    #     package="controller_manager",
    #     executable=spawner,
    #     arguments=["joint_trajectory_controller", "--inactive", "-c", "/controller_manager"],
    # )

    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Visualization
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("cartesian_controller_simulation"), "etc", "robot.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config]
    )

    # Nodes to start
    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        cartesian_motion_controller_spawner,
        motion_control_handle_spawner,
        # joint_trajectory_controller_spawner,
        robot_state_publisher,
        rviz
    ]

    return LaunchDescription(declared_arguments + nodes)