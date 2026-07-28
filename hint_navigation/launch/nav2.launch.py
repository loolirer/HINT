"""Mapless reactive Nav2 bringup for HINT.

Starts the pieces needed to follow a VLM path safely off a local costmap and re-orient
at the end: ``controller_server`` (FollowPath + MPPI, with its rolling local_costmap),
``behavior_server`` (the Spin behavior, for the BT's end-of-path / scan turn), and a
``nav2_lifecycle_manager`` that autostarts both. No map_server / amcl / planner_server /
bt_navigator — there is no global map. The obstacle layer is fed by ``obstacle_projector``
(`/obstacles`); the path is delivered by ``path_projector`` via the
controller's ``follow_path`` action; the turn by ``hint_behavior``'s SpinAction via ``/spin``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("hint_navigation"), "config", "nav2_local.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            # controller first so its local costmap is up before behavior_server (Spin)
            # subscribes to it for collision checking.
            "node_names": ["controller_server", "behavior_server"],
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        controller_server,
        behavior_server,
        lifecycle_manager,
    ])
