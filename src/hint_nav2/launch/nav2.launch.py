"""Mapless reactive Nav2 bringup for HINT.

Starts only the pieces needed to follow a VLM path safely off a local costmap:
``controller_server`` (FollowPath + MPPI, with its rolling local_costmap) and a
``nav2_lifecycle_manager`` that autostarts it. No map_server / amcl / planner_server /
bt_navigator — there is no global map. The obstacle layer is fed by ``ground_segmenter``
(`/ground/obstacles`), and the path is delivered by ``trajectory_navigator`` via the
controller's ``follow_path`` action.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("hint_nav2"), "config", "nav2_local.yaml"
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

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["controller_server"],
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        controller_server,
        lifecycle_manager,
    ])
