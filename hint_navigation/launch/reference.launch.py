import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    maps_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "maps")
    )
    params_file = os.path.join(
        get_package_share_directory("hint_navigation"), "config", "nav2_reference.yaml"
    )
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    bringup_launch = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
    rviz_config = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    region = LaunchConfiguration("region")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_region = DeclareLaunchArgument(
        "region", default_value="default",
        description="Environment name under hint_navigation/maps/<region>/",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([maps_dir, region, "map.yaml"]),
        description="Saved occupancy map to navigate on for the reference GoToGoal",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "slam": "False",
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_region,
        declare_map,
        declare_use_sim_time,
        nav2,
        rviz2,
    ])
