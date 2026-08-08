import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    maps_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "maps")
    )
    params_file = os.path.join(
        get_package_share_directory("hint_navigation"), "config", "localization.yaml"
    )
    localization_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "localization_launch.py"
    )

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
        description="Saved occupancy map to localize against",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        declare_region,
        declare_map,
        declare_use_sim_time,
        localization,
    ])
