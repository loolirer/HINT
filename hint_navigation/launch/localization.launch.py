import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAPS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "maps")
)


def localization_setup(context, *args, **kwargs):
    # Compose map_server + amcl directly rather than including nav2_bringup's localization_launch.py:
    # in this nav2 build that launch's `{yaml_filename: <map>}` override doesn't take effect, so
    # map_server comes up with an empty yaml_filename and publishes no map. Setting yaml_filename as
    # a plain Node parameter (a concrete path resolved here) loads the map reliably.
    region = LaunchConfiguration("region").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    if not map_yaml:
        map_yaml = os.path.join(MAPS_DIR, region, "map.yaml")

    params_file = os.path.join(
        get_package_share_directory("hint_navigation"), "config", "nav2_localization.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    return [map_server, amcl, lifecycle_manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "region", default_value="default",
            description="Environment name under hint_navigation/maps/<region>/",
        ),
        DeclareLaunchArgument(
            "map", default_value="",
            description="Saved occupancy map to localize against; empty → maps/<region>/map.yaml",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        OpaqueFunction(function=localization_setup),
    ])
