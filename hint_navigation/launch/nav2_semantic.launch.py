import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

MAPS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "maps")
)


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("hint_navigation"), "config"
    )
    common_params = os.path.join(config_dir, "nav2_common.yaml")
    semantic_params = os.path.join(config_dir, "nav2_semantic.yaml")

    region = LaunchConfiguration("region")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_region = DeclareLaunchArgument(
        "region", default_value="default",
        description="Environment name under hint_navigation/maps/<region>/",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([MAPS_DIR, region, "map.yaml"]),
        description="Saved occupancy map to localize the HINT run against",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    driving_params = [common_params, semantic_params, {"use_sim_time": use_sim_time}]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hint_navigation"),
                "launch",
                "localization.launch.py",
            )
        ),
        launch_arguments={
            "region": region,
            "map": map_yaml,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=driving_params,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=driving_params,
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
        declare_region,
        declare_map,
        declare_use_sim_time,
        localization,
        controller_server,
        behavior_server,
        lifecycle_manager,
    ])
