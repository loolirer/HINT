import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

MAPS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "maps")
)


def map_autosaver(context, *args, **kwargs):
    region = LaunchConfiguration("region").perform(context)
    interval = LaunchConfiguration("save_interval").perform(context)
    map_stem = os.path.join(MAPS_DIR, region, "map")
    os.makedirs(os.path.dirname(map_stem), exist_ok=True)

    save_loop = (
        f'while true; do sleep {interval}; '
        f'ros2 run nav2_map_server map_saver_cli -f "{map_stem}" '
        f'--ros-args -p save_map_timeout:=5.0; done'
    )
    return [
        ExecuteProcess(
            cmd=["bash", "-c", save_loop],
            name="map_autosaver",
            output="screen",
        )
    ]


def generate_launch_description():
    teleop_config = os.path.join(
        get_package_share_directory("hint_navigation"), "config", "teleop.yaml"
    )

    teleop = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/cmd_vel_teleop"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("teleop_twist_joy"),
                        "launch",
                        "teleop-launch.py",
                    )
                ),
                launch_arguments={
                    "config_filepath": teleop_config,
                    "publish_stamped_twist": "true",
                }.items(),
            ),
        ]
    )

    cartographer_config_dir = os.path.join(
        get_package_share_directory("turtlebot3_cartographer"), "config"
    )
    try:
        rviz_config = os.path.join(
            get_package_share_directory("turtlebot3_cartographer"),
            "rviz",
            "tb3_cartographer.rviz",
        )
        rviz_args = ["-d", rviz_config]
    except Exception:
        rviz_args = []

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_region = DeclareLaunchArgument(
        "region", default_value="default",
        description="Environment name — the map is autosaved to hint_navigation/maps/<region>/map",
    )
    declare_save_interval = DeclareLaunchArgument(
        "save_interval", default_value="5.0",
        description="Seconds between map autosaves; the last save before Ctrl+C is your map",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", cartographer_config_dir,
            "-configuration_basename", "turtlebot3_lds_2d.lua",
        ],
    )

    occupancy_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=rviz_args,
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_region,
        declare_save_interval,
        declare_use_sim_time,
        teleop,
        cartographer,
        occupancy_grid,
        rviz2,
        OpaqueFunction(function=map_autosaver),
    ])
