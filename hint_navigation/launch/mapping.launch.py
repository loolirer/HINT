import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    maps_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "maps")
    )

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

    region = LaunchConfiguration("region")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_region = DeclareLaunchArgument(
        "region", default_value="default",
        description="Environment name — the map is saved to hint_navigation/maps/<region>/map",
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

    def save_map_on_shutdown(event, context):
        map_stem = os.path.join(maps_dir, region.perform(context), "map")
        os.makedirs(os.path.dirname(map_stem), exist_ok=True)
        return [
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", map_stem,
                    "--ros-args", "-p", "save_map_timeout:=5.0",
                ],
                output="screen",
            )
        ]

    save_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=save_map_on_shutdown)
    )

    return LaunchDescription([
        declare_region,
        declare_use_sim_time,
        teleop,
        cartographer,
        occupancy_grid,
        rviz2,
        save_on_shutdown,
    ])
