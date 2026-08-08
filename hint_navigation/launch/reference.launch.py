import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    maps_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "maps")
    )
    config_dir = os.path.join(
        get_package_share_directory("hint_navigation"), "config"
    )
    common_params = os.path.join(config_dir, "nav2_common.yaml")
    geometric_params = os.path.join(config_dir, "nav2_geometric.yaml")
    teleop_config = os.path.join(
        get_package_share_directory("hint_navigation"), "config", "teleop.yaml"
    )
    rviz_config = os.path.join(
        get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz"
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
        description="Saved occupancy map to navigate on for the reference GoToGoal",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
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

    geometric_only = [geometric_params, {"use_sim_time": use_sim_time}]
    driving_params = [common_params, geometric_params, {"use_sim_time": use_sim_time}]
    cmd_vel_to_mux = ("cmd_vel", "cmd_vel_nav2")

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
        remappings=[cmd_vel_to_mux],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=geometric_only,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=driving_params,
        remappings=[cmd_vel_to_mux],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=geometric_only,
    )

    lifecycle_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
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
        teleop,
        localization,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_navigation,
        rviz2,
    ])
