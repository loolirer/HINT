import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    teleop_config = os.path.join(
        get_package_share_directory("hint"), "config", "teleop.yaml"
    )
    cartographer_config_dir = os.path.join(
        get_package_share_directory("turtlebot3_cartographer"), "config"
    )

    teleop = IncludeLaunchDescription(
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
    )

    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory",
            cartographer_config_dir,
            "-configuration_basename",
            "turtlebot3_lds_2d.lua",
        ],
    )

    occupancy_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
    )

    # --- Nav2 reactive navigation (replaces odom_waypoint_tracker + pursuit_servo) ---
    # Shared camera-rig geometry for the ground projection (match the real rig).
    camera_rig = {
        "camera_height": 0.14,
        "camera_forward_offset": 0.0,
        "camera_tilt": 0.0,
        "camera_hfov_deg": 62.2,
    }

    ground_segmenter = Node(
        package="hint_nav2",
        executable="ground_segmenter",
        output="screen",
        parameters=[camera_rig, {"device": "GPU"}],
    )

    trajectory_navigator = Node(
        package="hint_nav2",
        executable="trajectory_navigator",
        output="screen",
        parameters=[camera_rig],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hint_nav2"),
                "launch",
                "nav2.launch.py",
            )
        )
    )

    reasoner = Node(
        package="gemini_robotics_er",
        executable="reasoner",
        output="screen",
        parameters=[
            {
                "api_key_path": "/root/secrets/gemini_api_key.txt",
                "model_id": "gemini-3.1-flash-lite",
                "thinking_budget": -1,
                "history_frames": 1,
                "structured_output": "off",
            }
        ],
    )

    trajectory_planner = Node(
        package="gemini_robotics_er",
        executable="trajectory_planner",
        output="screen",
        parameters=[
            {
                "api_key_path": "/root/secrets/gemini_api_key.txt",
                "model_id": "gemini-robotics-er-1.6-preview",
                "thinking_budget": 0,
                "temperature": 1.0,
                "n_candidates": 5,
                "history_frames": 0,
                "structured_output": "off",
            }
        ],
    )

    mission_planner = Node(
        package="mission_planner",
        executable="mission_planner_node",
        output="screen",
    )

    bt_executor = Node(
        package="hint_bt",
        executable="bt_executor_node",
        output="screen",
        parameters=[
            {
                "action_name": "/bt_executor_node/execute_behavior_tree",
                "behavior_trees": ["hint_bt/behaviors"],
            }
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("hint"),
                "viz",
                "hint.rviz",
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            teleop,
            # cartographer,
            # occupancy_grid,
            ground_segmenter,
            trajectory_navigator,
            nav2,
            trajectory_planner,
            reasoner,
            mission_planner,
            bt_executor,
            rviz2,
        ]
    )
