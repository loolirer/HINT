import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    teleop_config = os.path.join(
        get_package_share_directory("hint_bringup"), "config", "teleop.yaml"
    )
    
    cartographer_config_dir = os.path.join(
        get_package_share_directory("hint_bringup"), "config"
    )

    camera_rig = {
        "camera_height": 0.105,
        "camera_forward_offset": 0.073,
        "camera_tilt": -0.025,
        "camera_hfov_deg": 62.2,
    }

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

    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        arguments=[
            "-configuration_directory",
            cartographer_config_dir,
            "-configuration_basename",
            "turtlebot3_lds_2d.lua",
            "--ros-args",
            "--log-level",
            "error",
        ],
    )

    occupancy_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        arguments=[
            "-resolution",
            "0.05",
            "-publish_period_sec",
            "2.0",
            "--ros-args",
            "--log-level",
            "error",
        ],
    )

    ground_segmenter = Node(
        package="hint_perception",
        executable="ground_segmenter",
        output="screen",
        parameters=[camera_rig, {"device": "GPU"}],
    )

    visual_debug = Node(
        package="hint_perception",
        executable="visual_debug",
        output="screen",
        parameters=[camera_rig],
    )

    trajectory_navigator = Node(
        package="hint_navigation",
        executable="trajectory_navigator",
        output="screen",
        parameters=[camera_rig],
    )

    nav2 = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/cmd_vel_nav2"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("hint_navigation"),
                        "launch",
                        "nav2.launch.py",
                    )
                )
            ),
        ]
    )

    visual_reasoner = Node(
        package="gemini_robotics_er",
        executable="visual_reasoner",
        output="screen",
        parameters=[
            {
                "api_key_path": "/root/secrets/gemini_api_key.txt",
                "model_id": "gemini-3.6-flash",
                "thinking_budget": -1,
                "history_frames": 3,
                "structured_output": "off",
            }
        ],
    )

    trajectory_generator = Node(
        package="gemini_robotics_er",
        executable="trajectory_generator",
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

    narrative_navigation = Node(
        package="hint_narrative",
        executable="narrative_navigation",
        output="screen",
    )

    bt_executor = Node(
        package="hint_behavior",
        executable="behavior_server",
        output="screen",
        parameters=[
            {
                "action_name": "/hint_behavior_server/execute_behavior_tree",
                "behavior_trees": ["hint_behavior/behaviors"],
            }
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("hint_bringup"),
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
            visual_debug,
            trajectory_generator,
            nav2,
            trajectory_planner,
            visual_reasoner,
            narrative_navigation,
            bt_executor,
            rviz2,
        ]
    )
