import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


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

    camera_remap = (
        "/camera/image_raw/compressed",
        "/camera/image_raw/compressed/throttle",
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
        parameters=[{"device": "GPU", "model": "segformer-b5-ade"}],
        remappings=[camera_remap],
    )

    obstacle_projector = Node(
        package="hint_navigation",
        executable="obstacle_projector",
        output="screen",
        parameters=[camera_rig],
    )

    path_projector = Node(
        package="hint_navigation",
        executable="path_projector",
        output="screen",
        parameters=[camera_rig],
    )

    path_planner = Node(
        package="hint_vlm",
        executable="path_planner",
        output="screen",
        parameters=[
            {
                "api_key_path": "/root/secrets/gemini_api_key.txt",
                "model_id": "gemini-robotics-er-1.6-preview",
                "thinking_budget": 0,
                "temperature": 1.0,
                "n_candidates": 1,
                "structured_output": "json",
            }
        ],
    )

    visual_reasoner = Node(
        package="hint_vlm",
        executable="visual_reasoner",
        output="screen",
        parameters=[
            {
                "api_key_path": "/root/secrets/gemini_api_key.txt",
                "model_id": "gemini-robotics-er-1.6-preview",
                "thinking_budget": -1,
                "structured_output": "json",
            }
        ],
    )

    narrative_navigation = Node(
        package="hint_narrative",
        executable="narrative_navigation",
        output="screen",
        parameters=[
            {
                "history_frames": 1,
            }
        ],
        remappings=[camera_remap],
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

    visual_debug = Node(
        package="hint_navigation",
        executable="visual_debug",
        parameters=[camera_rig],
        output="screen",
        remappings=[camera_remap],
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
            "--ros-args",
            "--log-level",
            "error",
        ],
    )

    return LaunchDescription(
        [
            teleop,
            nav2,
            cartographer,
            occupancy_grid,
            ground_segmenter,
            obstacle_projector,
            path_projector,
            path_planner,
            visual_reasoner,
            narrative_navigation,
            bt_executor,
            visual_debug,
            rviz2,
        ]
    )
