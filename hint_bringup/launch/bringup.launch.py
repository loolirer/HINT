import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    teleop_config = os.path.join(
        get_package_share_directory("hint_bringup"), "config", "teleop.yaml"
    )

    region = LaunchConfiguration("region")
    declare_region = DeclareLaunchArgument(
        "region", default_value="default",
        description="Environment name under hint_navigation/maps/<region>/ to localize on",
    )

    camera_rig = {
        "camera_height": 0.105,
        "camera_forward_offset": 0.073,
        "camera_tilt": -0.025,
        "camera_hfov_deg": 62.2,
    }

    vlm_timeout = 60.0
    tf_buffer_time = 2.0 * vlm_timeout + 10.0

    path_range = 5.0

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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hint_navigation"),
                "launch",
                "localization.launch.py",
            )
        ),
        launch_arguments={"region": region}.items(),
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
        parameters=[camera_rig, {"tf_buffer_time": tf_buffer_time, "path_range": path_range}],
    )

    path_planner = Node(
        package="hint_vlm",
        executable="visual_reasoner",
        name="path_planner",
        output="screen",
        parameters=[
            {
                "api_key_path": "/root/secrets/gemini_api_key.txt",
                "model_id": "gemini-robotics-er-1.6-preview",
                "thinking_budget": 0,
                "temperature": 1.0,
                "structured_output": "json",
                "api_timeout": vlm_timeout,
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
                "api_timeout": vlm_timeout,
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
                "reasoner_timeout": vlm_timeout,
                "planner_timeout": vlm_timeout,
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
            declare_region,
            localization,
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
