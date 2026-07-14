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

    lk_tracker = Node(
        package="visual_tracker",
        executable="lk_tracker",
        output="screen",
    )

    # Odometry-driven ground-plane re-projection tracker. Drop-in for the DIS
    # waypoint_tracker (same /waypoint_tracking topics); remapped to the
    # waypoint_tracker_node name so pursuit_servo's service clients still resolve.
    waypoint_tracker = Node(
        package="visual_tracker",
        executable="odom_waypoint_tracker",
        name="waypoint_tracker_node",
        output="screen",
    )

    visual_servo = Node(
        package="visual_servoing",
        executable="visual_servo",
        output="screen",
    )

    pursuit_servo = Node(
        package="visual_servoing",
        executable="pursuit_servo",
        output="screen",
    )

    description_detector = Node(
        package="gemini_robotics_er",
        executable="description_detector",
        output="screen",
        parameters=[{
            "api_key_path": "/root/secrets/gemini_api_key.txt",
            "model_id": "gemini-robotics-er-1.6-preview",
        }],
    )

    visual_question = Node(
        package="gemini_robotics_er",
        executable="visual_question",
        output="screen",
        parameters=[{
            "api_key_path": "/root/secrets/gemini_api_key.txt",
            "model_id": "gemini-robotics-er-1.6-preview",
        }],
    )

    trajectory_planner = Node(
        package="gemini_robotics_er",
        executable="trajectory_planner",
        output="screen",
        parameters=[{
            "api_key_path": "/root/secrets/gemini_api_key.txt",
            "model_id": "gemini-robotics-er-1.6-preview",
        }],
    )

    reasoner = Node(
        package="gemini_robotics_er",
        executable="reasoner",
        output="screen",
        parameters=[{
            "api_key_path": "/root/secrets/gemini_api_key.txt",
            "model_id": "gemini-2.5-flash",
            # The mission narrative compile is reasoning-heavy — give it a
            # thinking budget (tokens; 0 = off, -1 = dynamic). Tune for cost/quality.
            "thinking_budget": -1,
        }],
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
            #cartographer,
            #occupancy_grid,
            #lk_tracker,
            waypoint_tracker,
            #visual_servo,
            pursuit_servo,
            #description_detector,
            #visual_question,
            trajectory_planner,
            reasoner,
            mission_planner,
            bt_executor,
            rviz2,
        ]
    )
