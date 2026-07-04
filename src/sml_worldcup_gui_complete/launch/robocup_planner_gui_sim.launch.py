"""Launch the RoboCup Planner V2 simulation with the World Cup GUI."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    side = LaunchConfiguration("side")
    tier = LaunchConfiguration("tier")
    stage = LaunchConfiguration("stage")

    planner_share = get_package_share_directory("robocup_planner")
    simulation_launch = PythonLaunchDescriptionSource(
        [
            planner_share,
            "/launch/robocup_planner_sim.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("side", default_value="a"),
            DeclareLaunchArgument("tier", default_value="beginner"),
            DeclareLaunchArgument("stage", default_value="lifecycle"),
            Node(
                package="sml_worldcup_gui",
                executable="sml_worldcup_gui",
                name="sml_worldcup_gui",
                output="screen",
                parameters=[
                    {
                        "side": side,
                        "topic_name": "/eai/task",
                        "planner_state_topic": "/sml/test/planner_plan",
                        "manager_status_topic": "/sml/test/manager_status",
                        "manual_order_mode": True,
                        "task_template_topic": "/eai/task_template",
                        "auto_tier": tier,
                        "enable_location_monitor": False,
                    }
                ],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    IncludeLaunchDescription(
                        simulation_launch,
                        launch_arguments={
                            "side": side,
                            "tier": tier,
                            "stage": stage,
                            "order_task_topic": "/eai/task_template",
                            "shutdown_after_execution": "false",
                        }.items(),
                    ),
                ],
            ),
        ]
    )
