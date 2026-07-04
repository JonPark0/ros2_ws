from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # 비전 노드 (YOLO 모델 로딩에 시간 소요)
    vision_node = Node(
        package='vision_pkg',
        executable='vision_node',
        name='vision_node',
        output='screen',
    )

    gripper_node = Node(
        package='arm_controller_pkg',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
    )

    cargo_manager_node = Node(
        package='arm_controller_pkg',
        executable='cargo_manager_node',
        name='cargo_manager_node',
        output='screen',
    )

    # 오케스트레이터: 다른 노드들이 뜬 뒤 3초 후 시작
    amr_robot_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='arm_controller_pkg',
                executable='amr_robot_node',
                name='amr_robot_node',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        vision_node,
        gripper_node,
        cargo_manager_node,
        amr_robot_node,
    ])
