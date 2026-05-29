from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_state_transformer',
            executable='joint_state_transformer',
            name='joint_state_transformer',
            output='screen',
            parameters=[{
                'max_iterations': 100,
                'tolerance': 1e-5,
                'output_action_name': '/actuated_joint_trajectory_controller/follow_joint_trajectory',
                'input_action_name': '~/follow_joint_trajectory',
            }]
        )
    ])
