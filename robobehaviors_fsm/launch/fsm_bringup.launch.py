from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # finite state machine
        Node(
            package='robobehaviors_fsm',
            executable='finite_state_controller',
            name='finite_state_controller',
            output='screen'
        ),

        # wall follower behavior
        Node(
            package='robobehaviors_fsm',
            executable='wall_follower',
            name='wall_follower',
            output='screen'
        ),

        # # obstacle avoidance behavior
        # Node(
        #     package='robobehaviors_fsm',
        #     executable='obstacle_avoidance',
        #     name='obstacle_avoidance',
        #     output='screen'
        # ),

        # person following behavior
        Node(
            package='robobehaviors_fsm',
            executable='person_following',
            name='person_following',
            output='screen'
        ),

        # keyboard estop
        Node(
            package='robobehaviors_fsm',
            executable='keyboard_estop',
            name='keyboard_estop',
            output='screen'
        ),

        # bumper estop
        Node(
            package='robobehaviors_fsm',
            executable='bumper_estop',
            name='bumper_estop',
            output='screen'
        ),
    ])
