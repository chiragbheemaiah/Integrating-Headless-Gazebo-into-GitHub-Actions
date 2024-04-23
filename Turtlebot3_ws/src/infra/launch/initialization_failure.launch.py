from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the 'headless' launch argument
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to run the simulation in headless mode'
    )

    ld.add_action(headless_arg)

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to use RVIZ'
    )

    ld.add_action(use_rviz_arg)

    # Include the tb3_simulation_launch file with headless argument
    tb3_simulation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch/tb3_simulation_launch.py'
            )
        ),
        launch_arguments={'headless': LaunchConfiguration('headless'),
                          'use_rviz': LaunchConfiguration('use_rviz')}.items()
    )

    # Add nodes
    publish_initial_pose = Node(
        package='infra',
        executable='publish_initial_pose',
        parameters=[
            {'x_position': -2.0}, 
            {'y_position': -0.5},
            {'z_position': 0.01}
        ],
        output='screen'
    )
    
    publish_goal = Node(
        package='infra',
        executable='publish_goal_failure',
        output='screen'
    )

    # Add actions to the LaunchDescription
    ld.add_action(tb3_simulation_launch_file)
    ld.add_action(publish_initial_pose)
    ld.add_action(publish_goal)

        # Add a timer to shut down the system after 150 seconds
    shutdown_timer = TimerAction(
        period=60.0,
        actions=[Node(
            package='launch',
            executable='emit_event',
            arguments=['shutdown'],
            output='screen'
        )]
    )
    ld.add_action(shutdown_timer)

    return ld
