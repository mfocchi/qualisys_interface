import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='qualisys',
            executable='qualisys_node',
            name='qualisys_node',
            output='screen',
        ),
    ])
