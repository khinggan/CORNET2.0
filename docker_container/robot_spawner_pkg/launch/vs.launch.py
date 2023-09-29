import math
from launch_ros.actions import Node
from launch import LaunchDescription
package_name = 'robot_spawner_pkg'


def generate_launch_description():
    launch_description = [] # launched immediately 

    # add control node; configure which is leader, which are followers
    launch_description.append(Node(
        package=package_name, executable='vs', output='screen',
        parameters=[{
            'N': 3.0,    # 3 robots
            'ddistance': math.sqrt(3),  #  desired distance between each agents
        }]
    ))
    
    return LaunchDescription(launch_description)
