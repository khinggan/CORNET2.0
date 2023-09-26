import numpy as np
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
package_name = 'robot_spawner_pkg'


def generate_launch_description():
    # configure robots position and number
    robot_init_positions = np.array([
        [0.0, 0.0 , 0.0],
        [-2.0, 2.0, 0.0], 
        [-2.0, -2.0, 0.0]
    ])
    N = 3     # number of robots

    launch_description = [] # launched immediately 

    # add spawner for each robot
    for i in range(N):
        position = robot_init_positions[i, :].tolist()

        # turtlebot spawner
        launch_description.append(Node(
            package=package_name, executable='turtlebot3_spawner', output='screen',
            parameters=[{'namespace': 'agent_{}'.format(i), 'position': position}]))
    
    # add control node; configure which is leader, which are followers
    launch_description.append(Node(
        package=package_name, executable='vs', output='screen',
        parameters=[{
            'N': 3.0,    # 3 robots
            'ddistance': math.sqrt(3),  #  desired distance between each agents
        }]
    ))

    # launch_description.append(Node(
    #     package='rviz2',
    #     namespace='',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', ['/home/khinggan/my_research/colcon_ws/src/comm_based_mrs_formation/rviz/virtual_structure.rviz']]
    # ))
    
    # include launcher for gazebo
    # gazebo_launcher = os.path.join(get_package_share_directory(package_name), 'gazebo.launch.py')
    # launch_description.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launcher)))
    
    return LaunchDescription(launch_description)
