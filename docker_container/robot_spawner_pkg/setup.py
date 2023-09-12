from setuptools import setup
from glob import glob

package_name = 'robot_spawner_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('urdf/*')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khinggan',
    maintainer_email='1516113201@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_turtlebot = robot_spawner_pkg.spawn_turtlebot:main', 
            'turtlebot3_spawner = robot_spawner_pkg.turtlebot3_spawner:main',
            'vs = comm_based_mrs_formation.vs:main',
        ],
    },
)
