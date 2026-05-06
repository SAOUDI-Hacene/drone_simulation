import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_desc = get_package_share_directory('drone_description')
    pkg_gz   = get_package_share_directory('ros_gz_sim')

    urdf  = os.path.join(pkg_desc, 'urdf', 'drone.urdf.xacro')
    world = os.path.expanduser('~/drone_ws/src/drone_simulation/worlds/drone_world.sdf')

    return LaunchDescription([

        # 1. Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': world}.items()
        ),

        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf]),
                'use_sim_time': True
            }]
        ),

        # 3. Spawn بعد 5 ثوان
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name',  'drone',
                    '-topic', '/robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.3'
                ],
                output='screen'
            ),
        ]),

        # 4. Bridge بعد 3 ثوان
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/drone/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/drone/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                parameters=[{'use_sim_time': True}]
            ),
        ]),

    ])
