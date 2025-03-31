from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        Node(
            package='ros2_project_xxx',
            executable='robot_controller',
            name='robot_controller',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
            
        Node(
            package='ros2_project_xxx',
            executable='color_detector',
            name='color_detector',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
