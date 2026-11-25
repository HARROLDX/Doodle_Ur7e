"""Start sensors and MoveIt only. Does NOT enable comms or start drawing."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'),'launch','rs_launch.py'])),
        launch_arguments={'pointcloud.enable':'true','rgb_camera.color_profile':'1920x1080x30'}.items(),
        condition=IfCondition(LaunchConfiguration('camera')))
    moveit = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('ur_moveit_config'),'launch','ur_moveit.launch.py'])),
        launch_arguments={'ur_type':'ur7e','launch_rviz':'true','launch_servo':'false'}.items(),
        condition=IfCondition(LaunchConfiguration('moveit')))
    mount = Node(package='doodle_ur7e',executable='doodle',
                 arguments=['--config',LaunchConfiguration('config'),'mount'],
                 condition=IfCondition(LaunchConfiguration('mount')),output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('config',description='Absolute path to measured local.yaml'),
        DeclareLaunchArgument('camera',default_value='true'),
        DeclareLaunchArgument('moveit',default_value='true'),
        DeclareLaunchArgument('mount',default_value='true'),camera,moveit,mount])
