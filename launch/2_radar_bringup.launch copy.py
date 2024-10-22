from ast import arguments
from http.server import executable
import os

from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
# from launch.actions import p
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_ros.actions

def generate_launch_description():

        xwr_rawr_ros_dir = get_package_share_directory('xwr_raw_ros')

        tf_map_radar_0 = ['0.2', '-0.15', '0.0', '0.0', '0.0', '0.0', 'map', 'radar_0']
        tf_map_radar_1 = ['-0.2', '0.15', '0.0', '1.57079632679', '0.0', '0.0', 'map', 'radar_1']

        # Find launchs
        awr1843_launch = PathJoinSubstitution([    # doppler azimuth mode for awr1843 (front) 
            FindPackageShare('xwr_raw_ros'),
            'launch',
            'radar_visda_c_1843.launch.py'])
        
        awr1843aop_launch = PathJoinSubstitution([ # range azimuth mode for awr1843aop (rear) 
            FindPackageShare('xwr_raw_ros'),
            'launch',
            'radar_visra_c_1843aop.launch.py'])

        # Launch Descriptions
        awr1843 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(awr1843_launch),
                launch_arguments=[
                ('radar', 'radar_0'),
                ],
        )

        awr1843aop = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(awr1843aop_launch),
                launch_arguments=[
                ('radar', 'radar_1'),
                ],
        )
    
        Rviz2 = Node(
                package='rviz2',
                executable='rviz2',
                namespace=LaunchConfiguration('radar'),
                arguments=['-d', os.path.join(xwr_rawr_ros_dir, 'launch', 'rviz.rviz')]
        )

        tf_static_map_radar_0 = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name="tf_static_map_radar",
                arguments = tf_map_radar_0,
        )
        tf_static_map_radar_1 = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name="tf_static_map_radar",
                arguments = tf_map_radar_1,
        )

        return LaunchDescription([GroupAction(
        actions=[
                awr1843_launch,
                awr1843aop_launch,
                awr1843,
                awr1843aop,
                tf_static_map_radar_0,
                tf_static_map_radar_1,
                Rviz2
        ])])
