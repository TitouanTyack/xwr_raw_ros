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
        radar_name = ('1843') # {1843, 1843aop}

        ti_mmwave_dir = get_package_share_directory('ti_mmwave_rospkg')

        
        if(radar_name=='1843'):
                # Find launchs
                radar_launch = PathJoinSubstitution([    # doppler azimuth mode for awr1843 (front) 
                FindPackageShare('xwr_raw_ros'),
                'launch',
                'radar_visda_c_1843.launch.py'])
                # Launch Descriptions
                radar = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(radar_launch),
                        launch_arguments=[
                        ('radar', 'radar_0'),
                        ],
                )
                tf_map_radar = ['0.2', '-0.15', '0.0', '0.0', '0.0', '0.0', 'map', 'radar_0']
                
        elif(radar_name=='1843aop'):
                # Find launchs
                radar_launch = PathJoinSubstitution([ # range azimuth mode for awr1843aop (rear) 
                FindPackageShare('xwr_raw_ros'),
                'launch',
                'radar_visra_c_1843aop.launch.py'])
                # Launch Descriptions
                radar = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(radar_launch),
                        launch_arguments=[
                        ('radar', 'radar_1'),
                        ],
                )
                tf_map_radar = ['-0.2', '0.15', '0.0', '1.57079632679', '0.0', '0.0', 'map', 'radar_1']

        Rviz2 = Node(
                package='rviz2',
                executable='rviz2',
                namespace=LaunchConfiguration('radar'),
                arguments=['-d', os.path.join(ti_mmwave_dir, 'launch', 'rviz.rviz')]
        )

        tf_static_map_radar = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name="tf_static_map_radar",
                arguments = tf_map_radar,
        )

        return LaunchDescription([GroupAction(
        actions=[
                radar_launch,
                radar,
                tf_static_map_radar,
                Rviz2
        ])])
