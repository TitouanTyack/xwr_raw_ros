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
        ti_mmwave_dir = get_package_share_directory('ti_mmwave_rospkg')

        radar1                = DeclareLaunchArgument('radar1',               default_value='radar2')
        radar1_cfg            = DeclareLaunchArgument('radar1_cfg',           default_value='1843aop/1843aop_range_v1.cfg')
        radar1_cmd_tty        = DeclareLaunchArgument('radar1_cmd_tty',       default_value='/dev/ttyISK_00')
        radar1_data_tty       = DeclareLaunchArgument('radar1_data_tty',      default_value='/dev/ttyISK_01')
        radar1_dca_ip         = DeclareLaunchArgument('radar1_dca_ip',        default_value='192.168.34.181')
        radar1_dca_cmd_port   = DeclareLaunchArgument('radar1_dca_cmd_port',  default_value='4096')
        radar1_host_ip        = DeclareLaunchArgument('radar1_host_ip',       default_value='192.168.34.31')
        radar1_host_cmd_port  = DeclareLaunchArgument('radar1_host_cmd_port', default_value='4096')
        radar1_host_data_port = DeclareLaunchArgument('radar1_host_data_port',default_value='4098')

        config_path = DeclareLaunchArgument('config_path', default_value=[xwr_rawr_ros_dir,'/configs/',LaunchConfiguration('radar1_cfg')])
        tf_map_radar1 = ['-0.2', '0.15', '0.0', '1.57079632679    ', '0.0', '0.0', 'map', 'radar2']

        # Declare Nodes
        radar_cfg = Node(
                package='xwr_raw_ros',
                executable='radar_cfg.py',
                name='xwr_radar',
                namespace=LaunchConfiguration('radar1'),
                output='screen',
                parameters=[{"cfg": LaunchConfiguration('config_path'),
                             "host_ip": LaunchConfiguration('radar1_host_ip'),
                             "host_data_port": LaunchConfiguration('radar1_host_data_port'),
                             "host_cmd_port": LaunchConfiguration('radar1_host_cmd_port'),
                             "cmd_tty":LaunchConfiguration('radar1_cmd_tty'),
                             "dca_ip":LaunchConfiguration('radar1_dca_ip'),}
                ],
                )
        
        recver = Node(
                package='xwr_raw_ros',
                executable='recver',
                name='xwr_recver',
                namespace=LaunchConfiguration('radar1'),
                output='screen',
                parameters=[{"host_ip": LaunchConfiguration('radar1_host_ip'),
                             "host_data_port": LaunchConfiguration('radar1_host_data_port')}
                ],
                prefix="bash -c 'sleep 2.0; $0 $@' ",
                )
        
        visra = Node(
                package='xwr_raw_ros',
                executable='visra.py',
                name='xwr_ra_visualizer',
                namespace=LaunchConfiguration('radar1'),
                output='screen',
                )
        
        DataHandlerClass = Node(
                package="ti_mmwave_rospkg",
                executable="DataHandlerClass",
                name="DataHandlerClass",
                namespace=LaunchConfiguration('radar1'),
                output="screen",
                emulate_tty=True,
                parameters=[
                {"mmwavecli_name": "/mmWaveCLI"},
                {"mmwavecli_cfg": LaunchConfiguration('config_path')},
                {"data_port": LaunchConfiguration('radar1_data_tty')},
                {"data_rate": "921600"},
                {"frame_id": LaunchConfiguration('radar1')}
                ],
                prefix="bash -c 'sleep 2.0; $0 $@' ",
                )
        
        tf_static_map_radar = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name="tf_static_map_radar",
                arguments = tf_map_radar1,
        )
    
        Rviz2 = Node(
                package='rviz2',
                executable='rviz2',
                namespace=LaunchConfiguration('radar1'),
                arguments=['-d', os.path.join(ti_mmwave_dir, 'launch', 'rviz.rviz')]
        )

        return LaunchDescription([GroupAction(
        actions=[
                radar1,
                radar1_cfg,
                radar1_cmd_tty,
                radar1_data_tty,
                radar1_dca_ip,
                radar1_dca_cmd_port,
                radar1_host_ip,
                radar1_host_cmd_port,
                radar1_host_data_port,
                config_path,
                radar_cfg,
                recver,
                visra,
                DataHandlerClass,
                tf_static_map_radar,
                Rviz2
        ])])