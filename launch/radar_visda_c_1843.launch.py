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

        # Declare Launch Arguments
        radar                = DeclareLaunchArgument('radar',               default_value='radar_0')
        radar_cfg            = DeclareLaunchArgument('radar_cfg',           default_value='1843/1843_doppler_v1.cfg')
        radar_cmd_tty        = DeclareLaunchArgument('radar_cmd_tty',       default_value='/dev/tty1843_00')
        radar_data_tty       = DeclareLaunchArgument('radar_data_tty',      default_value='/dev/tty1843_03')
        radar_dca_ip         = DeclareLaunchArgument('radar_dca_ip',        default_value='192.168.33.180')
        radar_dca_cmd_port   = DeclareLaunchArgument('radar_dca_cmd_port',  default_value='4096')
        radar_host_ip        = DeclareLaunchArgument('radar_host_ip',       default_value='192.168.33.30')
        radar_host_cmd_port  = DeclareLaunchArgument('radar_host_cmd_port', default_value='4096')
        radar_host_data_port = DeclareLaunchArgument('radar_host_data_port',default_value='4098')

        config_path = DeclareLaunchArgument('config_path', default_value=[xwr_rawr_ros_dir,'/configs/',LaunchConfiguration('radar_cfg')])
        tf_map_radar = ['0.5', '-0.15', '0.0', '0.0', '0.0', '0.0', 'map', 'radar_0']

        # Declare Nodes
        radar_cfg = Node(
                package='xwr_raw_ros',
                executable='radar_cfg.py',
                name='xwr_radar',
                namespace=LaunchConfiguration('radar') ,
                output='screen',
                parameters=[{"cfg": LaunchConfiguration('config_path'),
                             "host_ip": LaunchConfiguration('radar_host_ip'),
                             "host_data_port": LaunchConfiguration('radar_host_data_port'),
                             "host_cmd_port": LaunchConfiguration('radar_host_cmd_port'),
                             "cmd_tty":LaunchConfiguration('radar_cmd_tty'),
                             "dca_ip":LaunchConfiguration('radar_dca_ip'),}
                ],
                )
        
        recver = Node(
                package='xwr_raw_ros',
                executable='recver',
                name='xwr_recver',
                namespace=LaunchConfiguration('radar') ,
                output='screen',
                parameters=[{"host_ip": LaunchConfiguration('radar_host_ip'),
                             "host_data_port": LaunchConfiguration('radar_host_data_port')}
                ],
                prefix="bash -c 'sleep 2.0; $0 $@' ",
                )
        
        visda = Node(
                package='xwr_raw_ros',
                executable='visda.py',
                name='xwr_da_visualizer',
                namespace=LaunchConfiguration('radar') ,
                output='screen',
                )
        
        DataHandlerClass = Node(
                package="ti_mmwave_rospkg",
                executable="DataHandlerClass",
                name="DataHandlerClass",
                output="screen",
                emulate_tty=True,
                namespace=LaunchConfiguration('radar'),
                parameters=[
                {"mmwavecli_name": "/mmWaveCLI"},
                {"mmwavecli_cfg": LaunchConfiguration('config_path')},
                {"data_port": LaunchConfiguration('radar_data_tty')},
                {"data_rate": "921600"},
                {"frame_id": LaunchConfiguration('radar')}
                ],
                prefix="bash -c 'sleep 2.0; $0 $@' ",
                )

        return LaunchDescription([GroupAction(
        actions=[
                radar,
                radar_cfg,
                radar_cmd_tty,
                radar_data_tty,
                radar_dca_ip,
                radar_dca_cmd_port,
                radar_host_ip,
                radar_host_cmd_port,
                radar_host_data_port,
                config_path,
                radar_cfg,
                recver,
                visda,
                DataHandlerClass,
        ])])
