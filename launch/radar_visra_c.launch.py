from ast import arguments
from http.server import executable
import os

from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

        # Declare Launch Arguments
        radar1 = DeclareLaunchArgument(
                'radar1',
                default_value='radar1',
        )
        radar1_cfg = DeclareLaunchArgument(
                'radar1_cfg',
                default_value='1843/1843_doppler_v0.cfg',
        )
        radar1_cmd_tty = DeclareLaunchArgument(
                'radar1_cmd_tty',
                default_value='/dev/tty1843_00',
        )
        radar1_dca_ip = DeclareLaunchArgument(
                'radar1_dca_ip',
                default_value='192.168.33.180',
        )
        radar1_dca_cmd_port = DeclareLaunchArgument(
                'radar1_dca_cmd_port',
                default_value='4096',
        )
        radar1_host_ip = DeclareLaunchArgument(
                'radar1_host_ip',
                default_value='192.168.33.30',
        )
        radar1_host_cmd_port = DeclareLaunchArgument(
                'radar1_host_cmd_port',
                default_value='4096',
        )
        radar1_host_data_port = DeclareLaunchArgument(
                'radar1_host_data_port',
                default_value='4098',
        )

        # radar1_config = os.path.join(
        #         get_package_share_directory('xwr_raw_ros'),
        #         'config',
        #         LaunchConfiguration('radar1_cfg')
        #         )

        # Declare Nodes
        radar_visra_c = Node(
                package='xwr_raw_ros',
                executable='radar_cfg.py',
                name='xwr_radar',
                namespace=LaunchConfiguration('radar1') ,
                # remappings = [
                #     ("radar/scan",      "/ti_mmwave/radar_scan_pcl"),
                #     ("radar/trigger" ,  "/sensor_platform/radar_right/trigger"),
                # ],
                output='screen',
                parameters=[{"cfg": LaunchConfiguration('radar1_cfg'),
                             "cmd_tty": LaunchConfiguration('radar1_cmd_tty'),
                             "dca_ip": LaunchConfiguration('radar1_dca_ip'),
                             "dca_cmd_port": LaunchConfiguration('radar1_dca_cmd_port'),
                             "host_ip": LaunchConfiguration('radar1_host_ip'),
                             "host_cmd_port": LaunchConfiguration('radar1_host_cmd_port')}
                ],
                )
        
        # velocity_estimation_evaluator = Node(
        #         package='radar_ego_velocity_estimator',
        #         executable='velocity_estimation_evaluator',
        #         name='velocity_estimation_evaluator',
        #         parameters=[config],
        #         remappings = [
        #             ("radar/scan",      "/ti_mmwave/radar_scan_pcl"),
        #             ("radar/trigger" ,  "/sensor_platform/radar_right/trigger"),
        #         ],
        #         output='screen',
        #     )

        return LaunchDescription([GroupAction(
        actions=[
                radar1,
                radar1_cfg,
                radar1_cmd_tty,
                radar1_dca_ip,
                radar1_dca_cmd_port,
                radar1_host_ip,
                radar1_host_cmd_port,
                radar1_host_data_port,
                radar_visra_c,
        ])])
