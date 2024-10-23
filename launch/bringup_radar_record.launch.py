from ast import arguments
from http.server import executable
import os

from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction, ExecuteProcess, OpaqueFunction
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

from datetime import datetime

now = datetime.now()
date_str = now.strftime("%Y%m%d-%H%M%S")

def launch_setup_pcl_imu(context, *args,**kwargs):

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                '--output', f'bag/radar_log/isae_meca_pcl_imu-{date_str}',
                '--max-bag-duration', str(60*10),
                '/imu/data',
                '/radar_0/ti_mmwave/radar_scan_pcl',
                '/radar_1/ti_mmwave/radar_scan_pcl',
            ],
            output={'stdout': 'log', 'stderr': 'log'}
        )
    ]

def launch_setup_raw_data(context, *args,**kwargs):

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                '--output', f'bag/radar_log/isae_meca_raw_data-{date_str}',
                '--max-bag-duration', str(60*10),
                '/radar_0/radar_data',
                '/radar_1/radar_data',
            ],
            output={'stdout': 'log', 'stderr': 'log'}
        )
    ]

def launch_setup_full(context, *args,**kwargs):

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                '--output', f'bag/radar_log/isae_meca_full-{date_str}',
                '--max-bag-duration', str(60*10),
                '/imu/data',
                '/radar_0/radar_data',
                '/radar_0/ti_mmwave/radar_scan_pcl',
                '/radar_1/radar_data',
                '/radar_1/ti_mmwave/radar_scan_pcl',
            ],
            output={'stdout': 'log', 'stderr': 'log'}
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup_pcl_imu),
            # OpaqueFunction(function=launch_setup_raw_data),
            OpaqueFunction(function=launch_setup_full),
        ]
    )

