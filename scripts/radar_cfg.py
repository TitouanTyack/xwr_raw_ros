#!/usr/bin/env python3

"""Simple publisher of raw radar data.
"""
import os
import rcl_interfaces.msg
import rcl_interfaces.srv
import rclpy
import sys
import time
import socket
import serial
import argparse
import numpy

from rclpy.node import Node
from xwr_raw_ros_msgs.msg import RadarFrame
from xwr_raw_ros_msgs.msg import RadarFrameFull
from xwr_raw_ros_msgs.msg import RadarFrameStamped
from xwr_raw_ros.radar_config import RadarConfig
from xwr_raw_ros.radar_pub import RadarPub
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from ament_index_python import get_package_share_directory

class RadarCfg(Node):
    # --------------------------------------------------------------------------------------
    def __init__(self):
        super().__init__('xwr_radar')

        # Read path to config file.
        self.cfg           = self.declare_parameter("cfg",           "")
        self.cmd_tty       = self.declare_parameter("cmd_tty",       '/dev/ttyACM0',   )
        self.dca_ip        = self.declare_parameter("dca_ip",        '192.168.33.180', )
        self.dca_cmd_port  = self.declare_parameter("dca_cmd_port",  4096,             )
        self.host_ip       = self.declare_parameter("host_ip",       '192.168.33.30',  )
        self.host_cmd_port = self.declare_parameter("host_cmd_port", 4096,             )

        # Parse and publish config file.
        cfg_path = os.path.join(get_package_share_directory('xwr_raw_ros'),
                'configs',
                self.cfg.get_parameter_value().string_value
                )
        with open(cfg_path, 'r') as f:
            cfg = f.readlines()
        radar_config = RadarConfig(cfg)
        self.declare_parameter("radar_config/Platform"      ,radar_config['Platform'])
        radar_params = radar_config.get_params()
        self.declare_parameter("radar_params/sdk"           ,radar_params['sdk']           )     
        self.declare_parameter("radar_params/platform"      ,radar_params['platform']      )          
        self.declare_parameter("radar_params/adc_output_fmt",int(radar_params['adc_output_fmt']),ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))                
        self.declare_parameter("radar_params/range_bias"    ,float(radar_params['range_bias'])  ,ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))            
        self.declare_parameter("radar_params/rx_phase_bias" ,radar_params['rx_phase_bias']      ,ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))               
        self.declare_parameter("radar_params/n_chirps"      ,int(radar_params['n_chirps'])      ,ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))          
        self.declare_parameter("radar_params/rx"            ,radar_params['rx']                 ,ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))    
        self.declare_parameter("radar_params/n_rx"          ,int(radar_params['n_rx'])          ,ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))      
        self.declare_parameter("radar_params/tx"            ,radar_params['tx']                 ,ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))    
        self.declare_parameter("radar_params/n_tx"          ,int(radar_params['n_tx'])          ,ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))      
        self.declare_parameter("radar_params/n_samples"     ,int(radar_params['n_samples'])     ,ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))           
        self.declare_parameter("radar_params/frame_size"    ,int(radar_params['frame_size'])    ,ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))            
        self.declare_parameter("radar_params/frame_time"    ,float(radar_params['frame_time'])  ,ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))            
        self.declare_parameter("radar_params/chirp_time"    ,float(radar_params['chirp_time'])  ,ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))            
        self.declare_parameter("radar_params/chirp_slope"   ,float(radar_params['chirp_slope']) ,ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))             
        self.declare_parameter("radar_params/sample_rate"   ,int(radar_params['sample_rate'])   ,ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))             
        self.declare_parameter("radar_params/velocity_max"  ,float(radar_params['velocity_max']),ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))              
        self.declare_parameter("radar_params/velocity_res"  ,float(radar_params['velocity_res']),ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))              
        self.declare_parameter("radar_params/range_max"     ,float(radar_params['range_max'])   ,ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))           
        self.declare_parameter("radar_params/range_res"     ,float(radar_params['range_res'])   ,ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))           

        self.get_logger().info("Node started")
        

        # Configure and start radar capture.
        self.radar = RadarPub(cfg,
                        cmd_tty        = self.cmd_tty.get_parameter_value().string_value,
                        dca_ip         = self.dca_ip.get_parameter_value().string_value,
                        dca_cmd_port   = int(self.dca_cmd_port.get_parameter_value().integer_value),
                        host_ip        = self.host_ip.get_parameter_value().string_value,
                        host_cmd_port  = int(self.host_cmd_port.get_parameter_value().integer_value),
                        host_data_port = None)
        self.radar.configure()
        self.radar.start_capture()



# --------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    radar_cfg = RadarCfg()


    rclpy.spin(radar_cfg)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    rate = radar_cfg.create_rate(1)
    try:
        rate.sleep()
    except KeyboardInterrupt:
        radar_cfg.radar.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
