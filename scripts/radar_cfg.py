#!/usr/bin/env python3

"""Simple publisher of raw radar data.
"""
import os
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
        cfg_path =os.path.join(get_package_share_directory('xwr_raw_ros'),
                'configs',
                self.cfg.get_parameter_value().string_value
                )
        with open(cfg_path, 'r') as f:
            cfg = f.readlines()
        radar_config = RadarConfig(cfg)
        # print(type(radar_config))
        # self.get_logger().info(str(radar_config.cmds))
        # for x in radar_config.cmds:
        #     self.get_logger().info(str(radar_config[x]))
        for x,y in dict(radar_config).items():
            self.get_logger().info(str(type(x)))
            self.get_logger().info(str(type(y)))

            # self.set_parameters(rclpy.Parameter(str(x)))

            # self.set_parameters()

        self.get_logger().info("Node started")


    

    
    
    # rospy.set_param('radar_config', dict(**radar_config))

    # # Extract params from config.
    # radar_params = radar_config.get_params()
    # rospy.set_param('radar_params', dict(**radar_params))

    # # Configure and start radar capture.
    # radar = RadarPub(cfg,
    #                  cmd_tty        = args.cmd_tty,
    #                  dca_ip         = args.dca_ip,
    #                  dca_cmd_port   = int(args.dca_cmd_port),
    #                  host_ip        = args.host_ip,
    #                  host_cmd_port  = int(args.host_cmd_port),
    #                  host_data_port = None)
    # radar.configure()
    # radar.start_capture()

    # rospy.on_shutdown(lambda : radar.close())

    # rate = rospy.Rate(1)
    # while True:
    #     rate.sleep()

# --------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    radar_cfg = RadarCfg()

    rclpy.spin(radar_cfg)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    radar_cfg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
