"""Listens for ROS 2 messages and stores them into the ctrlX Data Layer."""

# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node

import std_msgs.msg

import ctrlxdatalayer
from ctrlxdatalayer.variant import Result, Variant

from ctrlx_datalayer.ctrlx_datalayer_helper import get_provider
from ctrlx_datalayer.ctrlx_provider_node import CtrlXProviderNode


class Ros2ListenerDataLayerProvider(Node):
    """
    Class Ros2ListenerDataLayerProvider.

    Receives ROS 2 messages and stores the received values into a ctrlX Data Layer node.
    """

    def __init__(self, dl_address, ros2_topic):
        """Initialize the instance."""
        self.dl_address = dl_address
        self.ros2_topic = ros2_topic

        self.datalayer_system = None
        self.datalayer_provider = None
        self.variant = Variant()
        self.provider_node = None

        self.ros2_subscription = None
        self.ros2_msg_data = None

    def read_data_layer_value(self):
        """Read the value from the ctrlX Data Layer."""
        print('INFO Reading ctrlX Data Layer node', self.dl_address, ' - BEGIN')
        result, variant = self.datalayer_provider.read_sync(self.dl_address)
        print('INFO Reading ctrlX Data Layer node', self.dl_address, ' - END')
        if result != Result.OK:
            print('ERROR Reading ctrlX Data Layer node', self.dl_address, 'failed', result)
            return None

        print('INFO Data type ', variant.get_type().name)
        return variant
    
    def start(self):
        """Start the activity."""
        self.datalayer_system = ctrlxdatalayer.system.System('')
        self.datalayer_system.start(False)  

        print('INFO Creating ctrlX Data Layer connnection')
        self.datalayer_provider, connection_string = get_provider(self.datalayer_system)
        if self.datalayer_provider is None:
            print('ERROR Creating ctrlX Data Layer connnection to', connection_string, 'failed')
            return False
        self.datalayer_provider.start()

        self.variant.set_float64(-1.0)
        self.provider_node = CtrlXProviderNode(self.datalayer_provider, self.dl_address, self.variant)
        result = self.provider_node.register_node()
        if result != Result.OK:
            print('ERROR Registering', self.dl_address, 'failed')
            return False

        print('INFO Initializing ROS 2 node Ros2ListenerCtrlxProvider')
        super().__init__('Ros2ListenerCtrlxProvider')

        print('INFO Creating ROS 2 listener to topic', self.ros2_topic)
        self.ros2_subscription = self.create_subscription(std_msgs.msg.Float64, self.ros2_topic, self.listener_callback, 10)
        if self.ros2_subscription is None:
            print('ERROR Creating ROS 2 subscription failed')
            return False

        return True

    def listener_callback(self, msg):
        """Get the ROS 2 message data and store it into ctrlX Data Layer."""
        self.get_logger().info('Received "%s"' % msg.data)
        self.ros2_msg_data = msg.data
        self.variant.set_float64(msg.data)
        self.provider_node.set_value(self.variant)    

    def stop(self):
        """Stop the activity."""
        self.datalayer_system = ctrlxdatalayer.system.System('')
        self.datalayer_system.stop(False)

        if self.timer is not None:
            self.destroy_timer(self.timer)
