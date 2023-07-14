"""Read the ctrlX Data Layer value and publish it via ROS 2 message."""

# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node

import std_msgs.msg

import ctrlxdatalayer
from ctrlxdatalayer.variant import Result, Variant

from ctrlx_datalayer.ctrlx_datalayer_helper import get_client

# Assignment table 'ROS 2 Message Data Type' <-> 'ctrlX Data Layer Variant
# gettter method'.
CTRLX_TO_ROS2_CONVERTERS = {
    'INT8': { 
        'ctrlxGetter': Variant.get_int8,
        'msgDataType': std_msgs.msg.Int8},
    'INT16': {
        'ctrlxGetter': Variant.get_int16,
        'msgDataType': std_msgs.msg.Int16},
    'INT32': {
        'ctrlxGetter': Variant.get_int32,
        'msgDataType': std_msgs.msg.Int32},
    'INT64': {
        'ctrlxGetter': Variant.get_int64,
        'msgDataType': std_msgs.msg.Int64},
    'UINT8': {
        'ctrlxGetter': Variant.get_uint8,
        'msgDataType': std_msgs.msg.UInt8},
    'UINT16': {
        'ctrlxGetter': Variant.get_uint16,
        'msgDataType': std_msgs.msg.UInt16},
    'UINT32': {
        'ctrlxGetter': Variant.get_uint32,
        'msgDataType': std_msgs.msg.UInt32},
    'UINT64': {
        'ctrlxGetter': Variant.get_uint64,
        'msgDataType': std_msgs.msg.UInt64},
    'FLOAT32': {
        'ctrlxGetter': Variant.get_float32,
        'msgDataType': std_msgs.msg.Float32},
    'FLOAT64': {
        'ctrlxGetter': Variant.get_float64,
        'msgDataType': std_msgs.msg.Float64},
    'STRING': {
        'ctrlxGetter': Variant.get_string,
        'msgDataType': std_msgs.msg.String},
    'BOOL8': {
        'ctrlxGetter': Variant.get_bool8,
        'msgDataType': std_msgs.msg.Bool},
    'ARRAY_INT8': {
        'ctrlxGetter': Variant.get_array_int8,
        'msgDataType': std_msgs.msg.Int8MultiArray},
    'ARRAY_INT16': {
        'ctrlxGetter': Variant.get_array_int16,
        'msgDataType': std_msgs.msg.Int16MultiArray},
    'ARRAY_INT32': {
        'ctrlxGetter': Variant.get_array_int32,
        'msgDataType': std_msgs.msg.Int32MultiArray},
    'ARRAY_INT64': {
        'ctrlxGetter': Variant.get_array_int64,
        'msgDataType': std_msgs.msg.Int64MultiArray},
    'UInt8MultiArray':  {
        'ctrlxGetter': Variant.get_array_uint8,
        'msgDataType': std_msgs.msg.UInt8MultiArray},
    'UInt16MultiArray': {
        'ctrlxGetter': Variant.get_array_uint16,
        'msgDataType': std_msgs.msg.UInt16MultiArray},
    'UInt32MultiArray': {
        'ctrlxGetter': Variant.get_array_uint32,
        'msgDataType': std_msgs.msg.UInt32MultiArray},
    'UInt64MultiArray': {
        'ctrlxGetter': Variant.get_array_uint64,
        'msgDataType': std_msgs.msg.UInt64MultiArray},
    'Float32MultiArray': {
        'ctrlxGetter': Variant.get_array_float32,
        'msgDataType': std_msgs.msg.Float32MultiArray},
    'Float64MultiArray': {
        'ctrlxGetter': Variant.get_array_float64,  
        'msgDataType': std_msgs.msg.Float64MultiArray}}


class DataLayerReaderRos2Publisher(Node):
    """
    Class DataLayerReaderRos2Publisher.

    Reads a ctrlX Data Layer varinat value and sends it via ROS 2 message.
    """

    def __init__(self, dl_address, ros2_topic):
        """
        Initialize an instance.

        Constructor of the class.
        """
        self.dl_address = dl_address
        self.ros2_topic = ros2_topic

        self.datalayer_system = None
        self.datalayer_client = None
        self.getter_method = None
        self.ros2_msg_data_type = None
        self.ros2_publisher = None
        self.timer = None

    def read_data_layer_value(self):
        """
        Read a ctrlX Data Layer value.

        Returns the value as variant.
        """
        result, variant = self.datalayer_client.read_sync(self.dl_address)
        if result != Result.OK:
            print('ERROR Reading ctrlX Data Layer node', self.dl_address, 'failed', result)
            return None

        return variant

    def start(self):
        """
        Start running.

        Connects to the ctrlX Data Layer, initializes the rclpy super class 
        instance and starts the timer.
        """
        self.datalayer_system = ctrlxdatalayer.system.System('')
        self.datalayer_system.start(False)  

        print('INFO Creating ctrlX Data Layer connnection')
        self.datalayer_client, connection_string = get_client(self.datalayer_system)
        if self.datalayer_client is None:
            print('ERROR Creating ctrlX Data Layer connnection to', connection_string, 'failed')
            return False

        variant = self.read_data_layer_value()
        if variant is None:
            return False

        print('INFO Initializing ROS 2 node ctrlxReaderRos2Talker')
        super().__init__('ctrlxReaderRos2Talker')

        ctrlx_to_ros2_converter = CTRLX_TO_ROS2_CONVERTERS[variant.get_type().name]
        print('INFO ros2_msg', ctrlx_to_ros2_converter)

        self.getter_method = ctrlx_to_ros2_converter['ctrlxGetter']
        self.ros2_msg_data_type = ctrlx_to_ros2_converter['msgDataType']
        print('INFO Creating ROS 2 publisher: type', self.ros2_msg_data_type, 'topic', self.ros2_topic)
        self.ros2_publisher = self.create_publisher(self.ros2_msg_data_type, self.ros2_topic, 10)
        if self.ros2_publisher is None:
            print('ERROR Creating ROS 2 publisher failed')
            return False

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        return True

    def timer_callback(self):
        """
        Handle timer tick.

        Reads a ctrlX Data Layer values and sends it.
        """
        variant = self.read_data_layer_value()
        if variant is None:
            return

        ros2_msg = self.ros2_msg_data_type()
        ros2_msg.data = self.getter_method(variant)

        with variant:
            self.get_logger().info('Publishing "%s"' % ros2_msg.data)
            self.ros2_publisher.publish(ros2_msg)

    def stop(self):
        """
        Stop the app.

        Stops the ctrlX Data Layer and the timer.
        """
        self.datalayer_system = ctrlxdatalayer.system.System('')
        self.datalayer_system.stop(False)

        if self.timer is not None:
            self.destroy_timer(self.timer)
