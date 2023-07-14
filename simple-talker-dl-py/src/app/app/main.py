"""Reads a ctrlX Data Layer value and sends it via ROS 2 message."""

# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

# ROS Client Library for Python
# https://docs.ros2.org/latest/api/rclpy/index.html
import rclpy

from ros2.datalayer_reader_ros2_publisher import DataLayerReaderRos2Publisher


def main(args=None):
    """Manage the usage of the ROS Client Library."""
    rclpy.init(args=args)

    ros2_publisher = DataLayerReaderRos2Publisher(
        dl_address='framework/metrics/system/cpu-utilisation-percent',
        ros2_topic='ctrlXCpuUtilisationPercent')

    result = ros2_publisher.start()
    if result is False:
        ros2_publisher.stop()
        ros2_publisher.destroy_node()
        return

    rclpy.spin(ros2_publisher)

    ros2_publisher.stop()
    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
