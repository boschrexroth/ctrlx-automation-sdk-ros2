# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

# ROS Client Library for Python
# https://docs.ros2.org/latest/api/rclpy/index.html
import rclpy

from ros2.ros2_listener_ctrlx_provider import Ros2ListenerDataLayerProvider

def main(args=None):
    """
    The main function.

    Manages usage of the ROS Client Library
    """
    rclpy.init(args=args)

    ros2_publisher = Ros2ListenerDataLayerProvider(
        dl_address='ros2/listener/py/cpu-utilisation-percent',
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
