#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, Range


class RangeToPointcloud:
    """
    Projects a range reading (sensor_msgs/Range) into a point cloud (sensor_msgs/PointCloud2).
    The generated point cloud is in the same frame as the original range reading.
    """

    FIELDS = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]

    def __init__(self):
        # Parameters
        self.sub_topic = rospy.get_param("~range_topic", "depth")
        self.pub_topic = rospy.get_param(
            "~pointcloud_topic", "mapping/range_pointcloud"
        )

        # ROS publisher
        self.pointcloud_pub = rospy.Publisher(self.pub_topic, PointCloud2, queue_size=1)

        # ROS subscriber
        self.range_sub = rospy.Subscriber(
            self.sub_topic, Range, self.range_cb, queue_size=1
        )

    def range_cb(self, msg):
        # convert the message of type LaserScan to a PointCloud2
        points = [[msg.range, 0, 0]]
        pc2_msg = point_cloud2.create_cloud(msg.header, self.FIELDS, points)

        # publish it
        self.pointcloud_pub.publish(pc2_msg)


if __name__ == "__main__":
    rospy.init_node("laserscan_to_pointcloud")
    try:
        RangeToPointcloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
