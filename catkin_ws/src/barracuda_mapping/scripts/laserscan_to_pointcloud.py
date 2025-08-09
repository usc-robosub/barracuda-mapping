#!/usr/bin/env python3

import math
import rospy
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg

from sensor_msgs.msg import PointCloud2, LaserScan


class LaserscanToPointcloud:
    """
    Projects a 2D laser scan (sensor_msgs/LaserScan) into a point cloud (sensor_msgs/PointCloud2).
    The generated point cloud is in the same frame as the original laser scan.

    This node is based on the example at http://wiki.ros.org/laser_geometry
    """

    def __init__(self):
        # Parameters
        self.sub_topic = rospy.get_param("~laserscan_topic", "/barracuda/scan")
        self.pub_topic = rospy.get_param(
            "~pointcloud_topic", "/barracuda/mapping/scan_pointcloud"
        )

        # Initialize LaserProjection
        self.lp = lg.LaserProjection()

        # ROS publisher
        self.pointcloud_pub = rospy.Publisher(self.pub_topic, PointCloud2, queue_size=1)

        # ROS subscriber
        self.laser_sub = rospy.Subscriber(
            self.sub_topic, LaserScan, self.scan_cb, queue_size=1
        )

    def scan_cb(self, msg):
        # convert the message of type LaserScan to a PointCloud2
        pc2_msg = self.lp.projectLaser(msg)

        # now we can do something with the PointCloud2 for example:
        # publish it
        self.pointcloud_pub.publish(pc2_msg)


if __name__ == "__main__":
    rospy.init_node("laserscan_to_pointcloud")
    try:
        LaserscanToPointcloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
