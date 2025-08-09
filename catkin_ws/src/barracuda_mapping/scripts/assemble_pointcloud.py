#!/usr/bin/env python3

import roslib
import rospy

from laser_assembler.srv import AssembleScans2, AssembleScans2Request, AssembleScans2Response
from sensor_msgs.msg import PointCloud2


class AssemblePointcloud:
    """
    Periodically assemble point clouds by polling a point_cloud2_assembler service

    This node is based on the example at http://wiki.ros.org/laser_assembler
    """

    def __init__(self):
        # Parameters
        self.interval = rospy.get_param("~interval", 1) # time window of scans to assemble (seconds)
        self.rate = rospy.get_param("~rate", 10) # how frequent to poll the service (Hz)

        self.pub_topic = rospy.get_param(
            "~pointcloud_topic", "/barracuda/mapping/assembled_pointcloud"
        )
        self.assembler_service = rospy.get_param(
            "~assembler_service", "assemble_scans2"
        )

        # ROS publisher
        self.pointcloud_pub = rospy.Publisher(self.pub_topic, PointCloud2, queue_size=1)

        # Start loop
        self.run()

    def run(self):
        r = rospy.Rate(self.rate)

        rospy.wait_for_service(self.assembler_service)

        # AssembleScans2 contains a PointCloud2
        assemble_scans = rospy.ServiceProxy(self.assembler_service, AssembleScans2)

        while not rospy.is_shutdown():
            try:
                # Request a cloud with scans occuring in the last interval (seconds)
                # resp = assemble_scans(rospy.get_rostime() - rospy.Duration(self.interval), rospy.get_rostime())
                resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
                self.pointcloud_pub.publish(resp.cloud)
            except rospy.ServiceException:
                pass
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("assembler_publisher")
    try:
        AssemblePointcloud()
    except rospy.ROSInterruptException:
        pass
