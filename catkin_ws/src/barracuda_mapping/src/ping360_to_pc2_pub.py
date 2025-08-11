#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from math import cos, sin, radians
from sensor_msgs.msg import LaserScan


ping360_to_pc2_pub = None

def on_receive_ping360_data(data):
    global ping360_to_pc2_pub

    points = []
    for i, r in enumerate(data.ranges):
        angle = data.angle_min + i * data.angle_increment
        x = r * cos(angle)
        y = r * sin(angle)
        z = 0
        points.append((x, y, z))

    header = Header()
    header.stamp = data.header.stamp
    header.frame_id = "barracuda/ping_360_link"
    pointcloud_msg = pc2.create_cloud_xyz32(header, points)

    ping360_to_pc2_pub.publish(pointcloud_msg)

def ping360_to_pc2_pub_node():
    global ping360_to_pc2_pub
    rospy.init_node('ping360_to_pc2_pub_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, on_receive_ping360_data) # check topic name!
    ping360_to_pc2_pub = rospy.Publisher('/ping360_pointcloud', PointCloud2, queue_size=10) # check topic name!
    rospy.spin()

if __name__ == '__main__':
    ping360_to_pc2_pub_node()