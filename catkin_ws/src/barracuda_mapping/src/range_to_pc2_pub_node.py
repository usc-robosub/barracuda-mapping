#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

range_to_pc2_pub = None

def on_receive_altimeter_data(data):
    global range_to_pc2_pub
    depth = data.range
    x, y, z = 0, 0, -depth
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "barracuda/ping1d_link"
    points = [(x, y, z)]
    pointcloud_msg = pc2.create_cloud_xyz32(header, points)
    range_to_pc2_pub.publish(pointcloud_msg)

def range_to_pc2_pub_node():
    global range_to_pc2_pub
    rospy.init_node('pointcloud2_pub_node', anonymous=True)
    rospy.Subscriber('ping1d/range', Range, on_receive_altimeter_data)
    range_to_pc2_pub = rospy.Publisher('/output_pointcloud', PointCloud2, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    range_to_pc2_pub_node()
    
    