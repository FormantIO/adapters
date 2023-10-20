#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from pcl import PointCloud
from pcl import VoxelGrid


def voxel_filter_callback(point_cloud_msg):
    # Convert PointCloud2 message to PCL PointCloud
    pcl_point_cloud = PointCloud()
    pcl_point_cloud.from_message(point_cloud_msg)

    # Create a Voxel Grid filter object
    voxel_grid = VoxelGrid()
    voxel_grid.setInputCloud(pcl_point_cloud)

    # Set the voxel grid size (adjust as needed)
    voxel_grid.setLeafSize(0.01, 0.01, 0.01)

    # Filter the point cloud
    filtered_point_cloud = PointCloud()
    voxel_grid.filter(filtered_point_cloud)

    # Publish the filtered point cloud
    filtered_point_cloud_msg = filtered_point_cloud.to_msg()
    filtered_point_cloud_msg.header = point_cloud_msg.header
    filtered_point_cloud_pub.publish(filtered_point_cloud_msg)


if __name__ == "__main__":
    rospy.init_node("voxel_filter_node")

    # Point cloud topics for input and output (adjust as needed)
    input_point_cloud_topic = "/input_point_cloud"
    output_point_cloud_topic = "/output_point_cloud"

    # Create subscribers and publishers
    rospy.Subscriber(input_point_cloud_topic, PointCloud2, voxel_filter_callback)
    filtered_point_cloud_pub = rospy.Publisher(
        output_point_cloud_topic, PointCloud2, queue_size=10
    )

    rospy.spin()


    
