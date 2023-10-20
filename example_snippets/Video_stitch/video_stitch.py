#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from image_geometry import StereoCameraModel


class VideoStitcherNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.stitched_publisher = rospy.Publisher(
            "/stitched_video", Image, queue_size=10
        )
        self.left_image_sub = rospy.Subscriber(
            "/left_video_topic", Image, self.left_image_callback
        )
        self.right_image_sub = rospy.Subscriber(
            "/right_video_topic", Image, self.right_image_callback
        )

        self.left_image = None
        self.right_image = None

    def left_image_callback(self, data):
        self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.stitch_images()

    def right_image_callback(self, data):
        self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.stitch_images()

    def stitch_images(self):
        if self.left_image is not None and self.right_image is not None:
            # Assuming both images have the same height
            height, width, _ = self.left_image.shape

            # Stitch the images side by side
            stitched_image = cv2.hconcat([self.left_image, self.right_image])

            # Create an Image message and publish it
            stitched_msg = self.bridge.cv2_to_imgmsg(stitched_image, "bgr8")
            self.stitched_publisher.publish(stitched_msg)


def main():
    rospy.init_node("video_stitcher_node")
    video_stitcher = VideoStitcherNode()
    rospy.spin()


if __name__ == "__main__":
    main()
