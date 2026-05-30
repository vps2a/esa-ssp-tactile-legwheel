#!/usr/bin/env python3

import math
import struct

import rclpy #Imports the python lib
from rclpy.node import Node #Imports the base class for making a ROS2 Node
from sensor_msgs.msg import Image, CameraInfo #Imports the standard message types


class FakeRgbdPublisherNode(Node): #Creating the node
    def __init__(self):
        super().__init__("fake_rgbd_publisher_node") #Giving the node a name

        self.declare_parameter("width", 320) #Defining paramemeters for the node, which can be set from the command line or a launch file
        self.declare_parameter("height", 240)
        self.declare_parameter("rate_hz", 10.0)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        #Creating publishers for the fake RGB_D node

        self.rgb_pub = self.create_publisher(
            Image, #Message type - Image
            "/iphone/rgb/image_raw", #Topic name
            10, #Queue depth, or 'buffer size' - how many messages to store if the subscriber is not keeping up
        )

        self.depth_pub = self.create_publisher(
            Image,
            "/iphone/depth/image_raw",
            10,
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            "/iphone/rgb/camera_info",
            10,
        )

        #Creating the timer to publish messages at the specified rate

        self.frame_index = 0

        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.publish_images)

        #Logging some info about the node when it starts up; this info can be accessed from the command line using 'ros2 node info /fake_rgbd_publisher_node' or from rqt_console
        self.get_logger().info(
            f"Publishing fake RGB-D at {self.rate_hz} Hz, "
            f"resolution {self.width}x{self.height}"
        )

    def publish_images(self):
        now = self.get_clock().now().to_msg() #we're using the current OS time

        rgb_msg = self.make_rgb_image(now)
        depth_msg = self.make_depth_image(now)
        camera_info_msg = self.make_camera_info(now)

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(camera_info_msg)

        self.frame_index += 1

    def make_rgb_image(self, stamp):
        msg = Image()
        msg.header.stamp = stamp #stamping the image with the current time, which can be used for synchronization with other messages
        msg.header.frame_id = "iphone_rgb_frame"

        msg.height = self.height
        msg.width = self.width
        msg.encoding = "rgb8" #this means each pixel is represented by 3 bytes (R, G, B)
        msg.is_bigendian = False
        msg.step = self.width * 3

        data = bytearray()

        for y in range(self.height):
            for x in range(self.width):
                r = int(255 * x / self.width)
                g = int(255 * y / self.height)
                b = int(127 + 127 * math.sin(self.frame_index * 0.1))
                data.extend([r, g, b])

        msg.data = bytes(data)
        return msg

    def make_depth_image(self, stamp):
        # Function to create a fake depth image where the depth value changes across the image and over time, simulating a dynamic scene. Each pixel's depth is calculated based on its x-coordinate and a sine wave that changes with the frame index, creating a wavy pattern that evolves over time.
        # It takes in self and stamp as arguments, where self is the instance of the class and stamp is the timestamp for the image. The function returns a sensor_msgs/Image message containing the depth data.
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "iphone_depth_frame"

        msg.height = self.height
        msg.width = self.width
        msg.encoding = "32FC1" #this means each pixel is represented by a single 32-bit float, which is common for depth images
        msg.is_bigendian = False
        msg.step = self.width * 4

        data = bytearray()

        for y in range(self.height):
            for x in range(self.width):
                depth_m = 0.5 + 1.5 * (x / self.width)
                depth_m += 0.1 * math.sin(self.frame_index * 0.1 + y * 0.05)
                data.extend(struct.pack("f", depth_m))

        msg.data = bytes(data)
        return msg

    def make_camera_info(self, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = "iphone_rgb_frame"

        msg.height = self.height
        msg.width = self.width

        fx = 300.0
        fy = 300.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]

        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FakeRgbdPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()