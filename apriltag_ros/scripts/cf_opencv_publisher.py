#!/usr/bin/env python3

import rclpy
import time
import socket,struct, time
import numpy as np
import cv2

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class cf_image_publisher(Node):

    def __del__(self):
        self.client_socket.close()

    def __init__(self):
        super().__init__(
            "cf_image_publisher",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.client_socket = socket.socket()
        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)
        self.name = self._ros_parameters['camera_name']
        self.publisher_ = self.create_publisher(Image, self.name + '/image', 10)
        self.publisher_info = self.create_publisher(CameraInfo, self.name + '/camera_info', 10)

    def _param_to_dict(self, param_ros):
        """
        Turn ROS2 parameters from the node into a dict
        """
        tree = {}
        for item in param_ros:
            t = tree
            for part in item.split('.'):
                if part == item.split('.')[-1]:
                    t = t.setdefault(part, param_ros[item].value)
                else:
                    t = t.setdefault(part, {})
        return tree
    
def rx_bytes(size, client_socket):
    data = bytearray()
    while len(data) < size:
        data.extend(client_socket.recv(size-len(data)))
    return data

def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = cf_image_publisher()
    bridge = CvBridge()

    ip = minimal_publisher._ros_parameters['robots'][minimal_publisher.name]['ip_add']
    port = minimal_publisher._ros_parameters['robots'][minimal_publisher.name]['port']

    minimal_publisher.get_logger().info("Connecting to socket on " + ip + ":" + str(port))
    minimal_publisher.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # minimal_publisher.client_socket.connect((ip, port))
    # minimal_publisher.client_socket.settimeout(3)
    
    while True:
        try:
            minimal_publisher.client_socket.connect((ip, port))
            break
        except socket.error:
            minimal_publisher.get_logger().error("Connection Failed, Retrying..")
            time.sleep(1)
            
    minimal_publisher.get_logger().info("Socket connected")

    cam_info = CameraInfo()
    cam_info.width = minimal_publisher._ros_parameters['image_width']
    cam_info.height = minimal_publisher._ros_parameters['image_height']
    cam_info.distortion_model = minimal_publisher._ros_parameters['distortion_model']
    cam_mats = minimal_publisher._ros_parameters['camera_matrix']['data']
    for i in range(len(cam_mats)):    
        cam_info.k[i] = cam_mats[i]
    
    distortion_mats = minimal_publisher._ros_parameters['distortion_coefficients']['data']
    for i in range(len(distortion_mats)):    
        cam_info.d.append(distortion_mats[i])
    
    projection_mats = minimal_publisher._ros_parameters['projection_matrix']['data']
    for i in range(len(projection_mats)):
        cam_info.p[i] = projection_mats[i]

    rectification_mats = minimal_publisher._ros_parameters['rectification_matrix']['data']
    for i in range(len(rectification_mats)):
        cam_info.r[i] = rectification_mats[i]

    max_length = 15
    time_array = []
    previous = time.time()
    counter = 0

    while(1):

        # First get the info
        packetInfoRaw = rx_bytes(4, minimal_publisher.client_socket)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

        imgHeader = rx_bytes(length - 2, minimal_publisher.client_socket)
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        cam_info.header.stamp = minimal_publisher.get_clock().now().to_msg()
        minimal_publisher.publisher_info.publish(cam_info)

        if magic == 0xBC:
            #print("Magic is good")
            #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
            #print("Image format is {}".format(format))
            #print("Image size is {} bytes".format(size))

            # Now we start rx the image, this will be split up in packages of some size
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = rx_bytes(4, minimal_publisher.client_socket)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = rx_bytes(length - 2, minimal_publisher.client_socket)
                imgStream.extend(chunk)

            ratePerImage = 1 / ((time.time()-previous))

            time_array.append(ratePerImage)

            while (len(time_array) > max_length):
                time_array.pop(0)
            
            moving_average = 0.0
            for i in range(len(time_array)):
                moving_average = moving_average + time_array[i]
            
            moving_average = moving_average/len(time_array)

            # minimal_publisher.get_logger().info("ratePerImage " + str(meanTimePerImage))
            
            counter += 1
            counter %= 10
            if (counter == 0):
                minimal_publisher.get_logger().info("ratePerImage " + str(moving_average))

            # print("{}".format(1/ratePerImage))

            # if format == 0:
            bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
            bayer_img.shape = (244, 324)
            cv_image = bridge.cv2_to_imgmsg(bayer_img, 'mono8')
            cv_image.header.stamp = cam_info.header.stamp
            minimal_publisher.publisher_.publish(cv_image)

            previous = time.time()
        else:
            minimal_publisher.get_logger().info("no image")
        
        

if __name__ == "__main__":
    main()