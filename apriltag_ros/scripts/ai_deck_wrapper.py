#!/usr/bin/env python3

import rclpy
import sys
#sys.path.append('/opt/ros/humble/lib/python3/dist-packages')
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
#from image_transport import ImageTransport, CameraPublisher

import time
import socket,os,struct,time
import numpy as np
import cv2
from cv_bridge import CvBridge

#wrapper node to take images from crazyflie ip and publish to ros2 topic of type sensor_msgs/Image
class AI_Deck_Wrapper(Node):
    def socketConnect(self):
        timeout = 3
        retryTime = 3

        self.client_socket.close()

        while(not self.connected):
            try:
                self.get_logger().info("Connecting to socket on {}:{}...".format(self.deck_ip, self.deck_port))
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.settimeout(timeout)
                self.client_socket.connect((self.deck_ip, self.deck_port))
                self.get_logger().info("Socket connected")
                self.connected = True
            except socket.error as msg:
                self.get_logger().info("socketConnect: {}".format(msg))

                time.sleep(retryTime)

    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            try:
                data.extend(self.client_socket.recv(size-len(data)))
            except socket.error as msg:
                self.get_logger().info("rx_bytes: {}".format(msg))
                self.connected = False
                break

        return data

    #maybe this all can be a rcvImage function (yes it can)
    def rcvImage(self):
        while(1):
            if not self.connected:
                self.socketConnect()

            # First get the info
            packetInfoRaw = self.rx_bytes(4)

            if not self.connected:
                continue #if rx_bytes failed, reconnect

            #self.get_logger().info(packetInfoRaw)
            try:
                [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
            except Exception as e:
                self.get_logger().info("Error unpacking [length, routing, function]: {}".format(e))
                self.connected = False
                continue

            #self.get_logger().info("Length is {}".format(length))
            #self.get_logger().info("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
            #self.get_logger().info("Function is 0x{:02X}".format(function))

            imgHeader = self.rx_bytes(length - 2)

            if not self.connected:
                continue #if rx_bytes failed, reconnect

            #self.get_logger().info(imgHeader)
            #self.get_logger().info("Length of data is {}".format(len(imgHeader)))
            try:
                [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)
            except Exception as e:
                self.get_logger().info("Error unpacking [magic, width, height, depth, format, size]: {}".format(e))
                self.connected = False
                continue

            if magic == 0xBC:
                #self.get_logger().info("Magic is good")
                #self.get_logger().info("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
                #self.get_logger().info("Image format is {}".format(format))
                #self.get_logger().info("Image size is {} bytes".format(size))

                # Now we start rx the image, this will be split up in packages of some size
                imgStream = bytearray()

                while len(imgStream) < size:
                    packetInfoRaw = self.rx_bytes(4)

                    if not self.connected:
                        break #if rx_bytes failed, reconnect

                    try:
                        [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                    except:
                        self.get_logger().info("Error unpacking [length, dst, src]: {}".format(e))
                        self.connected = False
                        break

                    #self.get_logger().info("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                    chunk = self.rx_bytes(length - 2)

                    if not self.connected:
                        break #if rx_bytes failed, break out of the while loop and reconnect after that

                    imgStream.extend(chunk)

                if not self.connected:
                    continue #if rx_bytes failed, reconnect

                self.count = self.count + 1
                self.count = self.count % 100
                #meanTimePerImage = (time.time()-self.start) / self.count
                #self.get_logger().info("{}".format(meanTimePerImage))
                #self.get_logger().info("{}".format(1/meanTimePerImage))

                self.imgdata = imgStream

            '''
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.data = self.imgdata
            #CompressedImage msg
            msg.format = 'jpeg'
            '''

            #Image msg
            # msg.height = height
            # msg.width = width
            # msg.encoding = format
            # msg.is_bigendian = False
            # msg.step = size


            # bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
            # bayer_img.shape = (244, 324)
            # cv_image = self.bridge.cv2_to_imgmsg(bayer_img, 'mono8')
            # cv_image.header.stamp = self.cam_info.header.stamp
            # self.publisher_.publish(cv_image)

            img_stream = np.frombuffer(imgStream, dtype=np.uint8)

            if format == 0:
                img_stream.shape = (244, 324)
                #self.get_logger().info("RAW")
                cv_image = self.bridge.cv2_to_imgmsg(img_stream, 'mono8')
            else:
                img_decoded = cv2.imdecode(img_stream, cv2.IMREAD_UNCHANGED)
                img_decoded.shape = (244, 324)
                #self.get_logger().info("JPEG")
                cv_image = self.bridge.cv2_to_imgmsg(img_decoded, 'mono8')

            #set timestamps
            self.cam_info.header.stamp = self.get_clock().now().to_msg()
            cv_image.header.stamp = self.cam_info.header.stamp

            self.publisher_.publish(cv_image)
            self.publisher_info.publish(self.cam_info)
            #self.publisher_.publish(msg)
            #self.publisher_.publish(msg, self.cam_info)
            #self.get_logger().info('Publishing Image: "%s"' % msg.header.stamp)
            #self.get_logger().info('Publishing Camera: "%s"' % self.cam_info.header.stamp)

            #display image
            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
                bayer_img.shape = (244, 324)
                # color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                cv2.putText(bayer_img, "Count: {:.1f}".format(self.count), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow(self.name, bayer_img)
                #cv2.imshow('Color', color_img)
                cv2.waitKey(1)
            else:
                #with open("img.jpeg", "wb") as f:
                #    f.write(imgStream)
                nparr = np.frombuffer(imgStream, np.uint8)
                decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                cv2.putText(decoded, "Count: {:.1f}".format(self.count), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow(self.name + " JPEG", decoded)
                cv2.waitKey(1)

    def __init__(self):
        # Init from cf_opencv_publisher
        super().__init__(
            "cf_image_publisher",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.client_socket = socket.socket()

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)

        self.name = self._ros_parameters['camera_name']
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.publisher_info = self.create_publisher(CameraInfo, 'camera_info', 10)

        # Init from cf_opencv_publisher main
        self.deck_ip = self._ros_parameters['robots'][self.name]['ip_add']
        self.deck_port = self._ros_parameters['robots'][self.name]['port']

        # Original init
        # super().__init__('ai_deck_wrapper')
        # self.declare_parameter("ip", "192.168.1.106")
        # self.declare_parameter("port", 5000)
        # self.declare_parameter("name", "cf18")

        # self.deck_ip = self.get_parameter('ip')
        # self.timer_period = self.get_parameter('period')
        # self.name = self.get_parameter('name')

        # self.publisher_ = self.create_publisher(Image, '/image_rect', 10)
        # #self.publisher = self.image_transport.advertiseCamera('/image_rect', CompressedImage, 'jpeg')
        # #self.publisher_info = self.create_publisher(CameraInfo, self.name.value + '/camera_info', 10)
        # self.publisher_info = self.create_publisher(CameraInfo, '/camera_info', 10)

        self.imgdata = None
        self.data_buffer = bytearray()
        self.count = 0
        self.start = time.time()
        self.bridge = CvBridge()

        self.cam_info = CameraInfo()
        self.cam_info.width = 324
        self.cam_info.height = 244
        self.cam_info.distortion_model = "plumb_bob"
        cam_mats = [180.049252, 0.000000, 169.270200, 0.000000, 180.528259, 160.598796, 0.000000, 0.000000, 1.000000]
        for i in range(len(cam_mats)):
            self.cam_info.k[i] = cam_mats[i]

        distortion_mats = [-0.077304, -0.006814, 0.000832, 0.000674, 0.000000]
        for i in range(len(distortion_mats)):
            self.cam_info.d.append(distortion_mats[i])

        projection_mats = [164.769073, 0.000000, 170.861434, 0.000000, 0.000000, 169.009918, 163.618060, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        for i in range(len(projection_mats)):
            self.cam_info.p[i] = projection_mats[i]

        rectification_mats = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
        for i in range(len(rectification_mats)):
            self.cam_info.r[i] = rectification_mats[i]

        self.connected = False
        self.rcvImage()

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

    def __del__(self):
        self.client_socket.close()

def main(args=None):
    rclpy.init(args=args)

    ai_deck_wrapper = AI_Deck_Wrapper()

    rclpy.spin(ai_deck_wrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ai_deck_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
