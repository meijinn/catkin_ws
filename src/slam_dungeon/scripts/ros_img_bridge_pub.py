#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

def publish_image():
    image_pub = rospy.Publisher("image_raw", Image, queue_size=10)
    bridge = CvBridge()
    capture = cv2.VideoCapture("/dev/video0")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fromX = 0 #対象範囲開始位置 X座標
    fromY = 0 #対象範囲開始位置 Y座標
    ToX = width #対象範囲終了位置 X座標
    ToY = height #対象範囲終了位置 Y座標

    rgb_pub = rospy.Publisher("RGB", Float32MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        # Capture a frame
        ret, img = capture.read()
        imgBox = img[fromY: ToY, fromX: ToX]
        b = imgBox.T[0].flatten().mean()
        g = imgBox.T[1].flatten().mean()
        r = imgBox.T[2].flatten().mean()

        if not ret:
            rospy.ERROR("Could not grab a frame!")
            break
        # Publish the image to the topic image_raw
        try:
            img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
            rgb_msg = []
            rgb_msg += [b, g, r]
            rgb_msg4pub = Float32MultiArray(data=rgb_msg)
            image_pub.publish(img_msg)
            rgb_pub.publish(rgb_msg4pub)

            print("B: %.2f" % (b))
            print("G: %.2f" % (g))
            print("R: %.2f" % (r))
        except CvBridgeError as error:
            print(error)

if __name__=="__main__":
    rospy.init_node("my_cam", anonymous=True)
    print("Image is being published to the topic /image_raw ...")
    publish_image()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
