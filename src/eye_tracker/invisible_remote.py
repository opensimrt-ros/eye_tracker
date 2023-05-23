#!/usr/bin/env python3
# coding: utf-8

import cv2
import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from sys import exit
import nest_asyncio
from pupil_labs.realtime_api.simple import Device, discover_one_device

rospy.init_node("eye")

nest_asyncio.apply()
#TODO: params? maybe someone wants to know this. 
device = Device(address="192.168.1.101", port="8080")

print(f"Phone IP address: {device.phone_ip}")
print(f"Phone name: {device.phone_name}")
print(f"Battery level: {device.battery_level_percent}%")
print(f"Free storage: {device.memory_num_free_bytes / 1024**3:.1f} GB")
print(f"Serial number of connected glasses: {device.serial_number_glasses}")

bridge = CvBridge()
image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
image_overlay_pub = rospy.Publisher("image_overlay_topic", Image, queue_size=1)
gaze_pub = rospy.Publisher("gaze", RegionOfInterest, queue_size=1)
gaze_size = 20

rate = rospy.Rate(60)

#TODO: node is not shutting down as expected. 
print("use ctrl-z to quit")

while not rospy.is_shutdown():
    try:
        scene_sample, gaze_sample = device.receive_matched_scene_video_frame_and_gaze()
        try:
            image_message = bridge.cv2_to_imgmsg(scene_sample.bgr_pixels, "passthrough")
        except CvBridgeError as e:
            print(e)
            break
        image_pub.publish(image_message)
        image = cv2.circle(scene_sample.bgr_pixels, (int(gaze_sample.x), int(gaze_sample.y)), gaze_size, (0,0,255), 4)
        try:
            image_message = bridge.cv2_to_imgmsg(image, "passthrough")
        except CvBridgeError as e:
            print(e)
            break
        image_overlay_pub.publish(image_message)
        gaze_msg = RegionOfInterest()
        gaze_msg.x_offset = gaze_sample.x - gaze_size/2
        gaze_msg.y_offset = gaze_sample.y - gaze_size/2
        gaze_msg.height = gaze_size
        gaze_msg.width = gaze_size
        gaze_pub.publish(gaze_msg)
        rate.sleep() ## maybe this helps break the node?
    except KeyboardInterrupt:
        exit()
