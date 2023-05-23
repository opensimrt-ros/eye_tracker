#!/usr/bin/env python3
# coding: utf-8


# The two lines below are only needed to execute this code in a Jupyter Notebook
import nest_asyncio
nest_asyncio.apply()

from pupil_labs.realtime_api.simple import Device, discover_one_device
#device = discover_one_device()
device = Device(address="192.168.1.101", port="8080")

print(f"Phone IP address: {device.phone_ip}")
print(f"Phone name: {device.phone_name}")
print(f"Battery level: {device.battery_level_percent}%")
print(f"Free storage: {device.memory_num_free_bytes / 1024**3:.1f} GB")
print(f"Serial number of connected glasses: {device.serial_number_glasses}")

import time

from datetime import datetime
import matplotlib.pyplot as plt
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
rospy.init_node("eye")

bridge = CvBridge()
image_pub = rospy.Publisher("image_topic", Image)
#dt_gaze = datetime.fromtimestamp(gaze_sample.timestamp_unix_seconds)
#dt_scene = datetime.fromtimestamp(scene_sample.timestamp_unix_seconds)
#print(f"This gaze sample was recorded at {dt_gaze}")
#print(f"This scene video was recorded at {dt_scene}")
#print(f"Temporal difference between both is {abs(gaze_sample.timestamp_unix_seconds - scene_sample.timestamp_unix_seconds) * 1000:.1f} ms")
while not rospy.is_shutdown():
    start = time.time()
    scene_sample, gaze_sample = device.receive_matched_scene_video_frame_and_gaze()
    scene_image_rgb = cv2.cvtColor(scene_sample.bgr_pixels, cv2.COLOR_BGR2RGB)
    try:
        #image_message = bridge.cv2_to_imgmsg(scene_image_rgb, "passthrough")
        image_message = bridge.cv2_to_imgmsg(scene_sample, "passthrough")
    except CvBridgeError as e:
        print(e)
    image_pub.publish(image_message)
    stop = time.time()
    print(stop-start)
    #cv2.imshow("preview", scene_image_rgb)
#plt.figure(figsize=(10, 10))
#plt.imshow(scene_image_rgb)
#plt.scatter(gaze_sample.x, gaze_sample.y, s=200, facecolors='none', edgecolors='r')
#plt.show(block=True)
