#!/usr/bin/env python3
# coding: utf-8

# In[5]:


import serial
import serial.tools.list_ports
import sys
import nest_asyncio
import time
import cv2
import keyboard
from pupil_labs.realtime_api.simple import Device

ip = "192.168.10.184"

"""Make sure that your PC and the Pupil Invisible device are connected to the same network!"""
"""The ip address of Pupil Invisible device may change at restarting each time. Check it in the settings!"""


def connect_pupil_invisible():
    device = Device(address=ip, port="8080")
    
    """You will see the device parameters below if the Pupil Invisible device is successfully connected"""

    print(f"Phone name: {device.phone_name}")
    print(f"Phone unique ID: {device.phone_id}")

    print(f"Battery level: {device.battery_level_percent}%")
    print(f"Battery state: {device.battery_state}")

    print(f"Free storage: {device.memory_num_free_bytes / 1024**3}GB")
    print(f"Storage level: {device.memory_state}")

    print(f"Connected glasses: SN {device.serial_number_glasses}")
    print(f"Connected scene camera: SN {device.serial_number_scene_cam}")  

    if device is None:
        print('Cannot find Pupil Invisible device. Check internet connection.')
        sys.exit
        
    return device
                            
           
def main():
    nest_asyncio.apply()
    device = connect_pupil_invisible()
    recording = False
    print('Ready to start/stop gaze acquisition...')
    
    try:
        while True:
            """The video and gaze will be displayed in a separate window shortly after running the code"""
            frame, gaze = device.receive_matched_scene_video_frame_and_gaze()
            cv2.circle(
                frame.bgr_pixels,
                (int(gaze.x), int(gaze.y)),
                radius=50,
                color=(0, 0, 255),
                thickness=15,
            )

            cv2.imshow("Scene camera with gaze overlay", frame.bgr_pixels)
            if cv2.waitKey(1) & 0xFF == 27:
                break 
                
            if keyboard.is_pressed('s'):
                """Press S on the keyboard to start recording"""
                if not recording:
                    print('Started gaze acquisition')
                    device.recording_start()
                    recording = True
                else: 
                    print('Already recording!')
                
            if keyboard.is_pressed('e'):
                """Press E on the keyboard to stop recording"""
                if recording:
                    print('Stopped gaze acquisition')
                    device.recording_stop_and_save()
                    recording = False  
                else: 
                    print('Not recording!')
                    
    except KeyboardInterrupt:
        pass                                
    finally:
        #print("Stopping...")
        device.close()  # explicitly stop auto-update
              
        
if __name__ == '__main__':
    main()    


# In[ ]:




