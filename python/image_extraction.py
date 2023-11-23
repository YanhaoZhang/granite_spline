import os
import argparse

import cv2
import pandas as pd
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def extraction(bag_file,image_topic,output_dir, ts_start, ts_stop, compressed=True):
    """Extract a folder of images from a rosbag.
    """
    
    lst_ts = []
    lst_name = []
    os.makedirs(os.path.join(output_dir, "mav0/cam0/data"), exist_ok=True)

    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if compressed:
            cv_img = bridge.compressed_imgmsg_to_cv2(msg)
        else:
            cv_img = bridge.imgmsg_to_cv2(msg)
        if t.secs > ts_start and t.secs < ts_stop:
            ts = str(int(t.secs*1e9) + t.nsecs)
            name = ts+'.png'
            lst_ts.append(ts)
            lst_name.append(name)
            clahe = cv2.createCLAHE(clipLimit=4, tileGridSize=(8,8))
            cv_img = clahe.apply(cv_img)
            cv2.imwrite(os.path.join(output_dir, "mav0/cam0/data", name), cv_img)
            
    bag.close()
    columns = ['#timestamp [ns]', 'filename']
    df = pd.DataFrame(zip(lst_ts,lst_name), columns=columns)
    df.to_csv(os.path.join(output_dir, "mav0/cam0/data.csv"), index=False)
    return
            
    
    
extraction(bag_file='/path/to/rosbag.bag',
           image_topic='/arrc_long_range_flight_system/camera/image/compressed',
           output_dir='/home/name/output/directory', 
           ts_start=1683701836.700061392, 
           ts_stop=1683702403.554839056,  
           compressed=True)
