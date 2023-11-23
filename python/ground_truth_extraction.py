import os
# import argparse
#
# import cv2
# import pandas as pd
import rosbag
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
import sys, csv

def extraction(bag_file,gt_topic,output_dir, ts_start, ts_stop):
    """Extract a folder of images from a rosbag.
    """
    
    lst_ts = []
    lst_name = []
    os.makedirs(os.path.join(output_dir), exist_ok=True)

    bag = rosbag.Bag(bag_file, "r")
    filename = output_dir + '/' + 'data.csv'
    with open(filename, 'w+') as csvfile:
        filewriter = csv.writer(csvfile, delimiter=',')
        firstIteration = True  # allows header row

        count = 0
        for topic, msg, t in bag.read_messages(topics=[gt_topic]):
            msgString = str(msg)
            msgList = msgString.split('\n')
            instantaneousListOfData = []
            for nameValuePair in msgList:
                splitPair = nameValuePair.split(':')
                for i in range(len(splitPair)):  # should be 0 to 1
                    splitPair[i] = splitPair[i].strip()
                instantaneousListOfData.append(splitPair)

            if firstIteration: # header
                headers_gt = ["#timestamp [ns]", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]",
                              "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []"]
                filewriter.writerow(headers_gt)
                firstIteration = False

            # write data
            if True: #t.secs > ts_start and t.secs < ts_stop:
                ts = str(int(t.secs*1e9) + t.nsecs)

                values = [str(ts)]  # first column will have rosbag timestamp
                for pair in instantaneousListOfData:
                    if len(pair) > 1:
                        values.append(pair[1])

                # values_gt = [values[0], values[9], values[10], values[11], values[13], values[14], values[15], values[16]]   #[time, x,y,z, qx, qy, qz, qw]
                values_gt = [values[0], values[9], values[10], values[11],
                             values[16], values[13], values[14], values[15]]  #[time, x,y,z, qw, qx, qy, qz]
                filewriter.writerow(values_gt)
        bag.close()

    return
            
    
    
extraction(bag_file='/media/yanhao/8tb1/00_data_TII/island_localization/cv-assignment-1.bag',
           gt_topic='/apm/mavros/local_position/pose',
           output_dir='/media/yanhao/8tb1/00_data_TII/island_localization/frames/ground_truth',
           ts_start=1683701468.0,
           ts_stop =1683701689.0)

# 1683701836.700061392

# 1683701467.439037888
# 1683701688.899556464
# 1683701468.001151088
# 1683701688.899556464